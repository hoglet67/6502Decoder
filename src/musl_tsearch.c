/* musl as a whole is licensed under the following standard MIT license: */

/* ---------------------------------------------------------------------- */
/* Copyright © 2005-2020 Rich Felker, et al. */

/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the */
/* "Software"), to deal in the Software without restriction, including */
/* without limitation the rights to use, copy, modify, merge, publish, */
/* distribute, sublicense, and/or sell copies of the Software, and to */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions: */

/* The above copyright notice and this permission notice shall be */
/* included in all copies or substantial portions of the Software. */

/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */
/* ---------------------------------------------------------------------- */

#include <stdlib.h>
#include "musl_tsearch.h"

/* AVL tree height < 1.44*log2(nodes+2)-0.3, MAXH is a safe upper bound.  */
#define MAXH (sizeof(void*)*8*3/2)

struct node {
	const void *key;
	void *a[2];
	int h;
};

void ttdestroy(void *root, void (*freekey)(void *))
{
	struct node *r = root;

	if (r == 0)
		return;
	ttdestroy(r->a[0], freekey);
	ttdestroy(r->a[1], freekey);
	if (freekey) freekey((void *)r->key);
	free(r);
}

void *ttfind(const void *key, void *const *rootp,
	int(*cmp)(const void *, const void *))
{
	if (!rootp)
		return 0;

	struct node *n = *rootp;
	for (;;) {
		if (!n)
			break;
		int c = cmp(key, n->key);
		if (!c)
			break;
		n = n->a[c>0];
	}
	return n;
}

static inline int height(struct node *n) { return n ? n->h : 0; }

static int rot(void **p, struct node *x, int dir /* deeper side */)
{
	struct node *y = x->a[dir];
	struct node *z = y->a[!dir];
	int hx = x->h;
	int hz = height(z);
	if (hz > height(y->a[dir])) {
		/*
		 *   x
		 *  / \ dir          z
		 * A   y            / \
		 *    / \   -->    x   y
		 *   z   D        /|   |\
		 *  / \          A B   C D
		 * B   C
		 */
		x->a[dir] = z->a[!dir];
		y->a[!dir] = z->a[dir];
		z->a[!dir] = x;
		z->a[dir] = y;
		x->h = hz;
		y->h = hz;
		z->h = hz+1;
	} else {
		/*
		 *   x               y
		 *  / \             / \
		 * A   y    -->    x   D
		 *    / \         / \
		 *   z   D       A   z
		 */
		x->a[dir] = z;
		y->a[!dir] = x;
		x->h = hz+1;
		y->h = hz+2;
		z = y;
	}
	*p = z;
	return z->h - hx;
}

/* balance *p, return 0 if height is unchanged.  */
static int __tsearch_balance(void **p)
{
	struct node *n = *p;
	int h0 = height(n->a[0]);
	int h1 = height(n->a[1]);
	if (h0 - h1 + 1u < 3u) {
		int old = n->h;
		n->h = h0<h1 ? h1+1 : h0+1;
		return n->h - old;
	}
	return rot(p, n, h0<h1);
}

void *ttsearch(const void *key, void **rootp,
	int (*cmp)(const void *, const void *))
{
	if (!rootp)
		return 0;

	void **a[MAXH];
	struct node *n = *rootp;
	struct node *r;
	int i=0;
	a[i++] = rootp;
	for (;;) {
		if (!n)
			break;
		int c = cmp(key, n->key);
		if (!c)
			return n;
		a[i++] = &n->a[c>0];
		n = n->a[c>0];
	}
	r = malloc(sizeof *r);
	if (!r)
		return 0;
	r->key = key;
	r->a[0] = r->a[1] = 0;
	r->h = 1;
	/* insert new node, rebalance ancestors.  */
	*a[--i] = r;
	while (i && __tsearch_balance(a[--i]));
	return r;
}

void *ttdelete(const void *restrict key, void **restrict rootp,
	int(*cmp)(const void *, const void *))
{
	if (!rootp)
		return 0;

	void **a[MAXH+1];
	struct node *n = *rootp;
	struct node *parent;
	struct node *child;
	int i=0;
	/* *a[0] is an arbitrary non-null pointer that is returned when
	   the root node is deleted.  */
	a[i++] = rootp;
	a[i++] = rootp;
	for (;;) {
		if (!n)
			return 0;
		int c = cmp(key, n->key);
		if (!c)
			break;
		a[i++] = &n->a[c>0];
		n = n->a[c>0];
	}
	parent = *a[i-2];
	if (n->a[0]) {
		/* free the preceding node instead of the deleted one.  */
		struct node *deleted = n;
		a[i++] = &n->a[0];
		n = n->a[0];
		while (n->a[1]) {
			a[i++] = &n->a[1];
			n = n->a[1];
		}
		deleted->key = n->key;
		child = n->a[0];
	} else {
		child = n->a[1];
	}
	/* freed node has at most one child, move it up and rebalance.  */
	free(n);
	*a[--i] = child;
	while (--i && __tsearch_balance(a[i]));
	return parent;
}

static void walk(const struct node *r, void (*action)(const void *, TVISIT, int), int d)
{
	if (!r)
		return;
	if (r->h == 1)
		action(r, tleaf, d);
	else {
		action(r, tpreorder, d);
		walk(r->a[0], action, d+1);
		action(r, tpostorder, d);
		walk(r->a[1], action, d+1);
		action(r, tendorder, d);
	}
}

void ttwalk(const void *root, void (*action)(const void *, TVISIT, int))
{
	walk(root, action, 0);
}
