#ifndef HEADER_DBD91B1DA09E43B1B27A0CE43D8B5E7B
#define HEADER_DBD91B1DA09E43B1B27A0CE43D8B5E7B

typedef enum { tpreorder, tpostorder, tendorder, tleaf } TVISIT;

void ttdestroy(void *root, void (*freekey)(void *));

void *ttfind(const void *key, void *const *rootp, int(*cmp)(const void *, const void *));

void *ttsearch(const void *key, void **rootp, int (*cmp)(const void *, const void *));

void *ttdelete(const void *restrict key, void **restrict rootp, int(*cmp)(const void *, const void *));

void ttwalk(const void *root, void (*action)(const void *, TVISIT, int));

#endif
