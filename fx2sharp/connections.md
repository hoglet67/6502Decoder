
# Notes on fx2sharp

To get this to work the Cypress cyusb3 driver needs to be installed on the relevant VID:PID (I got mine from the SDK C:\Program Files (x86)\Cypress\EZ-USB FX3 SDK\1.3\driver\bin\Win10\x64)

# Connections

The connections that are required to hook an LCSoft (or clone) board to fx2sharp:


## To configure the FIFO
	PA5	high (FIFOADR1)		+
	PA4	low (FIFOADR0)
	PA2	high (SLOE)		+
	RDY0	high (SLRD)		+

pins marked + can be left unconnected, they should have pullups!?!

## The 6502 negative going Phi2 clock
	RDY1	Phi2 (SLWR)

## Other connections as required

Then almost the same connections as before for the signals being captured:
	PB7..0 	D7..0
	PD0	RnW
	PD1	Sync
	PD2	Rdy
	PD3	unconnected (was Phi2, which is now connected to Rdy1)
	PD4	nIRQ
	PD5	nNMI
	PD6	nRST
	PD7	unconnected

see post https://stardot.org.uk/forums/viewtopic.php?p=186850#p186850

