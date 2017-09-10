# TBI_transmitter
This repositor has the main firmware files for a TBI instrument developed to complete an MRes degree at Imperial College London

BTE.h,BTE.c,Board.h: Are the customly developed board drivers - defining IO,ADC, Power, RF circuits and other periperals.

Transmitter.c: contains the application thread of the wireless wearable instrument.

Reciever.c: is the firmware run on the reciver for notifcation parsaing and processing before sending to the remote server.


These files are based on TI-cc2650/40 and used TI-RTOS libraries
