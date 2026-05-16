# softrftest
Small test program that was originally developed to activates SoftRF transmit
Now it is used to test NoSoftRF firmware

Attach your XR1 receiver to your host computer over a USB UART dogle
Flash it with NoSoftRF firmware

Compile main.c test program on your host
```
gcc main.c -I./mavlink -o mav_uart
```

and start it
```
./mav_uart -p /dev/ttyACM0 -b 115200 --lat 12.34 --lon 56.78
```
Note: choose additional options according to the options selected in the Web Gui

Output
```
[REQUEST_DATA_STREAM] stream=2 rate=2 start=1
[REQUEST_DATA_STREAM] stream=12 rate=2 start=1
time=1778676047700 ICAO=FC8DC4 CALL=FAFC8DC4 lat=12.34 lon=56.78 alt=100000 head=7453 hvel=97 vvel=0 at=0 et=14 squawk=7000
time=1778676050348 ICAO=FC8DC4 CALL=FAFC8DC4 lat=12.35 lon=56.79 alt=100000 head=7453 hvel=97 vvel=0 at=0 et=14 squawk=7000
...
```

If you selected MSP instead of the MAVLink in the Gui then do this:

Compile main_msp.c test program on your host:
```
g++ main_msp.c msp_protocol.cpp -o msp_uart
```

and start it:
```
./msp_uart -p /dev/ttyUSB0 -b 115200 --lat 12.34 --lon 65.78
```

Q: Why are the --lat and --lon parameters needed?

A: FLARM packets contain relative coordinates, so the transmitter and receiver must
   not be too far apart. If you use these test programs to receive FLARM data, specify
   your current location on the command line. The coordinates do not need to be
   exact — deviations of up to ~20 km should be fine.
