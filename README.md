# softrftest
Small test program that activates SoftRF transmit

Compile this test program on your host
```
gcc main.c -I./mavlink -o mav_uart
```

and start it
```
./mav_uart /dev/ttyACM0
```

Output
```
[REQUEST_DATA_STREAM] stream=2 rate=2 start=1
[REQUEST_DATA_STREAM] stream=12 rate=2 start=1
[ADSB] ICAO=076543  lat=12.3400000  lon=56.7800000  alt=456000
[ADSB] ICAO=076ABC  lat=45.6700000  lon=11.1111111  alt=123000
...
```
