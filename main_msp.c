/*
 * MAVLink UART example program (MAVLink v2/v1)
 *
 * Build:
 *   gcc -I./mavlink_c -o mav_uart -lm mav_uart.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <time.h>
#include <getopt.h>

#include "msp_protocol.h"


#define DEFAULT_COM_PORT  "/dev/ttyUSB0"
#define DEFAULT_COM_SPEED 115200
//#define DEFAULT_COM_SPEED 57600
#define DEFAULT_LAT 45.67 // fake lat
#define DEFAULT_LON 12.34 // fake lon
#define DEFAULT_ALT 500.0 // fake alt

int32_t g_latitude  = 0;
int32_t g_longitude = 0;
int32_t g_altitude  = 0;

// ------------------------------------------------------------
// MAP INTEGER BAUD RATE TO termios CONSTANT
// ------------------------------------------------------------
speed_t baud_to_termios(int baud)
{
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:
            return 0;   // unsupported baud
    }
}


// ------------------------------------------------------------
// UART INIT
// ------------------------------------------------------------
int uart_init(const char *device, int baudrate)
{
    speed_t br = baud_to_termios(baudrate);
    if (br == 0) {
        return -1;
    }

    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        return -2;
    }

    struct termios config;
    tcgetattr(fd, &config);

    cfsetispeed(&config, br);
    cfsetospeed(&config, br);

    config.c_cflag |= (CLOCAL | CREAD);
    config.c_cflag &= ~CSIZE;
    config.c_cflag |= CS8;
    config.c_cflag &= ~PARENB;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CRTSCTS;

    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    config.c_oflag &= ~OPOST;
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcsetattr(fd, TCSANOW, &config);

    return fd;
}



//-------------------------------------------------------
// BASIC 1 Hz SEND FUNCTIONS
//-------------------------------------------------------
uint64_t get_system_time_usec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    uint64_t time_unix_usec =
        (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;

    return time_unix_usec;
}


static uint16_t cdeg = 7500;

void send_gps_raw(int fd)
{
    msp_message_t msgout;
    uint8_t buf[MSP_MAX_PAYLOAD];

    msp_gps_raw_int_t gpsInfo;
    gpsInfo.time_usec = get_system_time_usec();
    gpsInfo.lat = g_latitude;
    gpsInfo.lon = g_longitude;
    gpsInfo.alt = g_altitude;
    gpsInfo.eph = 100;
    gpsInfo.epv = 100;
    gpsInfo.vel = 100;
    gpsInfo.cog = cdeg;
    gpsInfo.fix_type = 3;
    gpsInfo.satellites_visible = 8;
    gpsInfo.alt_ellipsoid = 0;
    gpsInfo.h_acc = 0;
    gpsInfo.v_acc = 0;
    gpsInfo.vel_acc = 0;
    gpsInfo.hdg_acc = 0;
    gpsInfo.yaw = 0;

    msp_msg_gps_raw_int_response_encode(&msgout, &gpsInfo);

    write(fd, buf, msp_msg_to_send_buffer(buf, &msgout));
}


//-------------------------------------------------------
// PARSER
//-------------------------------------------------------
void parse_message(const msp_message_t *msgin, int fd)
{
    switch (msgin->function)
    {
        case MSP2_INAV_GPS_RAW_INT:
//            printf("gps_raw_int_request\n");
            send_gps_raw(fd);
            break;
        case MSP2_SENSOR_ADSB: {
            msp_adsb_vehicle_t adsb;
            msp_msg_adsb_vehicle_decode(msgin, &adsb);
            uint64_t sysTime = get_system_time_usec()/1000; // in ms
            printf("time=%" PRIu64 " ICAO=%06X CALL=%s lat=%.7f lon=%.7f alt=%d head=%d hvel=%d vvel=%d at=%d et=%d squawk=%d\n",
                   sysTime,
                   adsb.ICAO_address,
                   adsb.callsign, /*<  The callsign, 8+null*/
                   adsb.lat / 1e7,
                   adsb.lon / 1e7,
                   adsb.altitude,
                   adsb.heading, /*< [cdeg] Course over ground*/
                   adsb.hor_velocity, /*< [cm/s] The horizontal velocity*/
                   adsb.ver_velocity, /*< [cm/s] The vertical velocity. Positive is up*/
                   adsb.altitude_type, /*<  ADSB altitude type.*/
                   adsb.emitter_type, /*<  ADSB emitter type.*/
                   adsb.squawk/*<  ADSB squawk.*/
                  );
            fflush(stdout);
            break;
        }

        default:
            printf("Message id: %d\n", msgin->function);
            break;

    }
}

void show_usage(char* prog_name)
{
    fprintf(stderr, "\nUsage: %s -p PORT [-b BAUD] [--lat LAT --lon LON]\n", prog_name);
    fprintf(stderr, "Defaults: -b 115200 --lat 45.67 --lon 12.34\n");
}

//-------------------------------------------------------
// MAIN
//-------------------------------------------------------
int main(int argc, char *argv[])
{
    const char* port = DEFAULT_COM_PORT;
    int baudrate = DEFAULT_COM_SPEED;
    double lat = DEFAULT_LAT;
    double lon = DEFAULT_LON;
    double alt = DEFAULT_ALT;

    int opt;
    static struct option long_options[] = {
        {"port",     required_argument, 0, 'p'},
        {"baudrate", required_argument, 0, 'b'},
        {"lat",      required_argument, 0, 1},
        {"lon",      required_argument, 0, 2},
        {"alt",      required_argument, 0, 3},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "p:b:", long_options, NULL)) != -1) {
        switch(opt) {
            case 'p': port = optarg; break;
            case 'b': baudrate = atoi(optarg); break;
            case 1: lat = atof(optarg); break;
            case 2: lon = atof(optarg); break;
            case 3: alt = atof(optarg); break;
            default:
                show_usage(argv[0]);
                return 1;
        }
    }

    if (!port) {
        fprintf(stderr, "Error: Port is required\n");
        show_usage(argv[0]);
        return 1;
    }

    int fd = uart_init(port, baudrate);
    if (fd < 0) {
        if (fd == -1) {
          printf("Error: Unsupported baudrate: %d\n", baudrate);
        }
        else if (fd == -2) {
          fprintf(stderr, "Error: Port %s cannot be opened\n", port);
        }
        show_usage(argv[0]);
        return 1;
    }

    printf("UART Port: %s\n", port);
    printf("Baud: %d\n", baudrate);
    printf("GPS: %.6f, %.6f\n", lat, lon);
    printf("ALT: %.0f", alt);

    g_latitude  = (int32_t)(lat * 1e7);
    g_longitude = (int32_t)(lon * 1e7);
    g_altitude  = (int32_t)(alt * 1000); // m → millimeters

    msp_message_t msg;

    uint64_t next_tick = 0;

    while (1) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_ms = ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;

        // initialize schedule once
        if (next_tick == 0) {
            next_tick = ((now_ms / 1000) + 1) * 1000;
        }

        //---- FIXED 1Hz MESSAGES ----
        if (now_ms >= next_tick) {
            //send_gps_raw(fd);
            next_tick += 1000;
        }

        //---- RECEIVE UNTIL NEXT TICK ----
        uint64_t now_poll = now_ms;
        while (now_poll < next_tick) {
            uint8_t c;
            ssize_t r = read(fd, &c, 1);
            if (r > 0) {
                if (msp_parse_char(c, &msg))
                    parse_message(&msg, fd);
            } else {
                // No data; optional tiny sleep to prevent busy spin
                usleep(1000); // 1ms
            }

            // update current time
            clock_gettime(CLOCK_MONOTONIC, &ts);
            now_poll = ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
        }
    }

    close(fd);
    return 0;
}

