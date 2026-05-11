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

#include "mavlink/common/mavlink.h"

#define SYS_ID   1
#define COMP_ID  1

#define DEFAULT_COM_PORT  "/dev/ttyUSB0"
#define DEFAULT_COM_SPEED 115200
//#define DEFAULT_COM_SPEED 57600
#define DEFAULT_MAVLINK_VERSION 2
#define DEFAULT_LAT 45.67 // fake lat
#define DEFAULT_LON 12.34 // fake lon

int32_t g_latitude = 0;
int32_t g_longitude = 0;

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
void send_heartbeat(int fd)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        SYS_ID, COMP_ID, &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );

    write(fd, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

uint64_t get_system_time_usec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    uint64_t time_unix_usec =
        (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;

    return time_unix_usec;
}

void send_system_time(int fd)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_system_time_pack(
        SYS_ID, COMP_ID, &msg,
        get_system_time_usec(),
        0
    );

    write(fd, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

static uint16_t cdeg = 7500;

void send_gps_raw(int fd)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    int32_t alt = 100 * 1000;   // 100m → millimeters

    mavlink_msg_gps_raw_int_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        get_system_time_usec(),
        3,                                  // fix type (3D fix)
        g_latitude,
        g_longitude,
        alt,
        100,                                // eph (cm)
        100,                                // epv
        100,                                // vel
        cdeg,                               // cog
        5,                                  // satellites
        0,                                  // alt_ellipsoid
        0,                                  // h_acc
        0,                                  // v_acc
        0,                                  // vel_acc
        0,                                  // hdg_acc
        0                                   // yaw
    );

    write(fd, buf, mavlink_msg_to_send_buffer(buf, &msg));
}

//-------------------------------------------------------
// STREAM CONTROL VARIABLES
//-------------------------------------------------------
int      send_extended_status     = 0;
uint16_t rate_extended_status     = 1;
uint64_t last_extended_status     = 0;

int      send_extra3              = 0;
uint16_t rate_extra3              = 1;
uint64_t last_extra3              = 0;

//-------------------------------------------------------
// STREAM FUNCTIONS
//-------------------------------------------------------
void send_stream_extended_status(int fd, uint64_t now_ms)
{
    if (!send_extended_status) return;
    if (now_ms - last_extended_status < (1000 / rate_extended_status)) return;

    mavlink_message_t m;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // enum MAV_SYS_STATUS_SENSOR
    uint32_t sensors_present = 0;
    uint32_t sensors_enabled = 0;
    uint32_t sensors_health  = 0;

    uint16_t load              = 0;
    uint16_t voltage_battery   = 12000;
    int16_t  current_battery   = -1;
    int8_t   battery_remaining = -1;

    uint16_t drop_rate_comm = 0;
    uint16_t errors_comm    = 0;

    uint16_t err1 = 0;
    uint16_t err2 = 0;
    uint16_t err3 = 0;
    uint16_t err4 = 0;

    mavlink_msg_sys_status_pack(
        SYS_ID,
        COMP_ID,
        &m,
        sensors_present,
        sensors_enabled,
        sensors_health,
        load,
        voltage_battery,
        current_battery,
        battery_remaining,
        drop_rate_comm,
        errors_comm,
        err1,
        err2,
        err3,
        err4
    );

    write(fd, buf, mavlink_msg_to_send_buffer(buf, &m));
    last_extended_status = now_ms;
}

void send_stream_extra3(int fd, uint64_t now_ms)
{
    if (!send_extra3) return;
    if (now_ms - last_extra3 < (1000 / rate_extra3)) return;

    mavlink_message_t m;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_vfr_hud_t hud;
    hud.airspeed    = 0.0f;
    hud.groundspeed = 0.0f;
    hud.heading     = 0;
    hud.throttle    = 0;
    hud.alt         = 0.0f;
    hud.climb       = 0.0f;

    mavlink_msg_vfr_hud_encode(
        SYS_ID,
        COMP_ID,
        &m,
        &hud
    );

    write(fd, buf, mavlink_msg_to_send_buffer(buf, &m));
    last_extra3 = now_ms;
}

//-------------------------------------------------------
// PARSER — only ADS-B and REQUEST_DATA_STREAM
//-------------------------------------------------------
void parse_message(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            break;
        case MAVLINK_MSG_ID_ADSB_VEHICLE: {
            mavlink_adsb_vehicle_t adsb;
            mavlink_msg_adsb_vehicle_decode(msg, &adsb);
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

        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            mavlink_request_data_stream_t req;
            mavlink_msg_request_data_stream_decode(msg, &req);

            printf("[REQUEST_DATA_STREAM] stream=%d rate=%d start=%d\n",
                   req.req_stream_id,
                   req.req_message_rate,
                   req.start_stop);

            if (req.req_stream_id == MAV_DATA_STREAM_EXTENDED_STATUS) {
                send_extended_status = req.start_stop ? 1 : 0;
                rate_extended_status = req.req_message_rate ? req.req_message_rate : 1;
            }
            else if (req.req_stream_id == MAV_DATA_STREAM_EXTRA3) {
                send_extra3 = req.start_stop ? 1 : 0;
                rate_extra3 = req.req_message_rate ? req.req_message_rate : 1;
            }
            break;
        }

        default:
            printf("Message id: %d\n", msg->msgid);
            break;

    }
}

void show_usage(char* prog_name)
{
    fprintf(stderr, "\nUsage: %s -p PORT [-b BAUD] [-m MAVLINK Version] [--lat LAT --lon LON]\n", prog_name);
    fprintf(stderr, "Defaults: -b 115200 -m 2 --lat 45.67 --lon 12.34\n");
}

//-------------------------------------------------------
// MAIN
//-------------------------------------------------------
int main(int argc, char *argv[])
{
    const char* port = DEFAULT_COM_PORT;
    int baudrate = DEFAULT_COM_SPEED;
    int mavlink_version = DEFAULT_MAVLINK_VERSION;  // optional
    double lat = DEFAULT_LAT;
    double lon = DEFAULT_LON;

    int opt;
    static struct option long_options[] = {
        {"port",     required_argument, 0, 'p'},
        {"baudrate", required_argument, 0, 'b'},
        {"mavlink",  required_argument, 0, 'm'},
        {"lat",      required_argument, 0, 1},
        {"lon",      required_argument, 0, 2},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "p:b:m:", long_options, NULL)) != -1) {
        switch(opt) {
            case 'p': port = optarg; break;
            case 'b': baudrate = atoi(optarg); break;
            case 'm': mavlink_version = atoi(optarg); break;
            case 1: lat = atof(optarg); break;
            case 2: lon = atof(optarg); break;
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

    if (mavlink_version != 1 && mavlink_version != 2) {
        fprintf(stderr, "Error: Invalid MAVLink version: %d (must be 1 or 2)\n", mavlink_version);
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
    printf("MAVLink Version: %d\n", mavlink_version);
    printf("GPS: %.6f, %.6f\n", lat, lon);

    g_latitude  = (int32_t)(lat * 1e7);
    g_longitude = (int32_t)(lon * 1e7);

    if (mavlink_version == 1) {
        mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

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
            send_gps_raw(fd);
            send_heartbeat(fd);
            send_system_time(fd);
            next_tick += 1000;
        }

        //---- STREAMS ----
        send_stream_extended_status(fd, now_ms);
        send_stream_extra3(fd, now_ms);

#if 0
        //---- RECEIVE ----
        uint8_t c;
        while (read(fd, &c, 1) > 0) {
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                parse_message(&msg);
        }

        // sleep until next second boundary
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now_after = ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;

        uint64_t sleep_ms = (next_tick > now_after) ? (next_tick - now_after) : 0;
        usleep(sleep_ms * 1000);
#endif

        //---- RECEIVE UNTIL NEXT TICK ----
        uint64_t now_poll = now_ms;
        while (now_poll < next_tick) {
            uint8_t c;
            ssize_t r = read(fd, &c, 1);
            if (r > 0) {
                if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                    parse_message(&msg);
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

