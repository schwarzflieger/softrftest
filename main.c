/*
 * MAVLink UART example program (MAVLink v1)
 *
 * Build:
 *   gcc -I./mavlink_c -o mav_uart mav_uart.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <time.h>

#include "mavlink/common/mavlink.h"

#define SYS_ID   1
#define COMP_ID  1

#define DEFAULT_COM_PORT  "/dev/ttyUSB0"
#define DEFAULT_COM_SPEED 57600



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
        printf("Unsupported baudrate: %d\n", baudrate);
        return -1;
    }

    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open");
        return -1;
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

void send_gps_raw(int fd)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // the test coords shall be set closer than ~25-30km to your actual position
    // otherwise flarm decodere will compute wrong coordinates
    int32_t lat = (int32_t)(45.67 * 1e7);
    int32_t lon = (int32_t)(12.34 * 1e7);

    int32_t alt = 100 * 1000;   // 100m → millimeters

    mavlink_msg_gps_raw_int_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        get_system_time_usec(),
        3,                                  // fix type (3D fix)
        lat,
        lon,
        alt,
        100,                                // eph (cm)
        100,                                // epv
        100,                                  // vel
        100,                                  // cog
        5                                   // satellites
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
        case MAVLINK_MSG_ID_ADSB_VEHICLE: {
            mavlink_adsb_vehicle_t adsb;
            mavlink_msg_adsb_vehicle_decode(msg, &adsb);
            printf("ICAO=%06X CALL=%s lat=%.7f lon=%.7f alt=%d head=%d hvel=%d vvel=%d at=%d et=%d squawk=%d\n",
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

//-------------------------------------------------------
// MAIN
//-------------------------------------------------------
int main(int argc, char *argv[])
{
    const char* port = DEFAULT_COM_PORT;
    int baud = DEFAULT_COM_SPEED;

    if (argc >= 2) port = argv[1];
    if (argc >= 3) baud = atoi(argv[2]);

    printf("Using UART: %s  baud: %d\n", port, baud);

    int fd = uart_init(port, baud);
    if (fd < 0) return 1;

    mavlink_message_t msg;
    mavlink_status_t status;

    uint64_t last_1hz = 0;

    while (1) {
        uint64_t now_ms = (uint64_t)(time(NULL) * 1000ULL);

        //---- FIXED 1Hz MESSAGES ----
        if (now_ms - last_1hz >= 1000) {
            send_heartbeat(fd);
            send_system_time(fd);
            send_gps_raw(fd);
            last_1hz = now_ms;
        }

        //---- STREAMS ----
        send_stream_extended_status(fd, now_ms);
        send_stream_extra3(fd, now_ms);

        //---- RECEIVE ----
        uint8_t c;
        if (read(fd, &c, 1) > 0) {
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
                parse_message(&msg);
        }

        usleep(1000);
    }

    close(fd);
    return 0;
}

