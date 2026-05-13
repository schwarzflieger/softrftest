#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MSP_MAX_PAYLOAD 512

#define MSP2_INAV_GPS_RAW_INT 0x2220
#define MSP2_SENSOR_ADSB      0x1F08

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint16_t function;
    uint16_t payload_size;
    uint8_t payload[MSP_MAX_PAYLOAD];
    uint8_t checksum;
} msp_message_t;

typedef struct __attribute__((packed)) {
  uint32_t ICAO_address; /*<  ICAO address*/
  int32_t lat; /*< [degE7] Latitude*/
  int32_t lon; /*< [degE7] Longitude*/
  int32_t altitude; /*< [mm] Altitude(ASL)*/
  uint16_t heading; /*< [cdeg] Course over ground*/
  uint16_t hor_velocity; /*< [cm/s] The horizontal velocity*/
  int16_t ver_velocity; /*< [cm/s] The vertical velocity. Positive is up*/
  uint16_t flags; /*<  Bitmap to indicate various statuses including valid data fields*/
  uint16_t squawk; /*<  Squawk code*/
  uint8_t altitude_type; /*<  ADSB altitude type.*/
  char callsign[9]; /*<  The callsign, 8+null*/
  uint8_t emitter_type; /*<  ADSB emitter type.*/
  uint8_t tslc; /*< [s] Time since last communication in seconds*/
} msp_adsb_vehicle_t;

typedef struct __attribute__((packed)) {
  uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
  int32_t lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
  int32_t lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
  int32_t alt; /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
  uint16_t eph; /*<  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
  uint16_t epv; /*<  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
  uint16_t vel; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
  uint16_t cog; /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
  uint8_t fix_type; /*<  GPS fix type.*/
  uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
  int32_t alt_ellipsoid; /*< [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.*/
  uint32_t h_acc; /*< [mm] Position uncertainty.*/
  uint32_t v_acc; /*< [mm] Altitude uncertainty.*/
  uint32_t vel_acc; /*< [mm] Speed uncertainty.*/
  uint32_t hdg_acc; /*< [degE5] Heading / track uncertainty*/
  uint16_t yaw; /*< [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.*/
} msp_gps_raw_int_t;


bool msp_parse_char(uint8_t c, msp_message_t *msg);
uint16_t msp_msg_to_send_buffer(uint8_t *buf, const msp_message_t *msg);
bool msp_msg_gps_raw_int_decode(const msp_message_t* msg, msp_gps_raw_int_t* gps);
void msp_msg_gps_raw_int_request_encode(msp_message_t* msg);
void msp_msg_gps_raw_int_response_encode(msp_message_t* msg, const msp_gps_raw_int_t* gpsInfo);
void msp_msg_adsb_vehicle_encode(msp_message_t* msg, const msp_adsb_vehicle_t* vehicle);
bool msp_msg_adsb_vehicle_decode(const msp_message_t* msg, msp_adsb_vehicle_t* vehicle);

