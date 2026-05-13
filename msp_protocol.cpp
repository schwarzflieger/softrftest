#include "msp_protocol.h"


struct MspParser
{
    enum State
    {
        MSP_IDLE,
        MSP_HEADER_X,
        MSP_TYPE,
        MSP_FLAGS,
        MSP_FUNCTION_LSB,
        MSP_FUNCTION_MSB,
        MSP_SIZE_LSB,
        MSP_SIZE_MSB,
        MSP_PAYLOAD,
        MSP_CHECKSUM
    };

    State state = MSP_IDLE;

    uint8_t crc = 0;
    uint16_t payload_received = 0;
};

static MspParser parser;

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;

    for (int i = 0; i < 8; i++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0xD5;
        else
            crc <<= 1;
    }

    return crc;
}

bool msp_parse_char(uint8_t c, msp_message_t *msg)
{
    switch (parser.state)
    {
    case MspParser::MSP_IDLE:
        if (c == '$')
            parser.state = MspParser::MSP_HEADER_X;
        break;

    case MspParser::MSP_HEADER_X:
        if (c == 'X')
            parser.state = MspParser::MSP_TYPE;
        else
            parser.state = MspParser::MSP_IDLE;
        break;

    case MspParser::MSP_TYPE:
        if (c == '<' || c == '>' || c == '!')
        {
            msg->type = c;

            // CRC starts from FLAGS field
            parser.crc = 0;

            parser.state = MspParser::MSP_FLAGS;
        }
        else
        {
            parser.state = MspParser::MSP_IDLE;
        }
        break;

    case MspParser::MSP_FLAGS:
        msg->flags = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.state = MspParser::MSP_FUNCTION_LSB;
        break;

    case MspParser::MSP_FUNCTION_LSB:
        msg->function = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.state = MspParser::MSP_FUNCTION_MSB;
        break;

    case MspParser::MSP_FUNCTION_MSB:
        msg->function |= ((uint16_t)c << 8);
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.state = MspParser::MSP_SIZE_LSB;
        break;

    case MspParser::MSP_SIZE_LSB:
        msg->payload_size = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);
        parser.state = MspParser::MSP_SIZE_MSB;
        break;

    case MspParser::MSP_SIZE_MSB:
        msg->payload_size |= ((uint16_t)c << 8);
        parser.crc = crc8_dvb_s2(parser.crc, c);

        if (msg->payload_size > MSP_MAX_PAYLOAD)
        {
            parser.state = MspParser::MSP_IDLE;
            break;
        }

        parser.payload_received = 0;

        if (msg->payload_size == 0)
            parser.state = MspParser::MSP_CHECKSUM;
        else
            parser.state = MspParser::MSP_PAYLOAD;

        break;

    case MspParser::MSP_PAYLOAD:
        msg->payload[parser.payload_received++] = c;
        parser.crc = crc8_dvb_s2(parser.crc, c);

        if (parser.payload_received >= msg->payload_size)
            parser.state = MspParser::MSP_CHECKSUM;

        break;

    case MspParser::MSP_CHECKSUM:
        msg->checksum = c;

        parser.state = MspParser::MSP_IDLE;

        if (parser.crc == c)
            return true;

        break;

    default:
        parser.state = MspParser::MSP_IDLE;
        break;
    }

    return false;
}

uint16_t msp_msg_to_send_buffer(uint8_t *buf, const msp_message_t *msg)
{
    uint16_t index = 0;
    uint8_t crc = 0;

    // Header
    buf[index++] = '$';
    buf[index++] = 'X';
    buf[index++] = msg->type;

    // Flags
    buf[index++] = msg->flags;
    crc = crc8_dvb_s2(crc, msg->flags);

    // Function (little endian)
    buf[index++] = (uint8_t)(msg->function & 0xFF);
    crc = crc8_dvb_s2(crc, buf[index - 1]);

    buf[index++] = (uint8_t)((msg->function >> 8) & 0xFF);
    crc = crc8_dvb_s2(crc, buf[index - 1]);

    // Payload size (little endian)
    buf[index++] = (uint8_t)(msg->payload_size & 0xFF);
    crc = crc8_dvb_s2(crc, buf[index - 1]);

    buf[index++] = (uint8_t)((msg->payload_size >> 8) & 0xFF);
    crc = crc8_dvb_s2(crc, buf[index - 1]);

    // Payload
    for (uint16_t i = 0; i < msg->payload_size; i++)
    {
        buf[index++] = msg->payload[i];
        crc = crc8_dvb_s2(crc, msg->payload[i]);
    }

    // Checksum
    buf[index++] = crc;

    return index;
}

bool msp_msg_gps_raw_int_decode(const msp_message_t* msg, msp_gps_raw_int_t* gps)
{
    if (msg->payload_size < sizeof(msp_gps_raw_int_t)) {
        return false;
    }

    memcpy(gps, msg->payload, sizeof(msp_gps_raw_int_t));
    return true;
}

void msp_msg_gps_raw_int_request_encode(msp_message_t* msg)
{
    // type '<' = request, '>' = response, '!' = error
    msg->type = '<';
    msg->flags = 0;
    msg->function = MSP2_INAV_GPS_RAW_INT;
    msg->payload_size = 0u; // request message has no payload
}

void msp_msg_gps_raw_int_response_encode(msp_message_t* msg, const msp_gps_raw_int_t* gpsInfo)
{
    // type '<' = request, '>' = response, '!' = error
    msg->type = '>';
    msg->flags = 0;
    msg->function = MSP2_INAV_GPS_RAW_INT;
    msg->payload_size = sizeof(msp_gps_raw_int_t);
    memcpy(msg->payload, gpsInfo, sizeof(msp_gps_raw_int_t));
}

void msp_msg_adsb_vehicle_encode(msp_message_t* msg, const msp_adsb_vehicle_t* vehicle)
{
    msg->type = '<';
    msg->flags = 1; // no response is needed
    msg->function = MSP2_SENSOR_ADSB;
    msg->payload_size = sizeof(msp_adsb_vehicle_t);
    memcpy(msg->payload, vehicle, sizeof(msp_adsb_vehicle_t));
}

bool msp_msg_adsb_vehicle_decode(const msp_message_t* msg, msp_adsb_vehicle_t* vehicle)
{
    if (msg->payload_size < sizeof(msp_adsb_vehicle_t)) {
        return false;
    }

    memcpy(vehicle, msg->payload, sizeof(msp_adsb_vehicle_t));
    return true;
}

