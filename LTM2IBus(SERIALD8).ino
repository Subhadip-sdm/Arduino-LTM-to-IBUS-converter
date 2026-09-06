//This code is made by subhadip-sdm which supports LTM to IBUS conversion
//LTM INPUT: Arduino D8
//iBUS OUTPUT: Arduino D11
//
//AltSoftSerial on ATmega328P:
//RX = D8
//TX = D9 (unused)

#include <Arduino.h>
#include <string.h>
#include <AltSoftSerial.h>
#include <iBUSTelemetry.h>

// =========================
// LTM SERIAL
// =========================

AltSoftSerial ltmSerial;


// =========================
// LTM DATA
// =========================

float ltm_voltage = 0.0;

uint8_t ltm_armed = 0;
uint8_t ltm_mode = 0;

int16_t ltm_pitch = 0;
int16_t ltm_roll = 0;
int16_t ltm_heading = 0;

uint8_t ltm_sats = 0;
uint8_t ltm_fix = 0;

int32_t ltm_alt = 0;   // centimeters
int32_t ltm_lat = 0;   // degrees * 10^7
int32_t ltm_lon = 0;   // degrees * 10^7


// =========================
// LTM PARSER
// =========================

enum LtmState {
    LTM_IDLE,
    LTM_HEADER_START,
    LTM_HEADER_T,
    LTM_HEADER_MSGID,
    LTM_CHECKSUM
};

LtmState ltm_state = LTM_IDLE;

uint8_t ltm_msgid = 0;
uint8_t ltm_payload_length = 0;
uint8_t ltm_payload_counter = 0;
uint8_t ltm_buffer[64];


// =========================
// iBUS
// =========================

#define UPDATE_INTERVAL 100

iBUSTelemetry telemetry(11);

uint32_t prevMillis = 0;


// =========================
// SETUP
// =========================

void setup()
{
    // LTM input:
    // Original D0 hardware Serial was:
    // Serial.begin(19200);
    //
    // Now LTM comes into D8 using AltSoftSerial.

    ltmSerial.begin(19200);

    telemetry.begin();

    telemetry.addSensor(IBUS_MEAS_TYPE_EXTV);          // 1
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_STATUS);    // 2
    telemetry.addSensor(IBUS_MEAS_TYPE_BAT_CURR);      // 3
    telemetry.addSensor(IBUS_MEAS_TYPE_CMP_HEAD);      // 4
    telemetry.addSensor(IBUS_MEAS_TYPE_COG);           // 5
    telemetry.addSensor(IBUS_MEAS_TYPE_CLIMB_RATE);    // 6
    telemetry.addSensor(IBUS_MEAS_TYPE_YAW);           // 7
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_DIST);      // 8
    telemetry.addSensor(IBUS_MEAS_TYPE_ARMED);         // 9
    telemetry.addSensor(IBUS_MEAS_TYPE_GROUND_SPEED);  // 10
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LAT);       // 11
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_LON);       // 12
    telemetry.addSensor(IBUS_MEAS_TYPE_GPS_ALT);       // 13
    telemetry.addSensor(IBUS_MEAS_TYPE_ALT);           // 14
    telemetry.addSensor(IBUS_MEAS_TYPE_FLIGHT_MODE);   // 15
}


// =========================
// LOOP
// =========================

void loop()
{
    while (ltmSerial.available())
    {
        uint8_t c = (uint8_t)ltmSerial.read();
        parseLtmchar(c);
    }

    updateValues();

    telemetry.run();
}


// =========================
// LTM PARSER
// =========================

void parseLtmchar(uint8_t c)
{
    switch (ltm_state)
    {
        case LTM_IDLE:

            if (c == '$')
                ltm_state = LTM_HEADER_START;

            break;


        case LTM_HEADER_START:

            if (c == 'T')
                ltm_state = LTM_HEADER_T;
            else
                ltm_state = LTM_IDLE;

            break;


        case LTM_HEADER_T:

            ltm_msgid = c;
            ltm_payload_counter = 0;

            if (ltm_msgid == 'G')
            {
                ltm_payload_length = 14;
                ltm_state = LTM_HEADER_MSGID;
            }
            else if (ltm_msgid == 'A')
            {
                ltm_payload_length = 6;
                ltm_state = LTM_HEADER_MSGID;
            }
            else if (ltm_msgid == 'S')
            {
                ltm_payload_length = 7;
                ltm_state = LTM_HEADER_MSGID;
            }
            else
            {
                ltm_state = LTM_IDLE;
            }

            break;


        case LTM_HEADER_MSGID:

            ltm_buffer[ltm_payload_counter++] = c;

            if (ltm_payload_counter >= ltm_payload_length)
                ltm_state = LTM_CHECKSUM;

            break;


        case LTM_CHECKSUM:
        {
            // =================================================
            // LTM CHECKSUM VALIDATION
            // =================================================
            //
            // LTM checksum is XOR of all payload bytes.
            //
            // If checksum is wrong, DON'T decode the packet.
            // This prevents corrupted serial data from creating
            // random GPS / altitude / heading / mode values.
            // =================================================

            uint8_t checksum = 0;

            for (uint8_t i = 0; i < ltm_payload_length; i++)
            {
                checksum ^= ltm_buffer[i];
            }

            if (checksum == c)
            {
                decodeLtmPacket(ltm_msgid, ltm_buffer);
            }

            ltm_state = LTM_IDLE;

            break;
        }
    }
}


// =========================
// DECODE LTM
// =========================

void decodeLtmPacket(uint8_t msgid, uint8_t *buf)
{
    // -------------------------
    // G FRAME
    // -------------------------

    if (msgid == 'G')
    {
        memcpy(&ltm_lat, &buf[0], 4);
        memcpy(&ltm_lon, &buf[4], 4);

        // buf[8] = ground speed

        // buf[9..12] = altitude in cm
        memcpy(&ltm_alt, &buf[9], 4);

        // buf[13]:
        // upper 6 bits = satellites
        // lower 2 bits = fix type

        ltm_sats = buf[13] >> 2;
        ltm_fix  = buf[13] & 0x03;
    }


    // -------------------------
    // A FRAME
    // -------------------------

    else if (msgid == 'A')
    {
        int16_t p;
        int16_t r;
        int16_t h;

        memcpy(&p, &buf[0], 2);
        memcpy(&r, &buf[2], 2);
        memcpy(&h, &buf[4], 2);

        ltm_pitch   = p;
        ltm_roll    = r;
        ltm_heading = h;
    }


    // -------------------------
    // S FRAME
    // -------------------------

    else if (msgid == 'S')
    {
        uint16_t vbat;

        memcpy(&vbat, &buf[0], 2);

        // Keep the original working voltage scaling.
        ltm_voltage = (float)vbat / 1000.0f;

        uint8_t statusByte = buf[6];

        // Bit 0 = armed
        ltm_armed = statusByte & 0x01;

        // Bits 2..7 = LTM flight mode
        ltm_mode = statusByte >> 2;
    }
}


// =========================
// LTM → iBUS MODE
// =========================

uint8_t getUniversalMappedMode(uint8_t mode)
{
    switch (mode)
    {
        // STAB
        case 0:     // MANUAL
        case 2:     // ANGLE
        case 3:     // HORIZON
        case 5:     // STABILIZED1
        case 6:     // STABILIZED2
        case 7:     // STABILIZED3
        case 11:    // HEADHOLD
            return 0;


        // ACRO
        case 1:     // RATE
        case 4:     // ACRO
            return 1;


        // ALT HOLD
        case 8:
            return 2;


        // AUTO
        case 10:    // WAYPOINTS
        case 16:    // FLYBYWIRE1
        case 17:    // FLYBYWIRE2
        case 18:    // CRUISE
        case 20:    // LAUNCH
        case 21:    // AUTOTUNE
            return 3;


        // LOITER
        case 9:     // GPSHOLD
        case 14:    // FOLLOWWME
            return 5;


        // CIRCLE
        case 12:
            return 7;


        // RTL
        case 13:
            return 6;


        // LAND
        case 15:
            return 9;


        default:
            return 0;
    }
}


// =========================
// UPDATE iBUS
// =========================

void updateValues()
{
    uint32_t currMillis = millis();

    if (currMillis - prevMillis >= UPDATE_INTERVAL)
    {
        prevMillis = currMillis;


        // 1 - Voltage
        telemetry.setSensorValueFP(
            1,
            ltm_voltage
        );


        // 3 - Current
        telemetry.setSensorValue(
            3,
            0
        );


        // 4 - Heading
        telemetry.setSensorValue(
            4,
            ltm_heading
        );


        // 5 - COG
        telemetry.setSensorValue(
            5,
            0
        );


        // 6 - Climb rate
        telemetry.setSensorValue(
            6,
            0
        );


        // 7 - Roll
        telemetry.setSensorValue(
            7,
            ltm_roll
        );


        // 8 - GPS distance
        telemetry.setSensorValue(
            8,
            0
        );


        // 9 - Armed
        telemetry.setSensorValue(
            9,
            ltm_armed
        );


        // 10 - Ground speed
        telemetry.setSensorValue(
            10,
            0
        );


        // 15 - Flight mode
        telemetry.setSensorValue(
            15,
            getUniversalMappedMode(ltm_mode)
        );


        // -------------------------
        // GPS
        // -------------------------

        if (ltm_fix > 0)
        {
            telemetry.setSensorValue(
                2,
                telemetry.gpsStateValues(
                    ltm_fix,
                    ltm_sats
                )
            );


            // Latitude: degrees * 10^7
            telemetry.setSensorValue(
                11,
                ltm_lat
            );


            // Longitude: degrees * 10^7
            telemetry.setSensorValue(
                12,
                ltm_lon
            );


            // LTM altitude is already centimeters.
            // iBUS 0x82 / 0x83 use centimeters.
            telemetry.setSensorValue(
                13,
                ltm_alt
            );


            telemetry.setSensorValue(
                14,
                ltm_alt
            );
        }
        else
        {
            telemetry.setSensorValue(
                2,
                telemetry.gpsStateValues(0, 0)
            );

            telemetry.setSensorValue(11, 0);
            telemetry.setSensorValue(12, 0);
            telemetry.setSensorValue(13, 0);
            telemetry.setSensorValue(14, 0);
        }
    }
}
