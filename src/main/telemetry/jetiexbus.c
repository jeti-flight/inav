/*
 * This file is part of Cleanflight and iNavFlight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 * Thomas Miric - marv-t
 *
 * Jeti EX Bus Communication Protocol:
 * http://www.jetimodel.com/en/show-file/642/
 *
 * JETI Telemetry Communication Protocol:
 * http://www.jetimodel.com/en/show-file/26/
 *
 * Following restrictions:
 * Communication speed: 125000 bps
 * Number of channels: 16
 *
 * Connect as follows:
 * Jeti EX Bus -> Serial RX (connect directly)
 * Serial TX -> Resistor(2k4) ->Serial RX
 * In jeti pdf it is different, but if the resistor breaks, the receiver continues to operate.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_JETIEXBUS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"
#include "common/log.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/jetiexbus.h"

#ifdef USE_TELEMETRY_JETIEXBUS
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/acceleration.h"

#include "telemetry/telemetry.h"
#include "telemetry/jetiexbus.h"

#define EXTEL_DATA_MSG      (0x40)
#define EXTEL_UNMASK_TYPE   (0x3F)
#define EXTEL_SYNC_LEN      1
#define EXTEL_CRC_LEN       1
#define EXTEL_HEADER_LEN    6
#define EXTEL_MAX_LEN       29
#define EXTEL_OVERHEAD      (EXTEL_SYNC_LEN + EXTEL_HEADER_LEN + EXTEL_CRC_LEN)
#define EXTEL_MAX_PAYLOAD   (EXTEL_MAX_LEN - EXTEL_OVERHEAD)
#define EXBUS_MAX_REQUEST_BUFFER_SIZE   (EXBUS_OVERHEAD + EXTEL_MAX_LEN)

enum exTelHeader_e {
    EXTEL_HEADER_SYNC = 0,
    EXTEL_HEADER_TYPE_LEN,
    EXTEL_HEADER_USN_LB,
    EXTEL_HEADER_USN_HB,
    EXTEL_HEADER_LSN_LB,
    EXTEL_HEADER_LSN_HB,
    EXTEL_HEADER_RES,
    EXTEL_HEADER_ID,
    EXTEL_HEADER_DATA
};

enum exDataType_e {
    EX_TYPE_6b   = 0, // int6_t  Data type 6b (-31 ¸31)
    EX_TYPE_14b  = 1, // int14_t Data type 14b (-8191 ¸8191)
    EX_TYPE_22b  = 4, // int22_t Data type 22b (-2097151 ¸2097151)
    EX_TYPE_DT   = 5, // int22_t Special data type – time and date
    EX_TYPE_30b  = 8, // int30_t Data type 30b (-536870911 ¸536870911)
    EX_TYPE_GPS  = 9  // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree.
};

const uint8_t exDataTypeLen[]={
    [EX_TYPE_6b]  = 1,
    [EX_TYPE_14b] = 2,
    [EX_TYPE_22b] = 3,
    [EX_TYPE_DT]  = 3,
    [EX_TYPE_30b] = 4,
    [EX_TYPE_GPS] = 4
};

typedef struct exBusSensor_s{
    const char *label;
    const char *unit;
    int32_t value;
    const uint8_t exDataType;
    const uint8_t decimals;
} exBusSensor_t;

fpVector3_t ex_measuredAcc[3];

#define DECIMAL_MASK(decimals) (decimals << 5)

// start old cf definitions
// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "CF-Dev 1.12 S2")
/*
exBusSensor_t jetiExSensors[] = {
    { "CF-Dev 1.12 S1", "",     0,      0,             0 },                     // device descripton
    { "Voltage",        "V",    0,      EX_TYPE_14b,   DECIMAL_MASK(1) },
    { "Current",        "A",    0,      EX_TYPE_14b,   DECIMAL_MASK(2) },
    { "Altitude",       "m",    0,      EX_TYPE_14b,   DECIMAL_MASK(1) },
    { "Capacity",       "mAh",  0,      EX_TYPE_22b,   DECIMAL_MASK(0) },
    { "frames lost",    " ",    0,      EX_TYPE_22b,   DECIMAL_MASK(0) },       // for debug only
    { "time Diff",      "us",   0,      EX_TYPE_14b,   DECIMAL_MASK(0) }        // for debug only
};
*/

// after every 15 sensors increment the step by 2 (e.g. ...EX_VAL15, EX_VAL16 = 17) to skip the device description
/*
enum exSensors_e {
    EX_VOLTAGE = 1,
    EX_CURRENT,
    EX_ALTITUDE,
    EX_CAPACITY,
    EX_FRAMES_LOST,                                                             // for debug only
    EX_TIME_DIFF                                                                // for debug only
};
*/
// end old cf definitions

// start re-use definitions from BetaFlight
// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "BF D2")
exBusSensor_t jetiExSensors[] = {
    {"iNav D1",         "",         0,  0,              0              },     // device descripton
    {"Voltage",         "V",        0,  EX_TYPE_14b,    DECIMAL_MASK(2)},
    {"Current",         "A",        0,  EX_TYPE_14b,    DECIMAL_MASK(2)},
    {"Altitude",        "m",        0,  EX_TYPE_14b,    DECIMAL_MASK(1)},
    {"Capacity",        "mAh",      0,  EX_TYPE_22b,    DECIMAL_MASK(0)},
    {"Power",           "W",        0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"Roll angle",      "\xB0",     0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"Pitch angle",     "\xB0",     0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"Heading",         "\xB0",     0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"Vario",           "m/s",      0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"GPS Sats",        "",         0,  EX_TYPE_22b,    DECIMAL_MASK(0)},
    {"GPS Long",        "",         0,  EX_TYPE_GPS,    DECIMAL_MASK(0)},
    {"GPS Lat",         "",         0,  EX_TYPE_GPS,    DECIMAL_MASK(0)},
    {"GPS Speed",       "m/s",      0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"GPS H-Distance",  "m",        0,  EX_TYPE_22b,    DECIMAL_MASK(0)},
    {"GPS H-Direction", "\xB0",     0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"iNav D2",         "",         0,  0,              0              },     // device descripton
    {"GPS Heading",     "\xB0",     0,  EX_TYPE_22b,    DECIMAL_MASK(1)},
    {"GPS Altitude",    "m",        0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"G-Force X",       "",         0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"G-Force Y",       "",         0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"G-Force Z",       "",         0,  EX_TYPE_22b,    DECIMAL_MASK(2)},
    {"frames lost",    " ",         0,  EX_TYPE_22b,    DECIMAL_MASK(0)},       // for debug only
    {"time Diff",      "us",        0,  EX_TYPE_14b,    DECIMAL_MASK(0)}        // for debug only
};

// after every 15 sensors increment the step by 2 (e.g. ...EX_VAL15, EX_VAL16 = 17) to skip the device description
enum exSensors_e {
    EX_VOLTAGE = 1,
    EX_CURRENT,
    EX_ALTITUDE,
    EX_CAPACITY,
    EX_POWER,
    EX_ROLL_ANGLE,
    EX_PITCH_ANGLE,
    EX_HEADING,
    EX_VARIO,
    EX_GPS_SATS,
    EX_GPS_LONG,
    EX_GPS_LAT,
    EX_GPS_SPEED,
    EX_GPS_DISTANCE_TO_HOME,
    EX_GPS_DIRECTION_TO_HOME,
    EX_GPS_HEADING = 17,
    EX_GPS_ALTITUDE,
    EX_GFORCE_X,
    EX_GFORCE_Y,
    EX_GFORCE_Z,
    EX_FRAMES_LOST,                                                             // for debug only
    EX_TIME_DIFF                                                                // for debug only
};
// end re-use definitions from BetaFlight

#define JETI_EX_SENSOR_COUNT (ARRAYLEN(jetiExSensors))

static uint8_t jetiExBusTelemetryFrame[40];
static uint8_t jetiExBusTransceiveState = EXBUS_TRANS_RX;
static void sendJetiExBusTelemetry(uint8_t packetID);

uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen);


// Jeti Ex Telemetry CRC calculations for a frame
uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen)
{
    uint8_t crc=0;

    for (uint8_t mlen = 0; mlen < msgLen; mlen++) {
        crc  ^= pt[mlen];
        crc = crc ^ (crc << 1) ^ (crc << 2) ^ (0x0e090700 >> ((crc >> 3) & 0x18));
    }
    return(crc);
}

/*
  -----------------------------------------------
   Jeti Ex Bus Telemetry
  -----------------------------------------------
*/

void initJetiExBusTelemetry(void)
{
    //debug to msp
    LOG_D(SYSTEM,"Jeti - init telemetry\n");

    // Init Ex Bus Frame header
    jetiExBusTelemetryFrame[EXBUS_HEADER_SYNC] = 0x3B;       // Startbytes
    jetiExBusTelemetryFrame[EXBUS_HEADER_REQ] = 0x01;
    jetiExBusTelemetryFrame[EXBUS_HEADER_DATA_ID] = 0x3A;    // Ex Telemetry


    // Init Ex Telemetry header
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];
    jetiExTelemetryFrame[EXTEL_HEADER_SYNC] = 0x9F;              // Startbyte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_LB] = 0x1E;            // Serial Number 4 Byte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_HB] = 0xA4;
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_LB] = 0x00;            // increment by telemetry count (%16) > only 15 values per device possible
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_HB] = 0x00;
    jetiExTelemetryFrame[EXTEL_HEADER_RES] = 0x00;               // reserved, by default 0x00
}


void createExTelemetrieTextMessage(uint8_t *exMessage, uint8_t messageID, const exBusSensor_t *sensor)
{
    uint8_t labelLength = strlen(sensor->label);
    uint8_t unitLength = strlen(sensor->unit);

    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_OVERHEAD + labelLength + unitLength;
    exMessage[EXTEL_HEADER_LSN_LB] = messageID & 0xF0;                              // Device ID
    exMessage[EXTEL_HEADER_ID] = messageID & 0x0F;                                  // Sensor ID (%16)
    exMessage[EXTEL_HEADER_DATA] = (labelLength << 3) + unitLength;

    LOG_D(SYSTEM,"Jeti - message id telemetry %x\n", messageID);
    LOG_D(SYSTEM,"Jeti - label %s\n", sensor->label);

    memcpy(&exMessage[EXTEL_HEADER_DATA + 1], sensor->label, labelLength);
    memcpy(&exMessage[EXTEL_HEADER_DATA + 1 + labelLength], sensor->unit, unitLength);

    exMessage[exMessage[EXTEL_HEADER_TYPE_LEN] + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], exMessage[EXTEL_HEADER_TYPE_LEN]);
}


uint8_t createExTelemetrieValueMessage(uint8_t *exMessage, uint8_t itemStart)
{
    uint8_t item = itemStart;
    uint8_t iCount;
    uint8_t messageSize;
    uint32_t sensorValue;

    if ((item & 0x0F) == 0)
        item++;

    if (item >= JETI_EX_SENSOR_COUNT)
        item = 1;

    exMessage[EXTEL_HEADER_LSN_LB] = item & 0xF0;                                   // Device ID
    uint8_t *p = &exMessage[EXTEL_HEADER_ID];

    while (item <= (itemStart | 0x0F)) {
        *p++ = ((item & 0x0F) << 4) | jetiExSensors[item].exDataType;               // Sensor ID (%16) | EX Data Type

        sensorValue = jetiExSensors[item].value;
        iCount = exDataTypeLen[jetiExSensors[item].exDataType];
        while (iCount > 1) {
            *p++ = sensorValue;
            sensorValue = sensorValue >> 8;
            iCount--;
        }
        *p++ = (sensorValue & 0x9F) | jetiExSensors[item].decimals;

        item++;
        if (item >= JETI_EX_SENSOR_COUNT)
            break;
        if (EXTEL_MAX_PAYLOAD <= ((p-&exMessage[EXTEL_HEADER_ID]) + exDataTypeLen[jetiExSensors[item].exDataType]) + 1)
            break;
    }

    messageSize = (EXTEL_HEADER_LEN + (p-&exMessage[EXTEL_HEADER_ID]));
    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_DATA_MSG | messageSize;
    exMessage[messageSize + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], messageSize);

    return item;        // return the next item
}


void createExBusMessage(uint8_t *exBusMessage, uint8_t *exMessage, uint8_t packetID)
{
    uint16_t crc16;

    exBusMessage[EXBUS_HEADER_PACKET_ID] = packetID;
    exBusMessage[EXBUS_HEADER_SUBLEN] = (exMessage[EXTEL_HEADER_TYPE_LEN] & EXTEL_UNMASK_TYPE) + 2;    // +2: startbyte & CRC8
    exBusMessage[EXBUS_HEADER_MSG_LEN] = EXBUS_OVERHEAD + exBusMessage[EXBUS_HEADER_SUBLEN];

    crc16 = calcCRC16(exBusMessage, exBusMessage[EXBUS_HEADER_MSG_LEN] - EXBUS_CRC_LEN);
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 2] = crc16;
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 1] = crc16 >> 8;
}


void checkJetiExBusTelemetryState(void)
{
    return;
}


void handleJetiExBusTelemetry(void)
{
    static uint16_t framesLost = 0; // only for debug
    uint32_t timeDiff;
    // Check if we shall reset frame position due to time

    if (jetiExBusRequestState == EXBUS_STATE_RECEIVED) {

        // to prevent timing issues from request to answer - max. 4ms
        timeDiff = micros() - jetiTimeStampRequest;

        if (timeDiff > 3000) {   // include reserved time
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            framesLost++;
            return;
        }

        if ((jetiExBusRequestFrame[EXBUS_HEADER_DATA_ID] == EXBUS_EX_REQUEST) && (calcCRC16(jetiExBusRequestFrame, jetiExBusRequestFrame[EXBUS_HEADER_MSG_LEN]) == 0)) {
            jetiExSensors[EX_VOLTAGE].value = getBatteryVoltage();
            jetiExSensors[EX_CURRENT].value = getAmperage();
            jetiExSensors[EX_ALTITUDE].value = baro.BaroAlt / 100;
            jetiExSensors[EX_CAPACITY].value = getMAhDrawn();
            // add other values here
            accGetMeasuredAcceleration(ex_measuredAcc);
            jetiExSensors[EX_GFORCE_X].value = ex_measuredAcc->v[0] / 10;
            jetiExSensors[EX_GFORCE_Y].value = ex_measuredAcc->v[1] / 10;
            jetiExSensors[EX_GFORCE_Z].value = ex_measuredAcc->v[2] / GRAVITY_CMSS * 100 ;

            // debug at the end
            jetiExSensors[EX_FRAMES_LOST].value = framesLost;
            jetiExSensors[EX_TIME_DIFF].value = timeDiff;

            // switch to TX mode
            if (uartTotalRxBytesWaiting(jetiExBusPort) == 0) {
                serialSetMode(jetiExBusPort, MODE_TX);
                jetiExBusTransceiveState = EXBUS_TRANS_TX;
                sendJetiExBusTelemetry(jetiExBusRequestFrame[EXBUS_HEADER_PACKET_ID]);
                jetiExBusRequestState = EXBUS_STATE_PROCESSED;
            }
        } else {
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            return;
        }
    }

    // check the state if transmit is ready
    if (jetiExBusTransceiveState == EXBUS_TRANS_IS_TX_COMPLETED) {
        if (isSerialTransmitBufferEmpty(jetiExBusPort)) {
            serialSetMode(jetiExBusPort, MODE_RX);
            jetiExBusTransceiveState = EXBUS_TRANS_RX;
            jetiExBusRequestState = EXBUS_STATE_ZERO;
        }
    }
}


void sendJetiExBusTelemetry(uint8_t packetID)
{
    static uint8_t sensorDescriptionCounter = 0;
    static uint8_t sensorValueCounter = 1;
    static uint8_t requestLoop = 0;
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];

    if (requestLoop == 100){              //every nth request send the name of a value
        if (sensorDescriptionCounter == JETI_EX_SENSOR_COUNT )
            sensorDescriptionCounter = 0;

        createExTelemetrieTextMessage(jetiExTelemetryFrame, sensorDescriptionCounter, &jetiExSensors[sensorDescriptionCounter]);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);

        requestLoop = 0;
        sensorDescriptionCounter++;
    } else {
        sensorValueCounter = createExTelemetrieValueMessage(jetiExTelemetryFrame, sensorValueCounter);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);
    }

    for (uint8_t iCount = 0; iCount < jetiExBusTelemetryFrame[EXBUS_HEADER_MSG_LEN]; iCount++) {
        serialWrite(jetiExBusPort, jetiExBusTelemetryFrame[iCount]);
    }
    jetiExBusTransceiveState = EXBUS_TRANS_IS_TX_COMPLETED;
    requestLoop++;
}
#endif // USE_TELEMETRY_JETIEXBUS
#endif // USE_SERIALRX_JETIEXBUS
