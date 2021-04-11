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


//
// Serial driver for Jeti EX Bus receiver
//

uint32_t jetiTimeStampRequest = 0;
uint8_t jetiExBusRequestState = EXBUS_STATE_ZERO;
uint8_t jetiExBusRequestFrame[EXBUS_MAX_REQUEST_FRAME_SIZE];
serialPort_t *jetiExBusPort;


static uint8_t jetiExBusFramePosition;
static uint8_t jetiExBusFrameLength;

static uint8_t jetiExBusFrameState = EXBUS_STATE_ZERO;

// Use max values for ram areas
static uint8_t jetiExBusChannelFrame[EXBUS_MAX_CHANNEL_FRAME_SIZE];

static uint16_t jetiExBusChannelData[JETIEXBUS_CHANNEL_COUNT];


// Jeti Ex Bus CRC calculations for a frame
uint16_t calcCRC16(uint8_t *pt, uint8_t msgLen)
{
    uint16_t crc16_data = 0;
    uint8_t data=0;

    for (uint8_t mlen = 0; mlen < msgLen; mlen++){
        data = pt[mlen] ^ ((uint8_t)(crc16_data) & (uint8_t)(0xFF));
        data ^= data << 4;
        crc16_data = ((((uint16_t)data << 8) | ((crc16_data & 0xFF00) >> 8))
                      ^ (uint8_t)(data >> 4)
                      ^ ((uint16_t)data << 3));
    }
    return(crc16_data);
}


void jetiExBusDecodeChannelFrame(uint8_t *exBusFrame)
{
    uint16_t value;
    uint8_t frameAddr;

    // Decode header
    switch (((uint16_t)exBusFrame[EXBUS_HEADER_SYNC] << 8) | ((uint16_t)exBusFrame[EXBUS_HEADER_REQ])){

    case EXBUS_CHANNELDATA_DATA_REQUEST:                   // not yet specified
    case EXBUS_CHANNELDATA:
        for (uint8_t i = 0; i < JETIEXBUS_CHANNEL_COUNT; i++) {
            frameAddr = EXBUS_HEADER_LEN + i * 2;
            value = ((uint16_t)exBusFrame[frameAddr + 1]) << 8;
            value += (uint16_t)exBusFrame[frameAddr];
            // Convert to internal format
            jetiExBusChannelData[i] = value >> 3;
        }
        break;
    }
}


void jetiExBusFrameReset(void)
{
    jetiExBusFramePosition = 0;
    jetiExBusFrameLength = EXBUS_MAX_CHANNEL_FRAME_SIZE;
}


/*
  supported:
  0x3E 0x01 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data with telemetry request (2nd byte 0x01)
  0x3E 0x03 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data forbids answering (2nd byte 0x03)
  0x3D 0x01 0x08 Packet_ID 0x3A 0x00 CRC16                   // Telemetry Request EX telemetry (5th byte 0x3A)

  other messages - not supported:
  0x3D 0x01 0x09 Packet_ID 0x3B 0x01 0xF0 CRC16              // Jetibox request (5th byte 0x3B)
  ...
*/

// Receive ISR callback
static void jetiExBusDataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    timeUs_t now;
    static timeUs_t jetiExBusTimeLast = 0;
    static timeDelta_t jetiExBusTimeInterval;

    static uint8_t *jetiExBusFrame;

    // Check if we shall reset frame position due to time
    now = micros();

    jetiExBusTimeInterval = cmpTimeUs(now, jetiExBusTimeLast);
    jetiExBusTimeLast = now;

    if (jetiExBusTimeInterval > JETIEXBUS_MIN_FRAME_GAP) {
        jetiExBusFrameReset();
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        jetiExBusRequestState = EXBUS_STATE_ZERO;
    }

    // Check if we shall start a frame?
    if (jetiExBusFramePosition == 0) {
        switch (c){
        case EXBUS_START_CHANNEL_FRAME:
            jetiExBusFrameState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = jetiExBusChannelFrame;
            break;

        case EXBUS_START_REQUEST_FRAME:
            jetiExBusRequestState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = jetiExBusRequestFrame;
            break;

        default:
            return;
        }
    }

    // Store in frame copy
    jetiExBusFrame[jetiExBusFramePosition] = (uint8_t)c;
    jetiExBusFramePosition++;

    // Check the header for the message length
    if (jetiExBusFramePosition == EXBUS_HEADER_LEN) {

        if ((jetiExBusFrameState == EXBUS_STATE_IN_PROGRESS) && (jetiExBusFrame[EXBUS_HEADER_MSG_LEN] <= EXBUS_MAX_CHANNEL_FRAME_SIZE)) {
            jetiExBusFrameLength = jetiExBusFrame[EXBUS_HEADER_MSG_LEN];
            return;
        }

        if ((jetiExBusRequestState == EXBUS_STATE_IN_PROGRESS) && (jetiExBusFrame[EXBUS_HEADER_MSG_LEN] <= EXBUS_MAX_REQUEST_FRAME_SIZE)) {
            jetiExBusFrameLength = jetiExBusFrame[EXBUS_HEADER_MSG_LEN];
            return;
        }

        jetiExBusFrameReset();                  // not a valid frame
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        jetiExBusRequestState = EXBUS_STATE_ZERO;
        return;
    }

    // Done?
    if (jetiExBusFrameLength == jetiExBusFramePosition) {
        if (jetiExBusFrameState == EXBUS_STATE_IN_PROGRESS)
            jetiExBusFrameState = EXBUS_STATE_RECEIVED;
        if (jetiExBusRequestState == EXBUS_STATE_IN_PROGRESS) {
            jetiExBusRequestState = EXBUS_STATE_RECEIVED;
            jetiTimeStampRequest = micros();
        }

        jetiExBusFrameReset();
    }
}


// Check if it is time to read a frame from the data...
static uint8_t jetiExBusFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (jetiExBusFrameState != EXBUS_STATE_RECEIVED)
        return RX_FRAME_PENDING;

    if (calcCRC16(jetiExBusChannelFrame, jetiExBusChannelFrame[EXBUS_HEADER_MSG_LEN]) == 0) {
        jetiExBusDecodeChannelFrame(jetiExBusChannelFrame);
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        return RX_FRAME_COMPLETE;
    } else {
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        return RX_FRAME_PENDING;
    }
}


static uint16_t jetiExBusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount)
        return 0;

    return (jetiExBusChannelData[chan]);
}


bool jetiExBusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = JETIEXBUS_CHANNEL_COUNT;
    rxRuntimeConfig->rxRefreshRate = 5500;

    rxRuntimeConfig->rcReadRawFn = jetiExBusReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = jetiExBusFrameStatus;

    jetiExBusFrameReset();

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);

    if (!portConfig) {
        return false;
    }

    jetiExBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        jetiExBusDataReceive,
        NULL,
        JETIEXBUS_BAUDRATE,
        MODE_RXTX,
        JETIEXBUS_OPTIONS | (tristateWithDefaultOffIsActive(rxConfig->halfDuplex) ? SERIAL_BIDIR : 0)
        );

    serialSetMode(jetiExBusPort, MODE_RX);
    return jetiExBusPort != NULL;
}
#endif // USE_SERIALRX_JETIEXBUS
