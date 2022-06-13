#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "log.h"
#include "aideck.h"
#include "uart1.h"
#include "usec_time.h"
#include "debug.h"
#include "param.h"

#define TASK_FREQ 100

// GAP8 register settings

static uint8_t aeg;
static uint8_t aGain;
static uint8_t dGain;
static uint16_t exposure;
static uint8_t trigger = 0;


typedef struct
{
    uint8_t cmd;
    uint64_t timestamp; // usec timestamp from STM32
    int16_t x;          // compressed [mm]
    int16_t y;          // compressed [mm]
    int16_t z;          // compressed [mm]
    uint32_t quat;      // compressed, see quatcompress.h
} __attribute__((packed)) StatePacket_t;
// static StatePacket_t cf_state;
static CPXPacket_t cpx_packet;
static StatePacket_t state_packet;

typedef struct
{
    uint8_t cmd;
    uint8_t aeg;
    uint8_t aGain;
    uint8_t dGain;
    uint16_t exposure;
} __attribute__((packed)) RegisterPacket_t;

void appMain()
{
    uint32_t lastWakeTime;

    //Wait for the system to be fully started to start stabilization loop
    systemWaitStart();


    logVarId_t x_id = logGetVarId("stateEstimateZ", "x");
    logVarId_t y_id = logGetVarId("stateEstimateZ", "y");
    logVarId_t z_id = logGetVarId("stateEstimateZ", "z");
    logVarId_t quat_id = logGetVarId("stateEstimateZ", "quat");

    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpx_packet.route);
    // cpx_packet.dataLength = sizeof(StatePacket_t);

    // StatePacket_t* state_packet = (StatePacket_t*)&cpx_packet.data;
    state_packet.cmd = 0;

    // Delay is only needed for CPX
    // vTaskDelay(60000);

    DEBUG_PRINT("Starting CVMRS task\n");

    lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(TASK_FREQ));

        // Sending current state information to GAP8
        state_packet.timestamp = usecTimestamp();
        state_packet.x = logGetInt(x_id);
        state_packet.y = logGetInt(y_id);
        state_packet.z = logGetInt(z_id);
        state_packet.quat = logGetInt(quat_id);

        // cpxSendPacket(&cpx_packet, /*timeout*/ 10 /* ms */);

        uint8_t magic = 0xBC;
        uint8_t length = sizeof(StatePacket_t);
        uart1SendData(1, &magic);
        uart1SendData(1, &length);
        uart1SendData(length, (uint8_t*)&state_packet);

        // compute crc
        uint8_t crc = 0;
        for (const uint8_t* p = (uint8_t*)&state_packet; p < (uint8_t*)&state_packet + length; p++) {
            crc ^= *p;
        }
        uart1SendData(1, &crc);

        if (uart1DidOverrun()) {
            DEBUG_PRINT("UART overrun!\n");
        }

        // DEBUG_PRINT("cpxSendPacket\n");

        if (trigger == 1) {
            cpx_packet.dataLength = sizeof(RegisterPacket_t);
            RegisterPacket_t* reg_packet = (RegisterPacket_t*)&cpx_packet.data;
            reg_packet->cmd = 0;
            reg_packet->aeg = aeg;
            reg_packet->aGain = aGain;
            reg_packet->dGain = dGain;
            reg_packet->exposure = exposure;

            cpxSendPacketBlocking(&cpx_packet);
            trigger = 0;
        }
    }
}


PARAM_GROUP_START(cvmrs)
/**
 * @brief Automatic Exposure Gain (0: disabled; 1: enabled)
 */
PARAM_ADD(PARAM_UINT8, aeg, &aeg)
/**
 * @brief Analog gain (1, 2, 4, or 8) [If AEG is disabled]
 */
PARAM_ADD(PARAM_UINT8, aGain, &aGain)
/**
 * @brief Digital gain in 2.6 format (2-bit integer, 6-bit floating)  [If AEG is disabled]
 */
PARAM_ADD(PARAM_UINT8, dGain, &dGain)
/**
 * @brief Exposure 2 to (frame_length_lines - 2) [If AEG is disabled]
 */
PARAM_ADD(PARAM_UINT16, exposure, &exposure)
/**
 * @brief Triggers an update of all settings/registers on the GAP8
 */
PARAM_ADD(PARAM_UINT8, trigger, &trigger)
PARAM_GROUP_STOP(cvmrs)