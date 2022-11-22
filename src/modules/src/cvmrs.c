#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "log.h"
#include "cpx_internal_router.h"
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
const uint32_t baudrate_esp32 = 115200;

typedef struct
{
    uint8_t cmd;
    uint64_t timestamp; // usec timestamp from STM32
    float x; // m
    float y; // m
    float z; // m
    float qx;
    float qy;
    float qz;
    float qw;
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


    logVarId_t x_id = logGetVarId("stateEstimate", "x");
    logVarId_t y_id = logGetVarId("stateEstimate", "y");
    logVarId_t z_id = logGetVarId("stateEstimate", "z");
    logVarId_t qx_id = logGetVarId("stateEstimate", "qx");
    logVarId_t qy_id = logGetVarId("stateEstimate", "qy");
    logVarId_t qz_id = logGetVarId("stateEstimate", "qz");
    logVarId_t qw_id = logGetVarId("stateEstimate", "qw");

    cpxInitRoute(CPX_T_STM32, CPX_T_GAP8, CPX_F_APP, &cpx_packet.route);
    // cpx_packet.dataLength = sizeof(StatePacket_t);

    // StatePacket_t* state_packet = (StatePacket_t*)&cpx_packet.data;
    state_packet.cmd = 1;

    // Delay is only needed for CPX
    // vTaskDelay(60000);

    DEBUG_PRINT("Starting CVMRS task\n");

    lastWakeTime = xTaskGetTickCount();
    uart1Init(baudrate_esp32);
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(TASK_FREQ));

        // Sending current state information to GAP8
        state_packet.timestamp = usecTimestamp();
        state_packet.x = logGetFloat(x_id);
        state_packet.y = logGetFloat(y_id);
        state_packet.z = logGetFloat(z_id);
        state_packet.qx = logGetFloat(qx_id);
        state_packet.qy = logGetFloat(qy_id);
        state_packet.qz = logGetFloat(qz_id);
        state_packet.qw = logGetFloat(qw_id);
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