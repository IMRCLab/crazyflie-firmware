#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "log.h"
#include "aideck.h"
#include "usec_time.h"
#include "debug.h"

#define TASK_FREQ 1

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
    cpx_packet.dataLength = sizeof(StatePacket_t);
    StatePacket_t* state_packet = (StatePacket_t*)&cpx_packet.data;
    state_packet->cmd = 0;

    vTaskDelay(10000);

    lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(TASK_FREQ));

        // Sending current state information to GAP8
        state_packet->timestamp = usecTimestamp();
        state_packet->x = logGetInt(x_id);
        state_packet->y = logGetInt(y_id);
        state_packet->z = logGetInt(z_id);
        state_packet->quat = logGetInt(quat_id);

        cpxSendPacket(&cpx_packet, /*timeout*/ 10 /* ms */);
        // DEBUG_PRINT("cpxSendPacket\n");
    }
}