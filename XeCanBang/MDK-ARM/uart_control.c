#include "uart_control.h"

uint8_t desiredDirection = STOP;

void ProcessUARTCommand(uint8_t *rxBuffer) {
    if (rxBuffer[0] == 'F') {
        desiredDirection = FORWARD;
    } else if (rxBuffer[0] == 'B') {
        desiredDirection = BACKWARD;
    } else if (rxBuffer[0] == 'L') {
        desiredDirection = TURN_LEFT;
    } else if (rxBuffer[0] == 'R') {
        desiredDirection = TURN_RIGHT;
    } else if (rxBuffer[0] == 'S') {
        desiredDirection = STOP;
    }
}
