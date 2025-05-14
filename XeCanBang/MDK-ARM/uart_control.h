#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdint.h>

#define STOP        0
#define FORWARD     1
#define BACKWARD    2
#define TURN_LEFT   3
#define TURN_RIGHT  4

extern uint8_t desiredDirection;

void ProcessUARTCommand(uint8_t *rxBuffer);

#endif
