#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>

class USART {
public:
    USART();
    void init();
    void transmit(uint8_t data);
    uint8_t receive();
};

#endif // USART_H
