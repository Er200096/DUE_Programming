#include "usart.h"

USART::USART() {
    // Constructor if needed
}

void USART::init() {
    // Set Baud Rate
    UBRR0H = BAUD_PRESCALER >> 8;
    UBRR0L = BAUD_PRESCALER;
    
    // Set Frame Format
    UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;
    
    // Enable Receiver and Transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    Serial1.begin(9600); // Initialize Serial1 with desired baud rate
    // Enable Global Interrupts
    sei();
}

void USART::transmit(uint8_t data) {
    Serial1.write(data);
}

uint8_t USART::receive() {
    while (!Serial1.available()); // Wait until data is received
    return Serial1.read();
}
