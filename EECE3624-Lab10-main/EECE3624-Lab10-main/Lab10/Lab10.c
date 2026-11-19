/*****************************************************************************
 * File: FinalProject_AVR_Controller.c
 *
 * Purpose:
 * Read ReadyAVR joystick inputs and send commands
 * over Bluetooth to the Zumo robot.
 *
 * Commands:
 * F = Forward
 * B = Back
 * L = Left
 * R = Right
 * S = Stop
 * C = Center press
 *
 * UART1 - PuTTY Debug
 * UART0 - Bluetooth
 *****************************************************************************/

#define F_CPU 7372800UL

#define END_STRING '\0'
#define CR         0x0D
#define LF         0x0A

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "UARTLibrary.h"
#include "UART0Library.h"

//UART Settings
#define BAUDRATE   9600
#define DATABITS   8
#define STOPBITS   1
#define PARITY     0

void send_string(const char *cp);
char ReadJoystick(void);
static void release_pd3(void);
static void debug_enable(void);
static void debug_disable(void);
static void debug_send_cmd(char c);

void send_string(const char *cp)
{	
	while(*cp != END_STRING)
    {	uart_tx(*cp++);
    }
}

static void release_pd3(void) //Return PD3 to ordinary pulled-up input so PB3 (UP) can be read
{
    DDRD  &= ~(1 << PD3);   // input
    PORTD |=  (1 << PD3);   // pull-up
}

static void debug_enable(void) //PD3 being used in UART1
{
    UCSR1A |= (1 << TXC1);
    UCSR1B |= (1 << TXEN1);
}

static void debug_disable(void) //PD3 used in UART0 again
{
    while(!(UCSR1A & (1 << TXC1)));
    UCSR1A |= (1 << TXC1);
    UCSR1B &= ~(1 << TXEN1);
    release_pd3();
}

static void debug_send_cmd(char c) //send character to PuTTY
{
    debug_enable();
    uart_tx(c);
    uart_tx(CR);
    uart_tx(LF);
    debug_disable();
}

char ReadJoystick(void) //active low
{
    if(!(PINB & (1 << PB4))) //center
        return 'C';

    if(!(PINB & (1 << PB1))) //down
        return 'B';

    if(!(PINB & (1 << PB2))) //left
        return 'L';

    if(!(PINB & (1 << PB0))) //right
        return 'R';

    if(!(PINB & (1 << PB3))) //up
        return 'F';

    return 'S'; // stop
}

int main(void)
{
    char currentCmd;
    char previousCmd = 'X';

    // UART1 -> PuTTY
    uart_init(
        BAUDRATE,
        DATABITS,
        STOPBITS,
        PARITY
    );

    // UART0 -> Bluetooth
    uart0_init(
        BAUDRATE,
        DATABITS,
        STOPBITS,
        PARITY
    );

    // Startup banner
    debug_enable();
    send_string("\r\nReadyAVR Bluetooth Controller\r\n");
    debug_disable();

    // configure joystick pins after UART init so nothing overrides them
    DDRB &= ~(
        (1 << PB0) |
        (1 << PB1) |
        (1 << PB2) |
        (1 << PB3) |
        (1 << PB4)
    );

    PORTB |= (
        (1 << PB0) |
        (1 << PB1) |
        (1 << PB2) |
        (1 << PB3) |
        (1 << PB4)
    );

    release_pd3();

    while(1)
    {	currentCmd = ReadJoystick();

        if(currentCmd != previousCmd)
        {	uart0_tx(currentCmd);

            if(previousCmd == 'F')
                debug_send_cmd('F');

            // UP is held
            if(currentCmd != 'F')
                debug_send_cmd(currentCmd);

            previousCmd = currentCmd;
        }

        _delay_ms(20);
    }

    return 0;
}