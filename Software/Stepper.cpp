#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Define pins (Arduino pin 8 is PB0, pin 9 is PB1 on ATmega328P)
#define STEP_PIN PB0
#define DIR_PIN PB1

// Variables to track the motor state
volatile uint8_t moving_forward = 1;
volatile int16_t current_position = 0;
#define STEPS_PER_REVOLUTION 200 // Change this according to your stepper motor's specification

// Timer1 compare match value for 500 steps per second
#define TIMER_COMPARE_VALUE 31250 // 16MHz / (500Hz * 1024 prescaler) - 1

void setup(void) {
    // Set STEP_PIN and DIR_PIN as outputs
    DDRB |= (1 << STEP_PIN) | (1 << DIR_PIN);
    
    // Initialize stepper state (set direction pin high)
    PORTB |= (1 << DIR_PIN);

    // Set up Timer1 for precise timing
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    OCR1A = TIMER_COMPARE_VALUE;
    
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // 1024 prescaler
    
    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt

    sei(); // Enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
    // Toggle the step pin
    PORTB ^= (1 << STEP_PIN);
    
    // Only update position on rising edge
    if (PORTB & (1 << STEP_PIN)) {
        // Update the current position
        if (moving_forward) {
            current_position++;
            if (current_position >= STEPS_PER_REVOLUTION) {
                moving_forward = 0;
                PORTB &= ~(1 << DIR_PIN); // Set direction pin low
            }
        } else {
            current_position--;
            if (current_position <= 0) {
                moving_forward = 1;
                PORTB |= (1 << DIR_PIN); // Set direction pin high
            }
        }
    }
}

int main(void) {
    setup();

    while (1) {
        // Add a delay between full rotations
        if (current_position == 0 || current_position == STEPS_PER_REVOLUTION) {
            TIMSK1 &= ~(1 << OCIE1A); // Disable Timer1 interrupt
            _delay_ms(1000);
            TIMSK1 |= (1 << OCIE1A); // Re-enable Timer1 interrupt
        }
    }

    return 0;
}