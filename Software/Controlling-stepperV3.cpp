#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/eeprom.h>

// Stepper motor pins
#define STEP_PIN1 PD0
#define DIR_PIN1  PD1
#define STEP_PIN2 PD2
#define DIR_PIN2  PD3
#define STEP_PIN3 PD4
#define DIR_PIN3  PD5

// Button pins
#define START_BUTTON_PIN PB0
#define STOP_BUTTON_PIN  PB1
#define EMERGENCY_STOP_BUTTON_PIN PB2
#define UP_BUTTON_PIN   PB3
#define DOWN_BUTTON_PIN PB4
#define LEFT_BUTTON_PIN PB5
#define RIGHT_BUTTON_PIN PB6
#define CALIBRATION_BUTTON_PIN PB7

// LCD pins
#define RS PC0
#define EN PC1
#define D4 PC2
#define D5 PC3
#define D6 PC4
#define D7 PC5

// Steps per millimeter for each axis
const float stepsPerMmHorizontal = 100.0;
const float stepsPerMmVertical = 100.0;
const float stepsPerMmGripper = 100.0;

// Variables for current position
int16_t currentHorizontalPosition = 0;
int16_t currentVerticalPosition = 0;
int16_t currentGripperPosition = 0;

// Function to set a bit in a port
#define SET_BIT(port, pin)   ((port) |= (1 << (pin)))
// Function to clear a bit in a port
#define CLEAR_BIT(port, pin) ((port) &= ~(1 << (pin)))

// Function to initialize pins
void init_pins() {
    // Set stepper motor pins as output
    DDRD |= (1 << STEP_PIN1) | (1 << DIR_PIN1) | (1 << STEP_PIN2) | (1 << DIR_PIN2) | (1 << STEP_PIN3) | (1 << DIR_PIN3);
    // Set button pins as input
    DDRB &= ~((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN));
    // Enable internal pull-up resistors for button pins
    PORTB |= (1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN);
    // Set LCD pins as output
    DDRC |= (1 << RS) | (1 << EN) | (1 << D4) | (1 << D5) | (1 << D6) | (1 << D7);
}

// Function to send command to LCD
void lcd_command(uint8_t cmd) {
    // Send higher nibble of command
    PORTC = (PORTC & 0x0F) | (cmd & 0xF0);
    CLEAR_BIT(PORTC, RS); // RS = 0 for command
    SET_BIT(PORTC, EN); // EN = 1 to enable LCD
    _delay_us(1);
    CLEAR_BIT(PORTC, EN); // EN = 0 to disable LCD
    _delay_us(200);
    // Send lower nibble of command
    PORTC = (PORTC & 0x0F) | ((cmd << 4) & 0xF0);
    SET_BIT(PORTC, EN); // EN = 1 to enable LCD
    _delay_us(1);
    CLEAR_BIT(PORTC, EN); // EN = 0 to disable LCD
    _delay_ms(2);
}

// Function to initialize LCD
void lcd_init() {
    _delay_ms(50);
    lcd_command(0x02); // Return home
    lcd_command(0x28); // 4-bit mode, 2 lines, 5x8 font
    lcd_command(0x0C); // Display on, cursor off
    lcd_command(0x06); // Entry mode set: increment cursor, no shift
    lcd_command(0x01); // Clear display
    _delay_ms(2);
}

// Function to send data to LCD
void lcd_data(uint8_t data) {
    // Send higher nibble of data
    PORTC = (PORTC & 0x0F) | (data & 0xF0);
    SET_BIT(PORTC, RS); // RS = 1 for data
    SET_BIT(PORTC, EN); // EN = 1 to enable LCD
    _delay_us(1);
    CLEAR_BIT(PORTC, EN); // EN = 0 to disable LCD
    _delay_us(200);
    // Send lower nibble of data
    PORTC = (PORTC & 0x0F) | ((data << 4) & 0xF0);
    SET_BIT(PORTC, EN); // EN = 1 to enable LCD
    _delay_us(1);
    CLEAR_BIT(PORTC, EN); // EN = 0 to disable LCD
    _delay_ms(2);
}

// Function to print a string on LCD
void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// Function to move a stepper motor
void move_stepper(volatile uint8_t *port, uint8_t stepPin, uint8_t dirPin, int16_t *position, float distance_mm, float stepsPerMm) {
    int steps = distance_mm * stepsPerMm;
    if (steps < 0) {
        CLEAR_BIT(*port, dirPin); // Set direction pin based on distance
        steps = -steps;
    } else {
        SET_BIT(*port, dirPin);
    }
    for (int i = 0; i < steps; ++i) {
        SET_BIT(*port, stepPin); // Step pulse
        _delay_us(100);
        CLEAR_BIT(*port, stepPin);
        _delay_us(100);
    }
    *position += (distance_mm * stepsPerMm); // Update current position
}

// Functions to move each axis
void moveHorizontal(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN1, DIR_PIN1, &currentHorizontalPosition, distance_mm, stepsPerMmHorizontal);
}

void moveVertical(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN2, DIR_PIN2, &currentVerticalPosition, distance_mm, stepsPerMmVertical);
}

void moveGripper(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN3, DIR_PIN3, &currentGripperPosition, distance_mm, stepsPerMmGripper);
}

// Function to perform a sequence of movements
void performSequence() {
    moveHorizontal(200);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-200);
    moveVertical(-50);
    moveGripper(-10);

    moveHorizontal(400);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-400);
    moveVertical(-50);
    moveGripper(-10);

    moveHorizontal(600);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-600);
    moveVertical(-50);
    moveGripper(-10);
}

// Function to check if a button is pressed
bool isButtonPressed(uint8_t pin) {
    if (!(PINB & (1 << pin))) {
        _delay_ms(50);
        if (!(PINB & (1 << pin))) {
            return true;
        }
    }
    return false;
}

// Function to restore initial position from EEPROM
void restorePosition() {
    int16_t initialHorizontalPosition = eeprom_read_word(&eepromInitialHorizontalPosition);
    int16_t initialVerticalPosition = eeprom_read_word(&eepromInitialVerticalPosition);
    int16_t initialGripperPosition = eeprom_read_word(&eepromInitialGripperPosition);

    moveHorizontal(initialHorizontalPosition / stepsPerMmHorizontal - currentHorizontalPosition / stepsPerMmHorizontal);
    moveVertical(initialVerticalPosition / stepsPerMmVertical - currentVerticalPosition / stepsPerMmVertical);
    moveGripper(initialGripperPosition / stepsPerMmGripper - currentGripperPosition / stepsPerMmGripper);

    currentHorizontalPosition = initialHorizontalPosition;
    currentVerticalPosition = initialVerticalPosition;
    currentGripperPosition = initialGripperPosition;
}

int main(void) {
    init_pins();
    lcd_init();
    lcd_print("System Ready");

    restorePosition();

    while (1) {
        if (isButtonPressed(START_BUTTON_PIN)) {
            // Start button pressed Set running flag and display message on LCD
            running = true;
            emergencyStopped = false;
            calibrationMode = false;
            lcd_command(0x01); // Clear display
            lcd_print("Running...");
        }

        if (isButtonPressed(STOP_BUTTON_PIN)) {
            // Stop button pressed Clear running flag and display message on LCD
            running = false;
            lcd_command(0x01); // Clear display
            lcd_print("Stopped");
        }

        if (isButtonPressed(EMERGENCY_STOP_BUTTON_PIN)) {
            // Emergency stop button pressed Clear running flag, set emergency stop flag, and display message on LCD
            running = false;
            emergencyStopped = true;
            lcd_command(0x01); // Clear display
            lcd_print("Emergency Stop");
        }

        if (isButtonPressed(CALIBRATION_BUTTON_PIN)) {
            // Calibration button pressed
            calibrationMode = !calibrationMode;
            if (calibrationMode) {
                lcd_command(0x01); // Clear display
                lcd_print("Calibration Mode");
            } else {
                lcd_command(0x01); // Clear display
                lcd_print("System Ready");
            }
        }

        if (calibrationMode) {
            if (isButtonPressed(UP_BUTTON_PIN)) {
                moveVertical(10);
            }
            if (isButtonPressed(DOWN_BUTTON_PIN)) {
                moveVertical(-10);
            }
            if (isButtonPressed(LEFT_BUTTON_PIN)) {
                moveHorizontal(-10);
            }
            if (isButtonPressed(RIGHT_BUTTON_PIN)) {
                moveHorizontal(10);
            }
        }

        if (running && !emergencyStopped) {
            // Perform sequence of movements
            performSequence();
        }
    }
    return 0;
}
