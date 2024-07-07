#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

// I2C definitions
#define F_CPU 16000000UL // Clock Speed
#define F_SCL 100000UL // I2C clock speed
#define TWBR_VALUE ((F_CPU / F_SCL - 16) / 2)

#define I2C_WRITE 0
#define I2C_READ 1

// Define I2C LCD address
#define LCD_ADDR 0x27

// Define stepper motor pins
#define STEP_PIN1 PD0
#define DIR_PIN1  PD1
#define STEP_PIN2 PD2
#define DIR_PIN2  PD3
#define STEP_PIN3 PD4
#define DIR_PIN3  PD5

// Define button pins
#define START_BUTTON_PIN PB0
#define STOP_BUTTON_PIN  PB1
#define EMERGENCY_STOP_BUTTON_PIN PB2
#define UP_BUTTON_PIN   PB3
#define DOWN_BUTTON_PIN PB4
#define LEFT_BUTTON_PIN PB5
#define RIGHT_BUTTON_PIN PB6
#define CALIBRATION_BUTTON_PIN PB7

// Define steps per mm for each axis
const float stepsPerMmHorizontal = 100.0;
const float stepsPerMmVertical = 100.0;
const float stepsPerMmGripper = 100.0;

// Define volatile variables for system state
volatile bool running = false;
volatile bool emergencyStopped = false;
volatile bool calibrationMode = false;

// EEPROM addresses for initial positions (predefined)
uint16_t EEMEM eepromInitialHorizontalPosition = 0;
uint16_t EEMEM eepromInitialVerticalPosition = 0;
uint16_t EEMEM eepromInitialGripperPosition = 0;

// Current positions
int16_t currentHorizontalPosition = 0;
int16_t currentVerticalPosition = 0;
int16_t currentGripperPosition = 0;

// Helper macros to set/clear bits in a port
#define SET_BIT(port, pin)   ((port) |= (1 << (pin)))
#define CLEAR_BIT(port, pin) ((port) &= ~(1 << (pin)))

// Function to initialize pins
void init_pins() {
    // Set stepper motor pins as output
    DDRD |= (1 << STEP_PIN1) | (1 << DIR_PIN1) | (1 << STEP_PIN2) | (1 << DIR_PIN2) | (1 << STEP_PIN3) | (1 << DIR_PIN3);
    
    // Set button pins as input and enable internal pull-up resistors
    DDRB &= ~((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN));
    PORTB |= (1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN);
}

// I2C initialization function
void i2c_init() {
    TWBR = (uint8_t)TWBR_VALUE;
}

// I2C start condition
void i2c_start() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

// I2C stop condition
void i2c_stop() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO));
}

// I2C write byte
void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

// I2C write command
void lcd_command(uint8_t cmd) {
    i2c_start();
    i2c_write(LCD_ADDR << 1 | I2C_WRITE);
    i2c_write(0x00);
    i2c_write(cmd);
    i2c_stop();
    _delay_ms(2);
}

// I2C write data
void lcd_data(uint8_t data) {
    i2c_start();
    i2c_write(LCD_ADDR << 1 | I2C_WRITE);
    i2c_write(0x40);
    i2c_write(data);
    i2c_stop();
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
        CLEAR_BIT(*port, dirPin);
        steps = -steps;
    } else {
        SET_BIT(*port, dirPin);
    }

    for (int i = 0; i < steps; ++i) {
        SET_BIT(*port, stepPin);
        _delay_us(100);
        CLEAR_BIT(*port, stepPin);
        _delay_us(100);
    }

    *position += (distance_mm * stepsPerMm);
}

// Function to move the horizontal axis
void moveHorizontal(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN1, DIR_PIN1, &currentHorizontalPosition, distance_mm, stepsPerMmHorizontal);
}

// Function to move the vertical axis
void moveVertical(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN2, DIR_PIN2, &currentVerticalPosition, distance_mm, stepsPerMmVertical);
}

// Function to move the gripper
void moveGripper(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN3, DIR_PIN3, &currentGripperPosition, distance_mm, stepsPerMmGripper);
}

// Function to perform the sequence of movements
void performSequence() {
    // First point
    moveHorizontal(200);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-200);
    moveVertical(-50);
    moveGripper(-10);

    // Second point
    moveHorizontal(400);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-400);
    moveVertical(-50);
    moveGripper(-10);

    // Third point
    moveHorizontal(600);
    moveVertical(-50);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-600);
    moveVertical(-50);
    moveGripper(-10);
}

// Function to debounce buttons
bool isButtonPressed(uint8_t pin) {
    if (!(PINB & (1 << pin))) {
        _delay_ms(50); // Debounce delay
        if (!(PINB & (1 << pin))) {
            return true;
        }
    }
    return false;
}

// Function to restore position from EEPROM
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
    i2c_init();
    lcd_init();
    lcd_print("System Ready");

    restorePosition();

    while (1) {
        if (isButtonPressed(START_BUTTON_PIN)) {
            running = true;
            emergencyStopped = false;
            calibrationMode = false;
            lcd_command(0x01); // Clear display
            lcd_print("Running...");
        }

        if (isButtonPressed(STOP_BUTTON_PIN)) {
            running = false;
            lcd_command(0x01); // Clear display
            lcd_print("Stopped");
        }

        if (isButtonPressed(EMERGENCY_STOP_BUTTON_PIN)) {
            running = false;
            emergencyStopped = true;
            lcd_command(0x01); // Clear display
            lcd_print("Emergency Stop");
        }

        if (isButtonPressed(CALIBRATION_BUTTON_PIN)) {
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
            performSequence();
        }
    }
    return 0;
}
