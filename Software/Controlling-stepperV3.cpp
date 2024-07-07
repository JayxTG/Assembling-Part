#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/eeprom.h>

// Define the pins for the stepper motors
#define STEP_PIN1 PD0
#define DIR_PIN1  PD1
#define STEP_PIN2 PD2
#define DIR_PIN2  PD3
#define STEP_PIN3 PD4
#define DIR_PIN3  PD5

// Define the pins for the buttons
#define START_BUTTON_PIN PB0
#define STOP_BUTTON_PIN  PB1
#define EMERGENCY_STOP_BUTTON_PIN PB2
#define UP_BUTTON_PIN   PB3
#define DOWN_BUTTON_PIN PB4
#define LEFT_BUTTON_PIN PB5
#define RIGHT_BUTTON_PIN PB6
#define CALIBRATION_BUTTON_PIN PB7

// Define the I2C address for the LCD
#define LCD_I2C_ADDR 0x27

// Define the steps per millimeter for each stepper motor
const float stepsPerMmHorizontal = 100.0;
const float stepsPerMmVertical = 100.0;
const float stepsPerMmGripper = 100.0;

// Define variables for the current positions of the stepper motors
int16_t currentHorizontalPosition = 0;
int16_t currentVerticalPosition = 0;
int16_t currentGripperPosition = 0;

// Define helper macros for setting and clearing bits in a port
#define SET_BIT(port, pin)   ((port) |= (1 << (pin)))
#define CLEAR_BIT(port, pin) ((port) &= ~(1 << (pin)))

// Function to initialize the pins
void init_pins() {
    // Set the direction of stepper motor pins as output
    DDRD |= (1 << STEP_PIN1) | (1 << DIR_PIN1) | (1 << STEP_PIN2) | (1 << DIR_PIN2) | (1 << STEP_PIN3) | (1 << DIR_PIN3);
    // Set the direction of button pins as input
    DDRB &= ~((1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN));
    // Enable internal pull-up resistors for button pins
    PORTB |= (1 << START_BUTTON_PIN) | (1 << STOP_BUTTON_PIN) | (1 << EMERGENCY_STOP_BUTTON_PIN) | (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << CALIBRATION_BUTTON_PIN);
}

// Function to initialize I2C communication
void i2c_init() {
    // Initialize I2C communication
    TWSR = 0x00;
    TWBR = 0x48;
    TWCR = (1 << TWEN);
}

// Function to start I2C communication
void i2c_start() {
    // Start I2C communication
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

// Function to stop I2C communication
void i2c_stop() {
    // Stop I2C communication
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

// Function to write data over I2C
void i2c_write(uint8_t data) {
    // Write data over I2C
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

// Function to send command to LCD
void lcd_command(uint8_t cmd) {
    // Send command to LCD
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(0x00);
    i2c_write(cmd);
    i2c_stop();
    _delay_ms(2);
}

// Function to send data to LCD
void lcd_data(uint8_t data) {
    // Send data to LCD
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(0x40);
    i2c_write(data);
    i2c_stop();
    _delay_ms(2);
}

// Function to initialize the LCD
void lcd_init() {
    _delay_ms(50);

    // Initialize LCD
    lcd_command(0x38);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    _delay_ms(2);
}

// Function to print a string on the LCD
void lcd_print(const char *str) {
    // Print string on LCD
    while (*str) {
        lcd_data(*str++);
    }
}

// Function to move a stepper motor
void move_stepper(volatile uint8_t *port, uint8_t stepPin, uint8_t dirPin, int16_t *currentPosition, float distance_mm, float stepsPerMm) {
    // Constants for acceleration and maximum velocity
    const float acceleration = 5.0;
    const float max_velocity = 33.33;
    const float distance_per_step = 0.01;

    // Calculate the total number of steps required to move the desired distance
    int total_steps = distance_mm * stepsPerMm;
    int current_steps = 0;

    // Calculate the acceleration time and distance
    float acc_time = max_velocity / acceleration;
    float acc_distance = 0.5 * acceleration * acc_time * acc_time;
    int acc_steps = acc_distance / distance_per_step;

    // Set the direction of the motor based on the distance
    if (distance_mm < 0) {
        CLEAR_BIT(*port, dirPin);
        distance_mm = -distance_mm;
    } else {
        SET_BIT(*port, dirPin);
    }

    // Move the motor step by step
    while (current_steps < total_steps) {
        float current_velocity = 0.0;
        if (current_steps < acc_steps) {
            current_velocity = acceleration * (current_steps * distance_per_step) / max_velocity;
        } else if (current_steps >= total_steps - acc_steps) {
            current_velocity = acceleration * ((total_steps - current_steps) * distance_per_step) / max_velocity;
        } else {
            current_velocity = max_velocity;
        }

        // Calculate the delay between steps based on the current velocity
        float step_delay = 1.0 / (stepsPerMm * current_velocity) * 1e6;

        // Step the motor
        SET_BIT(*port, stepPin);
        _delay_us(step_delay / 2);
        CLEAR_BIT(*port, stepPin);
        _delay_us(step_delay / 2);

        current_steps++;
    }

    // Update the current position of the motor
    *currentPosition += distance_mm * (stepsPerMm / 100.0);
}

// Function to move the horizontal stepper motor
void moveHorizontal(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN1, DIR_PIN1, &currentHorizontalPosition, distance_mm, stepsPerMmHorizontal);
}

// Function to move the vertical stepper motor
void moveVertical(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN2, DIR_PIN2, &currentVerticalPosition, distance_mm, stepsPerMmVertical);
}

// Function to move the gripper stepper motor
void moveGripper(float distance_mm) {
    move_stepper(&PORTD, STEP_PIN3, DIR_PIN3, &currentGripperPosition, distance_mm, stepsPerMmGripper);
}

// Function to perform a predefined sequence of movements
void performSequence() {
    moveHorizontal(100);
    moveVertical(100);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-100);
    moveVertical(-100);
    moveGripper(-10);

    moveHorizontal(250);
    moveVertical(100);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-250);
    moveVertical(-100);
    moveGripper(-10);

    moveHorizontal(400);
    moveVertical(100);
    moveGripper(10);
    moveVertical(50);
    moveHorizontal(-400);
    moveVertical(-100);
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

// Function to restore the initial positions of the stepper motors
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
            lcd_command(0x01);
            lcd_print("Running...");
        }

        if (isButtonPressed(STOP_BUTTON_PIN)) {
            running = false;
            lcd_command(0x01);
            lcd_print("Stopped");
        }

        if (isButtonPressed(EMERGENCY_STOP_BUTTON_PIN)) {
            running = false;
            emergencyStopped = true;
            lcd_command(0x01);
            lcd_print("Emergency Stop");
        }

        if (isButtonPressed(CALIBRATION_BUTTON_PIN)) {
            calibrationMode = !calibrationMode;
            if (calibrationMode) {
                lcd_command(0x01);
                lcd_print("Calibration Mode");
            } else {
                lcd_command(0x01);
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
