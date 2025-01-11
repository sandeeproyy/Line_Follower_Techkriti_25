#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Definitions
// IR Sensor Array
const int IR_PINS[] = {2, 3, 4, 5, 6, 7, 8}; // 7 channel array
const int OBSTACLE_SENSOR = 9;

// Motor Driver Pins
const int PWMA = 10;
const int AIN1 = 11;
const int AIN2 = 12;
const int PWMB = 13;
const int BIN1 = A0;
const int BIN2 = A1;

// Encoder Pins
const int RIGHT_ENCODER = A2;
const int LEFT_ENCODER = A3;

// Constants
const float WHEEL_DIAMETER = 65.0; // mm
const int PULSES_PER_REVOLUTION = 20;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Global Variables
int nodeCount = 0;
int currentSpeed = 0;
float targetAngle = 0;
bool circleMode = false;

// Robot States
enum RobotState {
    FOLLOWING_LINE,
    READING_NODE,
    SPEED_CONTROL,
    ANGLE_DETECTION,
    CIRCLE_FOLLOWING,
    FINISHED
} currentState = FOLLOWING_LINE;

void setup() {
    // Initialize OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Don't proceed if OLED fails
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Initialize IR Sensors
    for(int i = 0; i < 7; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
    pinMode(OBSTACLE_SENSOR, INPUT);
    
    // Initialize Motor Pins
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    
    // Initialize Encoder Pins
    pinMode(RIGHT_ENCODER, INPUT_PULLUP);
    pinMode(LEFT_ENCODER, INPUT_PULLUP);
    
    displayMessage("Ready!");
    delay(1000);
}

void loop() {
    switch(currentState) {
        case FOLLOWING_LINE:
            followLine();
            checkForNode();
            checkForObstacle();
            break;
            
        case READING_NODE:
            readNode();
            break;
            
        case SPEED_CONTROL:
            controlSpeed();
            break;
            
        case ANGLE_DETECTION:
            detectAngle();
            break;
            
        case CIRCLE_FOLLOWING:
            followCircle();
            break;
            
        case FINISHED:
            displayMessage("FINISH");
            stop();
            break;
    }
}

void followLine() {
    int sensorValues[7];
    int position = 0;
    int sum = 0;
    int count = 0;
    
    // Read all sensors
    for(int i = 0; i < 7; i++) {
        sensorValues[i] = digitalRead(IR_PINS[i]);
        if(sensorValues[i] == 1) {
            position += i * 1000;
            count++;
        }
        sum += sensorValues[i];
    }
    
    if(count > 0) {
        position /= count;
    }
    
    // PID constants
    float Kp = 0.5;
    float Ki = 0.0;
    float Kd = 0.1;
    
    static int lastError = 0;
    static float integral = 0;
    
    int error = 3000 - position;
    integral = integral + error;
    int derivative = error - lastError;
    
    int motorSpeed = Kp * error + Ki * integral + Kd * derivative;
    
    int leftSpeed = currentSpeed - motorSpeed;
    int rightSpeed = currentSpeed + motorSpeed;
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    lastError = error;
}

void readNode() {
    int nodeValue = 0;
    bool nodeSensors[4];
    
    // Read the 2x2 matrix
    nodeSensors[0] = digitalRead(IR_PINS[1]); // A
    nodeSensors[1] = digitalRead(IR_PINS[5]); // B
    nodeSensors[2] = digitalRead(IR_PINS[2]); // C
    nodeSensors[3] = digitalRead(IR_PINS[4]); // D
    
    // Convert to binary number
    for(int i = 0; i < 4; i++) {
        nodeValue = (nodeValue << 1) | nodeSensors[i];
    }
    
    displayNodeValue(nodeValue);
    nodeCount++;
    
    // Process node based on count
    switch(nodeCount) {
        case 1: // Speed control node
            currentSpeed = (nodeValue * 51); // Convert to speed (x/5 m/s to PWM)
            currentState = SPEED_CONTROL;
            break;
        case 2: // Zero node
            currentSpeed = 128; // Reset to default speed
            currentState = FOLLOWING_LINE;
            break;
        case 3: // Angle node
            targetAngle = nodeValue * 10.0;
            currentState = ANGLE_DETECTION;
            break;
    }
}

void checkForObstacle() {
    if(digitalRead(OBSTACLE_SENSOR) == LOW) {
        // Obstacle detected
        stop();
        delay(500);
        turn180();
    }
}

void displayNodeValue(int value) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("Node Value: ");
    display.println(value);
    display.print("Binary: ");
    
    // Display binary representation
    for(int i = 3; i >= 0; i--) {
        display.print((value >> i) & 1);
    }
    display.display();
}

void displayMessage(const char* message) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(message);
    display.display();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left Motor
    if(leftSpeed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        leftSpeed = -leftSpeed;
    }
    
    // Right Motor
    if(rightSpeed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        rightSpeed = -rightSpeed;
    }
    
    analogWrite(PWMA, leftSpeed);
    analogWrite(PWMB, rightSpeed);
}

void stop() {
    setMotorSpeeds(0, 0);
}

void checkForNode() {
    // Check if all middle sensors detect black
    if(digitalRead(IR_PINS[2]) && digitalRead(IR_PINS[3]) && digitalRead(IR_PINS[4])) {
        delay(100); // Ensure robot is centered on node
        currentState = READING_NODE;
    }
}

void followCircle() {
    // Implement circle following logic using encoders
    // This is a simplified version - you'll need to tune this based on your robot
    int innerSpeed = 128;
    int outerSpeed = 255;
    
    if(targetAngle > 0) {
        setMotorSpeeds(innerSpeed, outerSpeed); // Turn clockwise
    } else {
        setMotorSpeeds(outerSpeed, innerSpeed); // Turn counter-clockwise
    }
    
    // Check if we've reached the target angle using encoders
    // This is where you'd implement the actual angle detection
    // For now, we'll use a simple delay
    delay(abs(targetAngle) * 20);
    
    currentState = FOLLOWING_LINE;
}