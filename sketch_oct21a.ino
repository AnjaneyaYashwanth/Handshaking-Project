#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define pins for ultrasonic sensor
const int trigPin = 9;
const int echoPin = 10;

// Create a PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo channels for wheel and steering servos (PWM)
const int leftWheelServo = 0;   // Channel for the left wheel servo
const int rightWheelServo = 1;  // Channel for the right wheel servo
const int steeringServo = 2;    // Channel for the steering servo

long duration;
int distance;

// Variable to store the incoming serial data
char receivedData;

// Distance threshold for obstacle detection
const int safeDistance = 20;    // Safe distance in cm

void setup() {
  Serial.begin(9600);                       // Start serial communication with the laptop
  pinMode(trigPin, OUTPUT);                 // Set trig pin as output
  pinMode(echoPin, INPUT);                  // Set echo pin as input

  pwm.begin();                              // Initialize PWM driver
  pwm.setPWMFreq(50);                       // Set frequency to 50Hz for servo motors

  Serial.println("Ultrasonic Sensor and Path Planning Initialized.");

  // Set wheels to stop initially
  stopMovement();
}

void loop() {
  // Check if data is available to read from serial (signal from laptop)
  if (Serial.available() > 0) {
    receivedData = Serial.read();           // Read the incoming data

    if (receivedData == '1') {              // If the signal is '1', measure the distance
      distance = getDistance();
      
      // Send the measured distance back to the laptop
      Serial.print(distance);
      Serial.println(" cm");                // Send distance value with 'cm' label for clarity

      // Perform path planning based on the distance
      if (distance < safeDistance) {
        avoidObstacle();                    // Avoid if obstacle detected within the threshold
      } else {
        moveForward();                      // Move forward if no obstacle detected
      }
    }
  }
}

// Function to get the distance from ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);               // Clear the trigPin
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);              // Set the trigPin HIGH for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);        // Read the echoPin (pulse duration)
  int distance = duration * 0.034 / 2;      // Calculate distance in cm

  return distance;
}

// Function to move forward
void moveForward() {
  Serial.println("Moving Forward");

  pwm.setPWM(leftWheelServo, 0, 300);       // Set left wheel forward speed
  pwm.setPWM(rightWheelServo, 0, 300);      // Set right wheel forward speed
  pwm.setPWM(steeringServo, 0, 375);        // Keep steering straight
}

// Function to stop movement
void stopMovement() {
  Serial.println("Stopping");

  pwm.setPWM(leftWheelServo, 0, 0);         // Stop left wheel
  pwm.setPWM(rightWheelServo, 0, 0);        // Stop right wheel
}

// Function to turn left
void turnLeft() {
  Serial.println("Turning Left");

  pwm.setPWM(steeringServo, 0, 250);        // Turn steering left
  pwm.setPWM(leftWheelServo, 0, 200);       // Reduce left wheel speed
  pwm.setPWM(rightWheelServo, 0, 300);      // Maintain right wheel speed
  delay(1000);                              // Delay to ensure turn is completed
}

// Function to turn right
void turnRight() {
  Serial.println("Turning Right");

  pwm.setPWM(steeringServo, 0, 500);        // Turn steering right
  pwm.setPWM(leftWheelServo, 0, 300);       // Maintain left wheel speed
  pwm.setPWM(rightWheelServo, 0, 200);      // Reduce right wheel speed
  delay(1000);                              // Delay to ensure turn is completed
}

// Function to avoid obstacles
void avoidObstacle() {
  stopMovement();                           // Stop the robot when obstacle is detected
  delay(500);

  // Simple avoidance algorithm: turn right, if blocked again, turn left
  turnRight();
  delay(1000);                              // Delay to let the robot turn
  
  distance = getDistance();                 // Check distance after turning
  if (distance < safeDistance) {
    turnLeft();                             // Turn left if still blocked
  }

  moveForward();                            // Continue moving forward
}
