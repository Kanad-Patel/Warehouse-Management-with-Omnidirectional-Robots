#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <math.h>

// Wi-Fi credentials
const char* ssid = "Hotspot";
const char* password = "hi123456";

// Create a web server on port 80
WebServer server(80);

// Servo objects for the wheels and gripper
Servo servo1, servo2, servo3, gripperServo;

// GPIO pins for the servos
const int servoPin1 = 17; // Wheel 1
const int servoPin2 = 18; // Wheel 2
const int servoPin3 = 19; // Wheel 3
const int gripperPin = 16; // Gripper servo on GPIO 16

// Constants for the robot
const float wheelRadius = 0.029;    // Radius of the wheel in meters (29 mm)
const float robotRadius = 0.16;     // Distance from the center to the wheel in meters (160 mm)
const int stopPulse = 1500;         // Neutral position
const int maxPWMRange = 400;        // Max PWM range for full speed adjustment

// Gripper angles
const int gripperMinAngle = -65; // Minimum gripper angle
const int gripperMaxAngle = 65;  // Maximum gripper angle

// Grid and scaling factors
const float gridWidth = 1.5;      // Grid width in meters (150 cm)
const float gridHeight = 0.6;     // Grid height in meters (60 cm)
const float speed = 0.2;          // Movement speed in meters per second

// Robot positions
float currentX = 0.0, currentY = 0.0; // Robot's initial position
float targetX = 0.0, targetY = 0.0;   // Target position

// Function to calculate the PWM signal for each servo
void moveRobot(float Vx, float Vy, float omega) {
  float v1 = (Vx) / wheelRadius;                           // Wheel 1 velocity
  float v2 = (-sqrt(3) / 2 * Vx - 0.5 * Vy) / wheelRadius; // Wheel 2 velocity
  float v3 = (sqrt(3) / 2 * Vx - 0.5 * Vy) / wheelRadius;  // Wheel 3 velocity

  int pwm1 = stopPulse + constrain(v1 * maxPWMRange, -maxPWMRange, maxPWMRange);
  int pwm2 = stopPulse + constrain(v2 * maxPWMRange, -maxPWMRange, maxPWMRange);
  int pwm3 = stopPulse + constrain(v3 * maxPWMRange, -maxPWMRange, maxPWMRange);

  servo1.writeMicroseconds(pwm1);
  servo2.writeMicroseconds(pwm2);
  servo3.writeMicroseconds(pwm3);

  // Log PWM signals
  Serial.printf("Servo PWM: PWM1 = %d, PWM2 = %d, PWM3 = %d\n", pwm1, pwm2, pwm3);
}

// Function to stop all movement
void stopRobot() {
  moveRobot(0, 0, 0);
  Serial.println("Robot stopped.");
}

// Function to move the gripper
void moveGripper() {
  Serial.println("Gripper: Moving...");
  for (int angle = gripperMinAngle; angle <= gripperMaxAngle; angle++) {
    gripperServo.write(90 + angle); // Offset by 90° for standard servo range
    delay(20);
  }
  for (int angle = gripperMaxAngle; angle >= gripperMinAngle; angle--) {
    gripperServo.write(90 + angle);
    delay(20);
  }
  Serial.println("Gripper: Movement completed.");
}

// Function to move the robot to target coordinates
void moveToCoordinates() {
  Serial.printf("Starting movement to target (%.2f, %.2f) from (%.2f, %.2f)...\n", targetX, targetY, currentX, currentY);

  // Calculate velocities to reach the target
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;

  // Normalize direction and maintain speed
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  float Vx = (deltaX / distance) * speed; // Velocity in X direction
  float Vy = (deltaY / distance) * speed; // Velocity in Y direction

  unsigned long startTime = millis(); // Record start time
  while (distance > 0.05 && (millis() - startTime) < 3000) { // Limit motion to 3 seconds
    moveRobot(Vx, Vy, 0); // Move with calculated velocity
    delay(100);

    // Simulate encoder feedback to update the current position
    currentX += Vx * 0.1; // Assumes 0.1 seconds per loop
    currentY += Vy * 0.1;

    // Recalculate distance
    distance = sqrt((targetX - currentX) * (targetX - currentX) + (targetY - currentY) * (targetY - currentY));

    // Log current position
    Serial.printf("Current Position: X = %.2f, Y = %.2f, Remaining Distance = %.2f\n", currentX, currentY, distance);
  }

  // Stop the robot
  stopRobot();
  Serial.println("Robot: Reached target position or timeout!");

  // Move the gripper after stopping
  moveGripper();
}

// HTTP handler for the root endpoint
void handleRoot() {
  Serial.println("HTTP: Root endpoint accessed.");
  server.send(200, "text/plain", "ESP32 is ready to receive commands!");
}

// HTTP handler for setting initial and target coordinates
void handleSetCoordinates() {
  if (server.hasArg("robotX") && server.hasArg("robotY") && server.hasArg("objectX") && server.hasArg("objectY")) {
    // Parse coordinates from the HTTP request
    currentX = server.arg("robotX").toFloat() * gridWidth;  // Scale to meters
    currentY = server.arg("robotY").toFloat() * gridHeight; // Scale to meters
    targetX = server.arg("objectX").toFloat() * gridWidth;  // Scale to meters
    targetY = server.arg("objectY").toFloat() * gridHeight; // Scale to meters

    // Log received coordinates
    Serial.printf("Received Coordinates:\n");
    Serial.printf("  Current Position: X = %.2f, Y = %.2f\n", currentX, currentY);
    Serial.printf("  Target Position: X = %.2f, Y = %.2f\n", targetX, targetY);

    // Acknowledge receipt
    server.send(200, "text/plain", "Coordinates received!");

    // Move the robot
    moveToCoordinates();
  } else {
    Serial.println("HTTP: Missing parameters in /set_coordinates request.");
    server.send(400, "text/plain", "Missing required parameters!");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32: Starting...");

  // Attach servos
  servo1.attach(servoPin1, 500, 2500);
  servo2.attach(servoPin2, 500, 2500);
  servo3.attach(servoPin3, 500, 2500);
  gripperServo.attach(gripperPin, 500, 2500);
  Serial.println("Servos: Initialized.");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.printf("Wi-Fi: Connecting to %s...\n", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi: Connected.");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());

  // Define HTTP routes
  server.on("/", handleRoot);
  server.on("/set_coordinates", handleSetCoordinates);

  // Start the server
  server.begin();
  Serial.println("HTTP Server: Started.");
}

void loop() {
  server.handleClient();  // Handle incoming HTTP requests
}
