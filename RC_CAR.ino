#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include "WiFiS3.h" 
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiServer server(4000);
WiFiClient client;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(3);  // Motor 3 = Left
Adafruit_DCMotor *motorRight = AFMS.getMotor(4); // Motor 4 = Right

// Input pins for RC receiver
const int CH1_PIN = 2;  // Steering
const int CH2_PIN = 3;  // Throttle
const int DEADZONE = 20;

void waitForSignal(){
  int pwm;
  do {
    pwm = pulseIn(CH2_PIN, HIGH, 25000);
    Serial.println("Waiting for valid signal...");
    delay(500);
  } while (pwm < 900 || pwm > 21000);
}

void setup() {
  Serial.begin(115200);
  AFMS.begin();
  WiFi.setHostname("RC_CAR");
  WiFi.begin(ssid,pass);
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay (1000);
    Serial.print(".");
  }
  
  server.begin();
  //server.setNoDelay(true);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);

//  waitForSignal();
}

void loop() {
  if (!client || !client.connected()) {
    Serial.println("Listening for client");
    //printWifiStatus();
    client = server.available();
  }
  int throttlePWM = pulseIn(CH2_PIN, HIGH, 25000); // Throttle
  int steeringPWM = pulseIn(CH1_PIN, HIGH, 25000); // Steering

  // Default to center if signal is not received
  if (throttlePWM == 0) throttlePWM = 1468;
  if (steeringPWM == 0) steeringPWM = 1468;

  bool validThrottle = (throttlePWM >= 900 && throttlePWM <= 2100);
  bool validSteering = (steeringPWM >= 900 && steeringPWM <= 2100);

  if (!validThrottle || !validSteering) {
    if (client && client.connected()){
      client.println("Invalid signal: ");
      client.print("  Throttle PWM: "); client.print(throttlePWM);
      client.print("  Steering PWM: "); client.println(steeringPWM);
    }
  }
  else{
    // Convert RC input (1000–2000 µs) to -255 to 255
    int throttle = map(throttlePWM, 979, 1958, -255, 255);
    int steering = map(steeringPWM, 979, 1957, -255, 255);

    // Apply Deadzone
    if (abs(throttle) < DEADZONE) throttle = 0;
    if (abs(steering) < DEADZONE) steering = 0;

    // Combine throttle and steering for tank drive
    int leftSpeed = throttle + steering;
    int rightSpeed = throttle - steering;

    // Constrain speed to acceptable motor range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    if (client && client.connected()){
      client.print("Throttle PWM: "); client.print(throttlePWM);
      client.print("  Steering PWM: "); client.print(steeringPWM);
      client.print("  Mapped Throttle: "); client.print(throttle);
      client.print("  Mapped Steering: "); client.print(steering);
      client.print("  Left Speed: "); client.print(leftSpeed);
      client.print("  Right Speed: "); client.println(rightSpeed);
    }

    // Drive left motor
    if (leftSpeed > 0) {
      motorLeft->setSpeed(leftSpeed);
      motorLeft->run(FORWARD);
    } else {
      motorLeft->setSpeed(-leftSpeed);
      motorLeft->run(BACKWARD);
    }

    // Drive right motor
    if (rightSpeed > 0) {
      motorRight->setSpeed(rightSpeed);
      motorRight->run(BACKWARD);
    } else {
      motorRight->setSpeed(-rightSpeed);
      motorRight->run(FORWARD);
    }
  }
  delay(200); // Small delay to stabilize loop
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
