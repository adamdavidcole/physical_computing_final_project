#include <MedianFilter.h>
#include <SharpDistSensor.h>

#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 3

#define radius 15 // 15mm radius of gear
#define rackDistanceTotal 200 // 250 mm

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);


// Analog pin to which the sensor is connected
const byte sensorPin = A0;
// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;
// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

int currAngle = 0;
int initialAngle = 0;
int angleIncrement = 10;
int endOfRackThreshold = 10;

float distancePerDegree = (2 * PI * radius) / 360;

boolean isForwardDirection = true;

void setup() {
    Serial.begin(9600);

    // while the serial stream is not open, do nothing:
   while (!Serial) ;
    
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);

    // Set sensor model
    sensor.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

    //  servo.rotate(0, 4);

    initialAngle = servo.Angle();
    Serial.print("Initial anlge: ");
    Serial.println(initialAngle);
    currAngle = initialAngle;
}

float getDistanceOfRack(int angle) {
  return (angle- initialAngle) * distancePerDegree;
}

void loop() {
   Serial.println("-----------------------");
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
    int nextAngle = currAngle;

    if (isForwardDirection) {
      nextAngle = currAngle + angleIncrement;
  
      if (abs(nextAngle % 360) <= 30) {
        nextAngle += 30;
      }
    } else {
      nextAngle = currAngle - angleIncrement;
  
      if (abs(nextAngle % 360) <= 30) {
        nextAngle -= 30;
      }
    }

    
  
    
    servo.rotate(nextAngle, 2);
    currAngle = nextAngle;

    float distanceOfRack = getDistanceOfRack(currAngle);
    Serial.print("Distance of rack: ");
    Serial.print(distanceOfRack);
    Serial.println("mm");

    Serial.print("Current angle: ");
    Serial.print(currAngle);
    Serial.println("°");

    if (isForwardDirection && distanceOfRack >= (rackDistanceTotal - endOfRackThreshold)) {
      isForwardDirection = false;
    } else if (!isForwardDirection && distanceOfRack < endOfRackThreshold) {
      isForwardDirection = true;
    }

    delay(50);

    
    Serial.print("Servo angle reading: ");
    Serial.print(servo.Angle());
    Serial.println("°");

    // Get distance from sensor
    unsigned int distance = sensor.getDist();
  
    // Print distance to Serial
    Serial.print("Distance from sensor: ");
    Serial.println(distance);
}
