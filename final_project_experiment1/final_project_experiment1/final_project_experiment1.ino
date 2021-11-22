#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 3

#define radius 15 // 15mm radius of gear

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

int currAngle = 0;
int initialAngle = 0;
int angleIncrement = 1;

float distancePerDegree = (2 * PI * radius) / 360;

void setup() {
    Serial.begin(9600);

    // while the serial stream is not open, do nothing:
   while (!Serial) ;
    
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);

//    servo.rotate(0, 4);

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
    int nextAngle = currAngle + angleIncrement;

    if (abs(nextAngle % 360) <= 15) {
      nextAngle+= 30;
    }
    
    servo.rotate(nextAngle, 4);
    currAngle = nextAngle;

    float distanceOfRack = getDistanceOfRack(currAngle);
    Serial.print("Distance of rack: ");
    Serial.print(distanceOfRack);
    Serial.println("mm");

    Serial.print("Current angle: ");
    Serial.print(currAngle);
    Serial.println("°");

    delay(50);

    
//    Serial.print("Servo angle reading: ");
//    Serial.print(servo.Angle());
//    Serial.println("°");
}
