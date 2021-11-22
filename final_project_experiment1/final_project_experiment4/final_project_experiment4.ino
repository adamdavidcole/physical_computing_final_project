#include <MedianFilter.h>
#include <SharpDistSensor.h>

#include "FeedBackServo.h"
// define feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 3
#define LIGHT_PIN 8

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
int endOfRackThreshold = 1;

int farthesHandDistance = 395;
int lastHandDistance = farthesHandDistance;

float distancePerDegree = (2 * PI * radius) / 360;
float degreePerDistance = 1 / distancePerDegree;

boolean isForwardDirection = true;

int timeToWaitBeforeReady = 3000;
int lastRetractTime = millis();


// STATES
String RETRACTING = "retracting";
String READY = "ready";
String INITIALIZING = "initializing"; 
String currState = INITIALIZING;

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

    pinMode(LIGHT_PIN, OUTPUT);

//    currState = READY;
}

float getDistanceOfRack(int angle) {
  return (angle - initialAngle) * distancePerDegree;
}

void moveForward(float dist) {
  int degreesToMove = round(degreePerDistance * dist);
  int nextAngle = currAngle - degreesToMove;

   int modNextAngle = abs(nextAngle % 360);
   if (modNextAngle <= 30 || modNextAngle >= 330) {
      nextAngle -= 30;
   }

   Serial.println("Next angle: ");
   Serial.println(nextAngle);
   servo.rotate(nextAngle, 4);

   currAngle = nextAngle;
}

void moveBackward(float dist) {
  int degreesToMove = round(degreePerDistance * dist);
  int nextAngle = currAngle + degreesToMove;

   int modNextAngle = abs(nextAngle % 360);
   if (modNextAngle <= 30 || modNextAngle >= 330) {
//      nextAngle += 15;
   }

   servo.rotate(nextAngle, 4);

   currAngle = nextAngle;
}

void retract() {
   servo.rotate(initialAngle, 4);
   currAngle = initialAngle;
   
   lastHandDistance = farthesHandDistance;
   currState = RETRACTING;
   lastRetractTime = millis();
}

void loop() {
//   Serial.println("-----------------------");
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
   


    // Get distance from sensor
    unsigned int distanceToHand = sensor.getDist();
    float distanceOfRack = getDistanceOfRack(currAngle);
  
    // Print distance to Serial
    Serial.print("Distance from sensor: ");
    Serial.println(distanceToHand);
//    Serial.print("Distance of rack: ");
//    Serial.println(distanceOfRack);

//
//    Serial.print("distanceToHand < farthesHandDistance: ");
//    Serial.println(distanceToHand < farthesHandDistance);
//    Serial.print("distanceToHand < lastHandDistance: ");
//    Serial.println(distanceToHand < lastHandDistance);
//    Serial.print("distanceToHand < lastHandDistance: ");
//    Serial.println(abs(distanceOfRack) < (rackDistanceTotal - endOfRackThreshold));

// 

//
    int distanceToHandModifer = 0;
    if (currState == READY) {
      if (distanceToHand < farthesHandDistance && distanceToHand < lastHandDistance && abs(distanceOfRack) < (rackDistanceTotal - endOfRackThreshold)) {
         int distToHandChange = lastHandDistance - distanceToHand;
  
          Serial.print("Distance from sensor change: ");
          Serial.println(distToHandChange);
  
         if (distToHandChange >= 0 && distToHandChange <= 100) {
            float distToMove = distToHandChange / 2.0;
            Serial.print("distToMove: ");
            Serial.println(distToMove);

            distanceToHandModifer = -distToMove;
            moveForward(distToMove);
         }

  
         if (distToHandChange > 100) {
          Serial.print("RETRACTING!!! due to : ");
          Serial.println(distToHandChange);
          retract(); 
         }

        lastHandDistance = distanceToHand + distanceToHandModifer; 
      } 
    }
    else if (currState ==  INITIALIZING || currState == RETRACTING) {
      int timeSinceLastRetract = millis() - lastRetractTime;
      boolean hasEnoughTimePassedSinceLastRetract = timeSinceLastRetract >= timeToWaitBeforeReady;
      
      // ensure user does not have hand in field of rack
      if (distanceToHand > farthesHandDistance && hasEnoughTimePassedSinceLastRetract) {
          currState = READY;
      }
    }

//    lastHandDistance = distanceToHand + distanceToHandModifer;
//    else if (distanceToHand < farthesHandDistance && distanceToHand > lastHandDistance) {
//       int distToHandChange = distanceToHand - lastHandDistance;
//
//       if (distToHandChange >= 10 && distToHandChange <= 20) {
//         Serial.print("Distance from sensor change: ");
//         Serial.println(distToHandChange);
//  
//         moveBackward(distToHandChange / 2.0);
//
//         lastHandDistance = distanceToHand;
//       }
//    }

   
//
//    if (isForwardDirection) {
//       Serial.println("Move forward by increment");
//      nextAngle = currAngle + angleIncrement;
//  
//      if (abs(nextAngle % 360) <= 30) {
//        nextAngle += 30;
//      }
//    } else {
//       Serial.println("Move backward by increment");
//      nextAngle = currAngle - angleIncrement;
//  
//      if (abs(nextAngle % 360) <= 30) {
//        nextAngle -= 30;
//      }
//    }

//    nextAngle = currAngle - angleIncrement;
    

   

    
//    Serial.print("Distance of rack: ");
//    Serial.print(distanceOfRack);
//    Serial.println("mm");
//
//    Serial.print("Current angle: ");
//    Serial.print(currAngle);
//    Serial.println("°");

    if (isForwardDirection && distanceOfRack >= (rackDistanceTotal - endOfRackThreshold)) {
      isForwardDirection = false;
    } else if (!isForwardDirection && distanceOfRack < endOfRackThreshold) {
      isForwardDirection = true;
    }

    digitalWrite(LIGHT_PIN, currState == READY);

    delay(20);

//    
//    Serial.print("Servo angle reading: ");
//    Serial.print(servo.Angle());
//    Serial.println("°");


}
