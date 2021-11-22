#include <MedianFilter.h>
#include <SharpDistSensor.h>
#include <CapacitiveSensor.h>
#include "FeedBackServo.h"
#include <Adafruit_NeoPixel.h>


// define circuit pins
#define IR_DISTANCE_SENSOR_PIN A0
#define ULTRASONIC_DISTANCE_SENSOR A1

#define FEEDBACK_PIN 2
#define SERVO_PIN 3
#define SPEAKER_PIN 5
#define LED_STRIP_PIN 6
#define LIGHT_PIN 8
#define CAP_SENSOR_PIN_A 9
#define CAP_SENSOR_PIN_B 10
#define DOOR_SERVO_PIN 13

#define LED_COUNT 15

#define radius 15 // 15mm radius of gear
#define rackDistanceTotal 200 // 250 mm

// set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

CapacitiveSensor cs = CapacitiveSensor(CAP_SENSOR_PIN_A, CAP_SENSOR_PIN_B);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired
long initialCapSensorVal = -1;

// Analog pin to which the sensor is connected
// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;
// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(IR_DISTANCE_SENSOR_PIN, medianFilterWindowSize);

Servo doorServo;  // create servo object to control a servo

Adafruit_NeoPixel strip(LED_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

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
String TOUCH = "touch";
String INITIALIZING = "initializing";
String DEVELOPMENT = "development";

String currState = DEVELOPMENT;

void setup() {
    Serial.begin(9600);

    // while the serial stream is not open, do nothing:
   while (!Serial) {
    delay(10);
   }

   cs.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   initialCapSensorVal = cs.capacitiveSensor(30);
//    Serial.print("Initial CapSensorVal: ");
//    Serial.println(initialCapSensorVal);
  
    // set servo control pin number
    servo.setServoControl(SERVO_PIN);
    servo.setKp(1.0);

    // Set sensor model
    sensor.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

    doorServo.attach(DOOR_SERVO_PIN);  // attaches the servo on pin 9 to the servo object

    //  servo.rotate(0, 4);

    initialAngle = servo.Angle();
    Serial.print("Initial anlge: ");
    Serial.println(initialAngle);
    currAngle = initialAngle;

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    pinMode(LIGHT_PIN, OUTPUT);
}

float getDistanceOfRack(int angle) {
  return (angle - initialAngle) * distancePerDegree;
}

float getUltraSonicDistanceValue() {
  float distance1 = analogRead(ULTRASONIC_DISTANCE_SENSOR)/2.0;
  return distance1 * 2.54;
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

void openDoor() {
  Serial.print("Opening servo door: ");
  doorServo.write(180);
  delay(15);        
}

void closeDoor() {
  Serial.print("Closing servo door: ");
  doorServo.write(0);
  delay(15);        
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

void loop() {
//   Serial.println("-----------------------");
    // rotate servo to 270 and -180 degrees(with contains +-4 degrees error) each 1 second.
   
    Serial.print("Curr state: ");
    Serial.println(currState);

    rainbow(10);

    // Get distance from sensor
    unsigned int distanceToHand = sensor.getDist();
    float distanceOfRack = getDistanceOfRack(currAngle);
  
    // Print distance to Serial
    Serial.print("Distance from sensor: ");
    Serial.println(distanceToHand);
//    Serial.print("Distance of rack: ");
//    Serial.println(distanceOfRack);

    float ultraSonicDistance = getUltraSonicDistanceValue();
    Serial.print("Distance from ultra sonic sensor: ");
    Serial.println(ultraSonicDistance);

    long capacitiveTouchSensorIn =  cs.capacitiveSensor(30);
    Serial.print("capacitiveTouchSensorIn: ");
    Serial.println(capacitiveTouchSensorIn);
    if (initialCapSensorVal <= 0 && capacitiveTouchSensorIn > 100) {
        initialCapSensorVal = capacitiveTouchSensorIn;
        Serial.print("Initial CapSensorVal: ");
        Serial.println(initialCapSensorVal);
    }

    openDoor();
    delay(1000);
    closeDoor();
    delay(1000);

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

    
    if ((currState == READY || currState == TOUCH || currState == DEVELOPMENT) && capacitiveTouchSensorIn > 4 * initialCapSensorVal) {
      // STATE: user is touching finger
      currState = TOUCH;
      Serial.println("TOUCH");
    } else if (currState == TOUCH) {
      // STATE: user stopped touching finger
       Serial.print("RETRACTING!!! due to end of touch.");
       retract();
    } else if (currState == READY) {
      // STATE: user is potentially approching finger
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
    } else if (currState ==  INITIALIZING || currState == RETRACTING) {
      // STATE: finger is retracting or app is initializing
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

//    if (isForwardDirection && distanceOfRack >= (rackDistanceTotal - endOfRackThreshold)) {
//      isForwardDirection = false;
//    } else if (!isForwardDirection && distanceOfRack < endOfRackThreshold) {
//      isForwardDirection = true;
//    }

    digitalWrite(LIGHT_PIN, currState == READY);

    delay(20);

//    
//    Serial.print("Servo angle reading: ");
//    Serial.print(servo.Angle());
//    Serial.println("°");


}
