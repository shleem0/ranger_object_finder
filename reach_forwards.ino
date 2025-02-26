////////////////////////////////////////////////////////////////////////////////////////////////////////
//Initialisations
////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <Servo.h>
#include "BraccioV2.h"

Braccio arm;

#define GRIPPER_CLOSED 70
#define GRIPPER_OPENED 20

// Define variables to store the current joint positions
int shoulderPosition = 90;
int elbowPosition = 90;
int wristPosition = 90;
int gripperPosition = 50;



////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup
////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing... Please Wait");

  arm.setJointCenter(WRIST_ROT, 90);
  arm.setJointCenter(WRIST, 90);
  arm.setJointCenter(ELBOW, 90);
  arm.setJointCenter(SHOULDER, 90);
  arm.setJointCenter(BASE_ROT, 90);
  arm.setJointCenter(GRIPPER, 50);

  arm.setJointMax(GRIPPER, 100);
  arm.setJointMin(GRIPPER, 15);

  arm.begin(true);
  Serial.println("Initialization Complete");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read input from Serial Monitor
    command.trim();  // Remove any extra spaces or newlines

    if (command == "forward") {
      moveForward();
    } else if (command == "grab") {
      grabObject();
    } else if (command == "reset") {
      resetPosition();
    } else if (command == "open") {
      openPosition();
    } else if (command == "close") {
      closePosition();
    } else if (command == "hold") {
      holdPosition();
    }
    
    
    
    else {
      Serial.println("Unknown command. Use 'forward', 'grab', or 'reset'.");
    }
  }
}

void moveJointGradually(int joint, int targetPosition, int delayTime) {
  int *currentPosition = nullptr;
  
  // Determine which joint's position to modify
  if (joint == SHOULDER) {
    currentPosition = &shoulderPosition;
  } else if (joint == ELBOW) {
    currentPosition = &elbowPosition;
  } else if (joint == WRIST) {
    currentPosition = &wristPosition;
  } else if (joint == GRIPPER) {
    currentPosition = &gripperPosition;
  }

  // Gradually move the joint towards the target position
  if (currentPosition != nullptr) {
    int stepSize = (targetPosition > *currentPosition) ? 1 : -1;
    
    while (*currentPosition != targetPosition) {
      *currentPosition += stepSize;
      arm.setOneAbsolute(joint, *currentPosition);
      arm.safeDelay(delayTime);  // Slow down the movement with a delay
    }
  }
}

void moveForward() {
  Serial.println("Moving forward...");
  moveJointGradually(SHOULDER, 20, 20);  // Move shoulder gradually
  moveJointGradually(ELBOW, 0, 20);      // Move elbow gradually
  moveJointGradually(WRIST, 80, 20);     // Move wrist gradually
}

void grabObject() {
  Serial.println("Grabbing object...");
  openGripper();
  arm.safeDelay(2000);
  closeGripper();
  arm.safeDelay(2000);
}

void holdPosition() {
  Serial.println("Holding...");
  moveJointGradually(SHOULDER, 90, 20);
  moveJointGradually(ELBOW, 0, 20);
  moveJointGradually(WRIST, 180, 20);
}

void openPosition() {
  Serial.println("Opening...");
  openGripper();
  arm.safeDelay(2000);
}

void closePosition() {
  Serial.println("Closing...");
  closeGripper();
  arm.safeDelay(2000);
}


////////////////////////////////////////////////////////////////////////////////
// Often Used Functions
////////////////////////////////////////////////////////////////////////////////

void openGripper() {
  Serial.println("Opening gripper...");
  gripperPosition = GRIPPER_OPENED;
  arm.setOneAbsolute(GRIPPER, gripperPosition);
}

void closeGripper() {
  Serial.println("Closing gripper...");
  gripperPosition = GRIPPER_CLOSED;
  arm.setOneAbsolute(GRIPPER, gripperPosition);
}

void resetPosition() {
  Serial.println("Resetting to default position...");
  shoulderPosition = 90;
  elbowPosition = 90;
  wristPosition = 90;
  gripperPosition = 50;

  arm.setOneAbsolute(SHOULDER, shoulderPosition);
  arm.setOneAbsolute(ELBOW, elbowPosition);
  arm.setOneAbsolute(WRIST, wristPosition);
  arm.setOneAbsolute(GRIPPER, gripperPosition);
  arm.safeDelay(3000);
}
