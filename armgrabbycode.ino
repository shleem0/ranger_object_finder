#include <InverseK.h>
#include "BraccioV2.h"

Braccio arm;

#define GRIPPER_CLOSED 60
#define GRIPPER_OPENED 20

#define d_1 7
#define d_5 16
#define a_2 13
#define a_3 12.5
#define Q 0
#define P 0

void setup() {
  Serial.begin(9600);
  Serial.println("lalalalalalal");
  Serial.println("Initializing... Please Wait");

  arm.setJointCenter(WRIST_ROT, 90);
  arm.setJointCenter(WRIST, 10);
  arm.setJointCenter(ELBOW, 10);
  arm.setJointCenter(SHOULDER, 40);
  arm.setJointCenter(BASE_ROT, 90); // Starting position at 150°
  arm.setJointCenter(GRIPPER, 50);

  arm.setJointMax(GRIPPER, 100);
  arm.setJointMin(GRIPPER, 15);

  arm.begin(true);
  Serial.println("Initialization Complete");
  
  
  //arm.setOneAbsolute(GRIPPER, 20); //Open
  //arm.safeDelay(2000);
  //moveToCoordinates(15, -10, -20);
  //arm.safeDelay(2000);
  //arm.setOneAbsolute(GRIPPER, 70); //closed

  
}

//order of operations
//opens gripper
//moves to coordinates
//closes gripper
//elbow returns to 90
//wrist retuns to 90
//shoulder returns to 90

void loop() {
  

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); // Read until a newline (Enter key)
    data.trim(); // Trim spaces and newlines

    // Debugging output: Show the raw data
    Serial.print("Raw data: ");
    Serial.println(data); // Print what was read from the Serial Monitor

    // Debugging: Print ASCII values of each character in the raw data
    Serial.print("Character ASCII values: ");
    for (int i = 0; i < data.length(); i++) {
      Serial.print((int)data[i]); // Print the ASCII value of each character
      Serial.print(" ");
    }
    Serial.println(); // Newline for readability

    // Manually parse the data using substring and toFloat()
    int firstComma = data.indexOf(',');
    int secondComma = data.lastIndexOf(',');

    if (firstComma != -1 && secondComma != -1 && firstComma != secondComma) {
      // Extract P_x, P_y, P_z from the string
      String P_x_str = data.substring(0, firstComma);
      String P_y_str = data.substring(firstComma + 1, secondComma);
      String P_z_str = data.substring(secondComma + 1);

      // Convert them to floats
      float P_x = P_x_str.toFloat();
      float P_y = P_y_str.toFloat();
      float P_z = P_z_str.toFloat();

      // Debugging: Print parsed values
      Serial.print("Parsed P_x: ");
      Serial.println(P_x);
      Serial.print("Parsed P_y: ");
      Serial.println(P_y);
      Serial.print("Parsed P_z: ");
      Serial.println(P_z);

      // Proceed with your code after parsing
      arm.setOneAbsolute(GRIPPER, GRIPPER_OPENED);
      arm.safeDelay(1000); // Wait for gripper to open

      // Move to coordinates
      moveToCoordinates(P_x, P_y, P_z);
      arm.safeDelay(1000);

      // Close the gripper
      arm.setOneAbsolute(GRIPPER, GRIPPER_CLOSED);
      arm.safeDelay(1000);

      // Reset positions
      arm.setOneAbsolute(ELBOW, 10);
      arm.setOneAbsolute(WRIST, 10);
      arm.setOneAbsolute(SHOULDER, 40);
      arm.safeDelay(1000);
      

      Serial.println("Sequence complete. Waiting for new input...");
    } else {
      // If parsing failed, print error message
      Serial.println("Error: Invalid input format.");
    }
  }
}



void moveToCoordinates(float P_x, float P_y, float P_z) {
  // Calculate the base angle, offsetting it by 150° instead of 90°
  Serial.println(P_x);
  Serial.println(P_y);
  float theta1 = atan2(P_y, P_x);
  Serial.print("Calculated base rad: ");
  Serial.println(theta1);
  
  // Use M_PI instead of P_PI
  float baseAngle = theta1 * 180 / M_PI + 90 ;
  Serial.print("Calculated base angle: ");
  Serial.println(baseAngle);

  // Calculate X_4, Y_4, and Z_4
  float X_4 = P_x - (d_5 * sin(Q) * cos(theta1));
  Serial.print("Calculate X_4 in rad: ");
  Serial.println(X_4);
  float Y_4 = P_y - (d_5 * sin(Q) * sin(theta1));
  Serial.print("Calculate Y_4 in rad: ");
  Serial.println(Y_4);
  float Z_4 = P_z + (d_5 * cos(Q)) - d_1;
  Serial.print("Calculate Z_4 in rad: ");
  Serial.println(Z_4);

  // Correct variable declarations
  float l = sqrt(P_x * P_x + P_y * P_y);
  Serial.print("Calculate l in rad: ");
  Serial.println(l);
  float m = sqrt(l * l + Z_4 * Z_4);
  Serial.print("Calculate m in rad: ");
  Serial.println(m);
  float cosTheta1 = (pow(m, 2) + pow(a_2, 2) - pow(a_3, 2)) / (2 * m * a_2);
  cosTheta1 = constrain(cosTheta1, -1, 1); // Ensure it's within range
  float n = acos(cosTheta1);
  Serial.print("Calculate n in rad: ");
  Serial.println(n);

  float theta2 = (M_PI / 2) - (atan2(Z_4, l) + n); 
  Serial.print("Calculate theta2 in rad: ");
  Serial.println(theta2);
  float shoulderAngle = 90 - (theta2 * 180 / M_PI);
  Serial.print("Calculated shoulder angle: ");
  Serial.println(shoulderAngle);
  float shoulderAngle2 = 180 - shoulderAngle;
  Serial.println(shoulderAngle2);
  shoulderAngle2 = constrain(shoulderAngle2, 20, 160);
  Serial.println(shoulderAngle2);

  float cosTheta2 = (pow(a_2, 2) + pow(a_3, 2) - pow(m, 2)) / ( 2 * a_2 * a_3);
  cosTheta2 = constrain(cosTheta2, -1, 1);
  float theta3 = acos(cosTheta2);
  Serial.print("Calculate theta3 in rad: ");
  Serial.println(theta3);
  float elbowAngle = (theta3 * 180 / M_PI) - 90;
  Serial.print("Calculated elbow angle: ");
  Serial.println(elbowAngle);

  float cosTheta3 = (pow(a_2, 2) + pow(a_3, 2) - pow(m, 2)) / (2 * a_2 * a_3);
  cosTheta3 = constrain(cosTheta3, -1.0, 1.0); // Ensure it's within valid range
  float theta4 = (3 * (M_PI / 2)) - ((atan2(Z_4, l) + n) + acos(cosTheta3)) + Q;
  Serial.print("Calculate theta4 in rad: ");
  Serial.println(theta4);
  float wristAngle = (theta4 * 180 / M_PI) - 90 + 5;
  Serial.print("Calculated wrist angle: ");
  Serial.println(wristAngle);


  // Move all the joints at the same time
  arm.setOneAbsolute(BASE_ROT, baseAngle);
  arm.setOneAbsolute(SHOULDER, shoulderAngle2);
  arm.setOneAbsolute(ELBOW, elbowAngle);
  arm.setOneAbsolute(WRIST, wristAngle);
  arm.safeDelay(1000);
  arm.setOneAbsolute(GRIPPER, 70);

}
