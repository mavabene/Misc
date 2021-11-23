#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
// **** Define values for mixer matrix
const float l = 33e-3f; // in meters
const float K = 0.01; // in meters, coupling coefficient

float C_sig = mass*0; // in m/s/s

// **** Call p,q,r dot cmd
Vec3f desAng = Vec3f(0,0,0);
Vec3f desAngRate = Vec3f(0,0,0);
Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);

float desRoll = 0;
float desPitch = 0;
float desYaw = 0;

float n1 = inertia_xx*cmdAngAcc.x;
float n2 = inertia_yy*cmdAngAcc.y;
float n3 = inertia_zz*cmdAngAcc.z;


const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop
// Define new values
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);




float estRoll = 0;
float estPitch = 0;
float estYaw = 0;

float accRoll = 0;
float accPitch = 0;

const float rho = 0.01f;
// **** Add to outside loop for height estimator
float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

// ***** add desAcc's to initialize for velocity est's
float desAcc1=0;
float desAcc2=0;
float desAcc3=0;
float AccFact=0.1f;

// ***** add position var
float estPosX=0;
float estPosY=0;

float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

// **** Implement time constants for horizontal and vertical control
const float timeConst_horizVel = 2.0f;

const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// Implement time constants for angle rate control
float const timeConstant_rollRate = 0.04f; // [s]
float const timeConstant_pitchRate = timeConstant_rollRate;
float const timeConstant_yawRate = 0.1f; // [s] // Changed
// Implement time constants for angle control
float const timeConstant_rollAngle = 0.12f; // * .125f; // [s] //Changed
float const timeConstant_pitchAngle = timeConstant_rollAngle;
float const timeConstant_yawAngle = 0.2f; // [s] //Changed

MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;

  // **** Implement height estimator
  estHeight = estHeight + estVelocity_3 * dt;
  estVelocity_3 = estVelocity_3 + AccFact*desAcc3*dt; //assume constant

  // **** Measurement update
      // correction step

  if (in.heightSensor.updated) {
    float const mixHeight = 0.3f;
    // check that measurement is reasonable
    if (in.heightSensor.value < 5.0f) {
      float hMeas = in.heightSensor.value * cosf(estRoll)*cosf(estPitch);
      estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas)
          / (in.currentTime - lastHeightMeas_time);

      estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;
      // store meas for mext vel update
      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  // prediction
  // assume constant velocity
  // ***** changed 0*dt to add acceleration
  estVelocity_1 = estVelocity_1 + AccFact*desAcc1 * dt;
  estVelocity_2 = estVelocity_2 + AccFact*desAcc2 * dt;

  // ***** add position updates
  estPosX=estPosX+estVelocity_1*dt;
  estPosY=estPosY+estVelocity_2*dt;

  // correction step
  float const mixHorizVel = 0.8f;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;

    float div = (cosf(estRoll) * cosf(estPitch));
    if (div > 0.5f) {
      float deltaPredict = estHeight / div; // this is delta in the equatiopn

      float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
      float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;

      estVelocity_1 = (1 - mixHorizVel)*estVelocity_1 + mixHorizVel * v1Meas;
      estVelocity_2 = (1 - mixHorizVel)*estVelocity_2 + mixHorizVel * v2Meas;
    }
  }

  // ***** enable resetting position
  if (in.userInput.buttonGreen) {
    estPosX=0;
    estPosY=0;
  }




//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y
/*
  if (in.userInput.buttonBlue) {
    desAng.y = 30*3.1415926/180;
  }
  else {
    desAng.y = 0;
  }
  */
  // Given code by lab
  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  // Supposed to correct gyro vector with est bias
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //  Accelerometer estimates
  accRoll = float(in.imuMeasurement.accelerometer.y) / gravity;
  accPitch = -float(in.imuMeasurement.accelerometer.x) / gravity;

  // executing the integrator for attitude
  //  Added accelerometer estimation to help find attitude
  estPitch = (1.0f-rho)*(estPitch + dt*rateGyro_corr.y) + rho*accPitch;
  estRoll = (1.0f-rho)*(estRoll + dt*rateGyro_corr.x) + rho*accRoll;
  estYaw = estYaw + dt*rateGyro_corr.z;

  // after estimation
  // ***** try replacing 1/ timeConst with estPos/ time Const
  float desAcc1 = -((1 / timeConst_horizVel) * 2*estVelocity_1)-.05f*estPosX;
  float desAcc2 = -((1 / timeConst_horizVel) * 2*estVelocity_2)-.05f*estPosY;

  float desRoll = -desAcc2 / gravity;
  float desPitch = desAcc1 / gravity;
  float desYaw = 0;

  const float desHeight = 1.0f;
  const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3
      - natFreq_height * natFreq_height * (estHeight - desHeight);

  float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

  float C_sig =  mass*desNormalizedAcceleration; // in m/s/s

  cmdAngVel.x = -(estRoll - desRoll) / timeConstant_rollAngle;
  cmdAngVel.y = -(estPitch - desPitch) / timeConstant_pitchAngle;
  cmdAngVel.z = -(estYaw - desYaw) / timeConstant_yawAngle;

  cmdAngAcc.x = -(rateGyro_corr.x - cmdAngVel.x) / timeConstant_rollRate;
  cmdAngAcc.y = -(rateGyro_corr.y - cmdAngVel.y) / timeConstant_pitchRate;
  cmdAngAcc.z = -(rateGyro_corr.z - cmdAngVel.z) / timeConstant_yawRate;
  // **** get new torque vectors for motor control
  float n1 = inertia_xx*cmdAngAcc.x;
  float n2 = inertia_yy*cmdAngAcc.y;
  float n3 = inertia_zz*cmdAngAcc.z;
  // **Implement mixer matrix with total force and torque vectors
  float m1 = .25*(C_sig + n1/l + -n2/l + n3/K );
  float m2 = .25*(C_sig + -n1/l + -n2/l + -n3/K);
  float m3 = .25*(C_sig + -n1/l + n2/l + n3/K);
  float m4 = .25*(C_sig + n1/l + n2/l + -n3/K);

  outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(m1));
  outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(m2));
  outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(m3));
  outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(m4));
  // Print status every simulator step
  PrintStatus();




  // Set debug values to take attitude values
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;

  outVals.telemetryOutputs_plusMinus100[7] = desRoll;
  outVals.telemetryOutputs_plusMinus100[8] = desPitch;

  outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
  /*
  //  Set debug values to take cmdAngAcc
  outVals.telemetryOutputs_plusMinus100[3] = cmdAngAcc.x;
  outVals.telemetryOutputs_plusMinus100[4] = cmdAngAcc.y;
  outVals.telemetryOutputs_plusMinus100[5] = cmdAngAcc.z;
  // Set debug values to take cmdAngVel
  outVals.telemetryOutputs_plusMinus100[6] = cmdAngVel.x;
  outVals.telemetryOutputs_plusMinus100[7] = cmdAngVel.y;
  outVals.telemetryOutputs_plusMinus100[8] = cmdAngVel.z;

  outVals.telemetryOutputs_plusMinus100[9] = desAng.y;
  */
  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;

  return outVals;

}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  printf("Last range = %6.3fm, ", \
        double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x=%6.3f, y=%6.3f\n", \
         double(lastMainLoopInputs.opticalFlowSensor.value_x), \
         double(lastMainLoopInputs.opticalFlowSensor.value_y));

  // ***** add print for est x and y
  printf("Est X = %6.3fm, ", \
        estPosX);
  printf("Est Y = %6.3fm, ", \
        estPosY);
  /*

  //Accelerometer measurement
  printf("Acc:");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  //Rate gyro measurement
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");
  // Rate Gyro corrected measurement
  printf("Est Gyro Bias:");
  printf("Gx=%6.3f, ", double(estGyroBias.x));
  printf("Gy=%6.3f, ", double(estGyroBias.y));
  printf("Gz=%6.3f, ", double(estGyroBias.z));
  //printf(double(.rateGyro_corr.x));
  // Print correct gyro measurements
  printf("Corr Gyro:");
  printf("CorrX=%6.3f, ", double(rateGyro_corr.x));
  printf("CorrY=%6.3f, ", double(rateGyro_corr.y));
  printf("CorrZ=%6.3f, ", double(rateGyro_corr.z));
  //printf("CorrX=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x - estGyroBias.x));
  //printf("CorrY=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y - estGyroBias.y));
  //printf("CorrZ=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z - estGyroBias.z));
  printf("\n");
  // Print estimated attitude
  printf("Est Attitude:");
  printf("Pitch=%6.3f, ", double(estPitch));
  printf("Roll=%6.3f, ", double(estRoll));
  printf("Yaw=%6.3f, ", double(estYaw));
  //printf(double(estGyroBias.x));

*/

/*
  printf("Example variable values:\n");
  printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.userInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.userInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.userInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.userInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.userInput.buttonArm)
    printf("buttonArm ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));
  */
}

