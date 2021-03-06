#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <C610Bus.h>
#include <BasicLinearAlgebra.h>
#include "Wire.h"

// #define PRINT_DATA   // Define for human readable serial output, comment out for MATLAB serial readable

enum ControllerState {
   IDLE,    // Controller is not balancing, but will try to balance if close to upright
   ACTIVE,  // Controller is trying to balance
   STOPPED  // Controller is not balancing, will not try to balance
};

ControllerState currentState;

MPU6050 mpu;
C610Bus bus;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

#define IMU_pin 6

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

bool blink_state = false;

long lastPrintTime;
long lastCommandTime;
long currentControlLoopTime;
long lastControlLoopTime;
float controlLoopDeltaT;

const int printDelay = 20000;
const int commandDelay = 10000;

int16_t ax, ay, az, gx, gy, gz;
float yaw, pitch, roll, rollRate, pitchRate;
float measPitchMotorVel, measRollMotorVel, measPitchMotorTorque, measRollMotorTorque;

float filtRollRate = 0.0, filtPitchRate = 0.0;
float alpha = 0.35;

const float invTorqueConstant = 5555.55; // 1/0.00018 mA/Nm
int cmdPitchMotorCurrent, cmdRollMotorCurrent;
float cmdPitchTorque, cmdRollTorque;
const float maxTorque = 1.5;

float qRoll = 0.0, qPitch = 0.0;
const float maxWindup = 200;     // Max windup for wheel velocity integrator, increase if the IMU zero is
                                 // far away from equilibrium

using namespace BLA;

Matrix<2,8> K = {0, 0, 0, 0, 25, 3.1012, 0.025, -0.025,
                 25, 3.1012, 0.025, -0.025, 0, 0, 0, 0};

// Matrix<2,8> K = {0, 0, 0, 0, 25.0442, 3.5698, 0.025, -0.0750,
//                  25.0442, 3.5698, 0.025, -0.0750, 0, 0, 0, 0};

// Matrix<2,8> K = {0, 0, 0, 0, 25, 3.1012, 0.010, -0.025,
//                  25, 3.1012, 0.010, -0.025, 0, 0, 0, 0};
 
Matrix<8,1> x;
Matrix<2,1> u;
Matrix<2,1> r = {0, 0};

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void changeState(ControllerState nextState) {
   // Helper function to change state
   if (nextState == IDLE || nextState == STOPPED) {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
      digitalWrite(LED_BUILTIN, true);
   } else if (nextState == ACTIVE) {
      qRoll = 0.0;
      qPitch = 0.0;
      lastControlLoopTime = micros();
      digitalWrite(LED_BUILTIN, false);
      mpu.resetFIFO();
   }
   currentState = nextState;
}

bool readIMU() {
   // Attempts to get IMU data and put into corresponding variables, controller only uses 
   //    pitch, roll, pitchRate, rollRate. 
   // If there is no data to get, returns false, otherwise return true.

   if (!mpuInterrupt && fifoCount < packetSize) {
    return false;
   }

   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();
   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
   }
   else if (mpuIntStatus & 0x02 && millis() > 5000) {

      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw = ypr[0];
      pitch = -1*ypr[1];
      roll =  ypr[2];

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      // Scaling for +/- 2000 degrees/s full scale range
      pitchRate = 1 * gy / 16.4 * M_PI / 180.0;
      rollRate = 1 * gx / 16.4  * M_PI / 180.0;
      return true;
   }
   return false;
}

void printData() {
   #ifdef PRINT_DATA
      Serial.print(x(0));
      Serial.print("\t");
      Serial.print(x(1));
      Serial.print("\t");
      Serial.print(x(2));
      Serial.print("\t");
      Serial.print(x(3));
      Serial.print("\t");
      Serial.print(x(4));
      Serial.print("\t");
      Serial.print(x(5));
      Serial.print("\t");
      Serial.print(x(cmdRollTorque));
      Serial.print("\t");
      Serial.print(x(cmdPitchTorque));
      Serial.print("\t");
      Serial.println(millis());
   #else
      Serial.write((byte *) &x(0), 4);
      Serial.write((byte *) &x(1), 4);
      Serial.write((byte *) &x(2), 4);
      Serial.write((byte *) &x(3), 4);
      Serial.write((byte *) &x(4), 4);
      Serial.write((byte *) &x(5), 4);
      Serial.write((byte *) &x(6), 4);
      Serial.write((byte *) &x(7), 4);
      Serial.write((byte *) &cmdRollTorque, 4);
      Serial.write((byte *) &cmdPitchTorque, 4);
      Serial.write((byte *) &measRollMotorTorque, 4);
      Serial.write((byte *) &measPitchMotorTorque, 4);
   #endif
}

void setup() {
   Wire.begin();
   
   Serial.begin(115200);
   mpu.initialize();
   devStatus = mpu.dmpInitialize();

   mpu.setXAccelOffset(-2570);
   mpu.setYAccelOffset(-1476);
   mpu.setZAccelOffset(790);
   mpu.setXGyroOffset(29);
   mpu.setYGyroOffset(-46);
   mpu.setZGyroOffset(-92);

   attachInterrupt(digitalPinToInterrupt(IMU_pin), dmpDataReady, RISING);


   if (devStatus == 0) {
      mpu.setDMPEnabled(true);
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
   } else {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
   }

   pinMode(LED_BUILTIN, OUTPUT);

   changeState(IDLE);
   lastPrintTime = micros();
   lastCommandTime = micros();
}

void loop() {
   if (!dmpReady){
      changeState(STOPPED);
   }

   if (currentState == STOPPED) {
      return;
   } else if (currentState == ACTIVE) {
      if (readIMU()) {
         if (fabs(pitch) < 0.3 && fabs(roll) < 0.3) {
            currentControlLoopTime = micros();

            controlLoopDeltaT = (float) (currentControlLoopTime - lastControlLoopTime) * 0.000001;

            // Gyro measurements super noisy, first order low pass
            filtRollRate = alpha * rollRate + (1 - alpha) * filtRollRate;
            filtPitchRate = alpha * pitchRate + (1 - alpha) * filtPitchRate;

            measRollMotorVel = bus.Get(0).Velocity();
            measPitchMotorVel = bus.Get(1).Velocity();

            measRollMotorTorque = -1*bus.Get(0).Torque();
            measPitchMotorTorque = -1*bus.Get(1).Torque();

            // Integrator on wheel velocity
            qRoll = constrain(qRoll + -1*measRollMotorVel * controlLoopDeltaT, -1*maxWindup, maxWindup);
            qPitch = constrain(qPitch + -1*measPitchMotorVel * controlLoopDeltaT, -1*maxWindup, maxWindup);

            // Update state vector
            x << pitch, filtPitchRate, measPitchMotorVel, qPitch, roll, filtRollRate, measRollMotorVel, qRoll;

            // Control law of u = r-Kx
            u = r-K*x;

            cmdRollTorque = constrain(u(0), -1*maxTorque, maxTorque);
            cmdPitchTorque = constrain(u(1), -1*maxTorque, maxTorque);

            cmdRollMotorCurrent = (int) (-1*cmdRollTorque*invTorqueConstant);
            cmdPitchMotorCurrent = (int) (-1*cmdPitchTorque*invTorqueConstant);

            if (fabs(measRollMotorVel) > 40 || fabs(measPitchMotorVel) > 40) {
               digitalWrite(LED_BUILTIN, true);
            } else {
               digitalWrite(LED_BUILTIN, false);
            }

            if (currentControlLoopTime - lastCommandTime > commandDelay) {
               bus.CommandTorques(cmdRollMotorCurrent, cmdPitchMotorCurrent, 0, 0, C610Subbus::kIDZeroToThree);
               lastCommandTime = currentControlLoopTime;
            }

            if (currentControlLoopTime - lastPrintTime > printDelay) {
               printData();
               lastPrintTime = currentControlLoopTime;
            }

            lastControlLoopTime = currentControlLoopTime;
         } else {
            changeState(IDLE);
         }
      }
   } else {
      if (readIMU()) {
         if (fabs(pitch) < 0.1 && fabs(roll) < 0.1) {
            changeState(ACTIVE);
         }
      }
   }
}