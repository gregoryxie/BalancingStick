#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <C610Bus.h>
#include <BasicLinearAlgebra.h>
#include "Wire.h"

// #define PRINT_DATA

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

long last_print_time;
long last_command_time;

const int print_delay = 20000;
const int command_delay = 10000;

int16_t ax, ay, az, gx, gy, gz;
float yaw, pitch, roll, roll_rate, pitch_rate;
float meas_pitch_motor_vel, meas_roll_motor_vel, meas_pitch_motor_torque, meas_roll_motor_torque;

float filt_roll = 0.0, filt_pitch = 0.0, filt_roll_rate = 0.0, filt_pitch_rate = 0.0;
float alpha = 0.25, alpha2 = 0.25;

using namespace BLA;

Matrix<2,6> K = {0, 0, 0, 19.55, 2.7843, 0.0375, 
                 19.55, 2.7843, 0.0375, 0, 0, 0};

Matrix<6,1> x;
Matrix<2,1> u;
Matrix<2,1> r = {0, 0};

const float inv_torque_constant = 5555.55; // 1/0.00018 mA/Nm
int cmd_pitch_motor_current, cmd_roll_motor_current;
float cmd_pitch_torque, cmd_roll_torque;
const float max_torque = 1.5;

volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
   Wire.begin();
   
   Serial.begin(115200);
   mpu.initialize();
   devStatus = mpu.dmpInitialize();

   mpu.setXAccelOffset(-2566);
   mpu.setYAccelOffset(-1537);
   mpu.setZAccelOffset(950);
   mpu.setXGyroOffset(15);
   mpu.setYGyroOffset(-16);
   mpu.setZGyroOffset(57);

   // mpu.setXAccelOffset(-2316);
   // mpu.setYAccelOffset(-1313);
   // mpu.setZAccelOffset(1014);
   // mpu.setXGyroOffset(24);
   // mpu.setYGyroOffset(-28);
   // mpu.setZGyroOffset(110);

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

   last_print_time = micros();
   last_command_time = micros();
}

void loop() {
   if (!dmpReady){
      return;
   }

   while (!mpuInterrupt && fifoCount < packetSize) {
    ;
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

      filt_roll = alpha * roll + (1 - alpha) * filt_roll;
      filt_pitch = alpha * pitch + (1 - alpha) * filt_pitch;

      if (fabs(pitch) < 0.3 && fabs(roll) < 0.3) {
         mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
         // Scaling for +/- 2000 degrees/s full scale range
         pitch_rate = 1 * gy / 16.4 * M_PI / 180.0;
         roll_rate = 1 * gx / 16.4  * M_PI / 180.0;

         filt_roll_rate = alpha2 * roll_rate + (1 - alpha2) * filt_roll_rate;
         filt_pitch_rate = alpha2 * pitch_rate + (1 - alpha2) * filt_pitch_rate;

         meas_roll_motor_vel = bus.Get(0).Velocity()/50.0;
         meas_pitch_motor_vel = bus.Get(1).Velocity()/50.0;

         meas_roll_motor_torque = -1*bus.Get(0).Torque();
         meas_pitch_motor_torque = -1*bus.Get(1).Torque();

         // Update state vector
         x << filt_pitch, filt_pitch_rate, meas_pitch_motor_vel, filt_roll, filt_roll_rate, meas_roll_motor_vel;

         // Control law of u = r-Kx
         u = r-K*x;

         cmd_roll_torque = constrain(u(0), -1*max_torque, max_torque);
         cmd_pitch_torque = constrain(u(1), -1*max_torque, max_torque);

         cmd_roll_motor_current = (int) (-1*cmd_roll_torque*inv_torque_constant);
         cmd_pitch_motor_current = (int) (-1*cmd_pitch_torque*inv_torque_constant);

         if (fabs(meas_roll_motor_vel) > 40 || fabs(meas_pitch_motor_vel) > 40) {
            digitalWrite(LED_BUILTIN, true);
         } else {
            digitalWrite(LED_BUILTIN, false);
         }

         if (micros() - last_command_time > command_delay) {
            //cmd_roll_motor_current = max_current*sin(micros()/1000000.0);
            bus.CommandTorques(cmd_roll_motor_current, cmd_pitch_motor_current, 0, 0, C610Subbus::kIDZeroToThree);
            last_command_time = micros();
         }

         if (micros() - last_print_time > print_delay) {
            #ifdef PRINT_DATA
               Serial.print(pitch);
               Serial.print("\t");
               Serial.print(pitch_rate);
               Serial.print("\t");
               Serial.print(pitch_motor_vel);
               Serial.print("\t");
               Serial.print(roll);
               Serial.print("\t");
               Serial.print(roll_rate);
               Serial.print("\t");
               Serial.print(roll_motor_vel);
               Serial.print("\t");
               Serial.print(roll_motor_current);
               Serial.print("\t");
               Serial.print(pitch_motor_current);
               Serial.print("\t");
               Serial.println(millis());
            #else
               Serial.write((byte *) &x(0), 4);
               Serial.write((byte *) &x(1), 4);
               Serial.write((byte *) &x(2), 4);
               Serial.write((byte *) &x(3), 4);
               Serial.write((byte *) &x(4), 4);
               Serial.write((byte *) &x(5), 4);
               Serial.write((byte *) &cmd_roll_torque, 4);
               Serial.write((byte *) &cmd_pitch_torque, 4);
               Serial.write((byte *) &meas_roll_motor_torque, 4);
               Serial.write((byte *) &meas_pitch_motor_torque, 4);
            #endif
            last_print_time = micros();
         }
      }
      else {
         bus.CommandTorques(0, 0, 0, 0, C610Subbus::kIDZeroToThree);
         digitalWrite(LED_BUILTIN, true);
         // while (true) {

         // }
      }
   }
}