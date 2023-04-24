/*
PID control implementation for the Polulu Balboa balancing robot.
Angle estimation is done using gyroscope and accelerometer data
combined using a complimentary filter
Author: Kade Perrotti
*/


#include <LSM6.h>
#include <Balboa32U4.h>

#define GRAVITY (9.80665F)
 
// Constants to do with control loop iteration time
#define DT (0.002) //2ms per control loop iteration
#define UPDATE_TIME_US (DT * 1e6) //2000us per control loop iteration

// Constants to do with gyro sensor data
#define GYRO_SCALE (0.035F) //scale for 1000dps (scale is 35, but divide by 1000 to get DPS)
#define GYRO_Y_OFFSET (4) //gyro seems to report ~-4DPS at rest, so compensate for that

// Constants to do with accelerometer sensor data
// if we multiply the accelerometer reading by 10, then it will report +-98 when robot falls over. 
// this makes it easy to stop the motors when robot falls over, because the angle estimation approaches
// the value of the accelerometer over time, due to the use of the complimentary filter in the angle estimation
#define ACCEL_STOPANGLE_HACK (10)
#define ACCEL_SCALE (.000488F * ACCEL_STOPANGLE_HACK) //scale for 16g, gives m/s^2 (m by 10)

// Robot's balance point is not when it is orthoganol to the floor. 
// The balance point occurs when the robot is leaned slightly towards the battery side. 
// Further negative causes robot to lean towards battery side, futher positive leans 
// more towards pcb side
#define ACCEL_Z_OFFSET (-1.0 * ACCEL_STOPANGLE_HACK) 

// This constant determines how much to weigh the accelerometer vs. gyroscope data when performing
// an angle estimation. Decrease to bias more towards accelerometer, increase to bias more 
// towards gyroscope. Range is 1.0 to 0, but we want to vastly prefer the gyro because 
// accel data is noisy. Also, no trig is used in this program, so the angle reported from
// the accelerometer is only semi-accurate when close to angle 0, because of the small angle approximation.
// Read more about this here: https://scolton-www.s3.amazonaws.com/docs/filter.pdf
// The small angle approximation "range" here is probably divided by 10 though in my implementation
// because of the ACCEL_STOPANGLE_HACK
#define COMP_FILT_ALPHA (.994)
 
#define STOP_BALANCE_ANGLE (55) //robot stops attempting to above below abs(this)

// PID coefficients. I'm using a large (relative) integral coeff because
// the relationship between the current angle and the motor response 
// is not linear, it's probably more exponential 
#define P (20.0)
#define I (0.0) 
#define D (.7) 

//Values from using linear regression
#define thetaIntercept (0.062072F)
#define thetaAngle (17.406281F)
#define thetaAngleRate (0.468099F)
#define thetaIntegral (0.288597F)

// (stolen from Polulu Balancer.ino example)
// DISTANCE_DIFF_RESPONSE determines the response to differences
// between the left and right motors, preventing undesired
// rotation due to differences in the motors and gearing.  Unlike
// DISTANCE_REPONSE, it should be negative: if the left motor is
// lagging, we need to increase its speed and decrease the speed
// of the right motor.  If this constant is too small, the robot
// will spin left and right as it rocks back and forth; if it is
// too large it will become unstable.
const int16_t DISTANCE_DIFF_RESPONSE = -50;

//angle estimations and PID response calculated
float angle = 0;
float lastAngle = 0; //used to combat integral windup when the robot has passed through it's balance point
float angleRate = 0;
float response; 

//summed with angle each control loop iteration
int32_t integral = 0;

//used to help motors turn at same rate (stolen from Polulu)
int32_t distanceLeft = 0;
int32_t distanceRight = 0;

//FOR TEST PURPOSES
int32_t accelZSum = 0;
float angleApprox = 0;

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;

char out[100];
char floatStr[10];
void setup() {
  //Serial.begin(115200);
  Serial1.begin(115200);
  
  // Initialize IMU.
  Wire.begin();
  Wire.setClock(400000UL);
  if (!imu.init())
  {
    while(true)
    {
      Serial1.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();  
  imu.writeReg(LSM6::CTRL2_G, 0b01101000); //416Hz 1000dps Gyroscope        
  imu.writeReg(LSM6::CTRL1_XL, 0b01100100); //16g 416Hz Accelerometer
  motors.allowTurbo(true);
  delay(2000);
  Serial1.println("Starting...");
  ledGreen(true);
  ledRed(false);
}

/**
Stolen from polulu
Determine how many ticks the motors
have moved since the last iteration,
for both motors
**/
void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
}

/**
Combine gyroscope and accelerometer data
into a Pitch angle estimation
**/
void updatePosition()
{
  //calculate the acceleration that the robot experiences in the Z direction (0 when robot orthoganol to floor)
  float accelZ = ACCEL_Z_OFFSET + ((imu.a.z * ACCEL_SCALE * GRAVITY)); //m/s^2

  //calculate rate of change of angle during the time since last control loop iteration ran (DT)
  angleRate = GYRO_Y_OFFSET + ((imu.g.y * GYRO_SCALE)); //DPS

  //angleRate (Degrees Per Second) multiplied by DT (Seconds), gives an estimate for change in angle
  //sum this with the previous angle to get the current angle estimation
  //weighted sum the above with a weighted estimate of angle from the accelerometer, accelZ 
  angle = (COMP_FILT_ALPHA * ((angleRate * DT) + angle) + ((1 - COMP_FILT_ALPHA) * accelZ));

  //For test purposes
  angleApprox = (COMP_FILT_ALPHA * ((imu.g.y * DT) + angleApprox) + ((1 - COMP_FILT_ALPHA) * imu.a.z));
}

/**
Update the integral term, 3 cases:
  robot has crossed angle 0
    divide integral by 2 (reduce windup)
  integral >= MAX_MOTOR_SPEED (+-400)
    cap integral at +- MAX_MOTOR_SPEED
  general case
    sum the angle with current integral
**/
void updateIntegral()
{
  //reset integral if angle has changed sign since last iteration
  if((angle > 2 && lastAngle < 0) || (angle < -2 && lastAngle > 0))
  {
    integral = integral - 1;
  }
  else if(abs(integral) > 400) //limit integral windup
  {
    if(integral >= 400)
    {
      integral = 400;
    }
    else
    {
      integral = -400;
    }
  }
  else
  {
    integral += angle;
  }
  lastAngle = angle; //save this iteration's angle for next iteration
}

/**
Calculate and set motor response using PID
**/
void calculateMotorResponse()
{
  //response = (P * angle) + (D * angleRate) + (I * integral);
  //response = thetaIntercept + ((thetaAngle * angle) + (thetaAngleRate * angleRate) + (thetaIntegral * integral));

  //response using angleApprox
  response = -125.5170 + (0.7454 * angleApprox) + (.0072 * imu.g.y);
  
  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  integrateEncoders();
  int16_t distanceDiff = distanceLeft - distanceRight;
  
  motors.setSpeeds(
  response + distanceDiff * DISTANCE_DIFF_RESPONSE / 100,
  response - distanceDiff * DISTANCE_DIFF_RESPONSE / 100
  );
  
}

void debugPrint()
{

  /*
  Send short approximation of angle and  gyro y and send via bluetooth
  dtostrf(angleApprox, 6, 2, out);
  strcat(out, ",");
  
  itoa(imu.g.y, out + strlen(out), 10);
  strcat(out, ",");
  
  dtostrf(response, 6, 2, out + strlen(out));
  */

  /*
  Get accelerometer z, gyroscope y, and response out via bluetooth.
  */

  /*itoa(imu.a.z, out, 10);
  strcat(out, ",");
  itoa(imu.a.x, out + strlen(out), 10);
  strcat(out, ",");
  itoa(imu.g.y, out + strlen(out), 10);
  strcat(out, ",");
  dtostrf(response, 6, 2, out + strlen(out));
  */
  
  
  //Get angle, angleRate, integral, response out via bluetooth
  /*dtostrf(angle, 6, 2, out);
  strcat(out, ",");
  
  dtostrf(angleRate, 6, 2, out + strlen(out));
  strcat(out, ",");
  
  itoa(integral, out + strlen(out), 10);
  strcat(out, ",");

  dtostrf(response, 6, 2, out + strlen(out));
  */
  
  Serial1.println("Approx angle");

  //sprintf(out, "%.2f,%.2f,%d,%.2f\n", angle, angleRate, integral, response);
  //Serial1.println(out);
  //Serial1.print("a:");
  //Serial1.print(angle);
  //Serial1.print(",ar:");
  //Serial1.print(angleRate);
  //Serial1.print("i:");
  //Serial1.print(integral);
  //Serial1.print(",r:");
  //Serial1.println(response);
  //Serial.print(",t:");
  //Serial.print(10);
  //Serial.print(",b:");
  //Serial.println(-10);
}

void loop() {

  //record start time of control loop iteration
  volatile int controlLoopStart = micros();
  imu.read(); //update accel and gyro sensor data
  
  updatePosition(); //update angle estimate
  updateIntegral(); //update the integral while combating windup

  if(abs(angle) < STOP_BALANCE_ANGLE) //robot should be able to balance
  {
    calculateMotorResponse();
  }
  else //robot has or will fall over. Shutdown motors
  {
    integral = 0;
    distanceLeft = 0;
    distanceRight = 0;
    response = 0;
    motors.setSpeeds(0,0);
  }
  
  //record end time of control loop iteration
  debugPrint();
  volatile int controlLoopEnd = micros();
  int32_t timeElapsed = controlLoopEnd - controlLoopStart; //in microseconds
  //Serial.println(timeElapsed);
  delayMicroseconds(UPDATE_TIME_US - timeElapsed); //delay for 2000us - the time it took for this iteration to run  
}
