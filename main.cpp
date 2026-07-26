#include <Arduino.h>
#include <Wire.h>
#include <TimerOne.h>

#define M_PI 3.14159265358979323846
#define MPU_ADDR 0x68  // address of the mpu 


// Function declarations
void readmpu();
float get_angle();
void drive_motor(float pwm);


int ENA = 5, IN1 = 7, IN2 = 8;          // pins for motor A
int ENB = 6, IN3 = 12, IN4 = 11;       // pins for motor B
int basePWM = 60;        // for over coming the friction
float gainB = 1.45;      // one of the motor is much slower than the other

// mpu reading
int16_t ax, ay, az;        
int16_t gx, gy, gz;

float acc_offset_x = 2000.73;
float acc_offset_y = 1000.65;
float acc_offset_z = 1600.01; 
float gyro_offset_x = -274.52;
float gyro_offset_y = 569.30;
float gyro_offset_z = -118.53;

// angles
float angle = 0.0;
float ang;

// filtered coeficciont
const float alpha1 = 0.90;      // filter coefficciont for the complementary filter
const float alpha2 = 0.85;      // filter coefficciont for the dirivative low pass filter
const float Ts = 0.005;         // sample time

// pid gains
float kp = 20.0;
float ki = 1.0; 
float kd = 0.1;

// pid variables
float last_error = 0;
float integral = 0;
float derivative = 0, filter_derivative = 0;
float output = 0;
float Imax = 150.0;

// ISR (Interrupt Service Routine) for achieving the sample time
volatile bool timerFlag = false;
void timerIsr() {
  timerFlag = true;
}

void setup() {
  // put your setup code here, to run once:
  delay(5000);
  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // timer 
  Timer1.initialize(Ts * 1000000); 
  Timer1.attachInterrupt(timerIsr); 

  // MPU initialization
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(); // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(); // Accel ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(); // Gyro ±250°/s


  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!(timerFlag)) return;
  timerFlag = false;

  ang = get_angle();

  if (abs(angle) > 45.0) {
  integral = 0;          // reset integrator (anti-windup)
  filter_derivative = 0;
  last_error = 0;

  readmpu();
  
  drive_motor(0);
  return;                // EXIT this control cycle
}

  float setpoint = -2.5;
  float error = setpoint - ang;

  integral += error * Ts;
  integral = constrain(integral, -Imax, Imax);

  
  derivative = (error - last_error) / Ts;
  filter_derivative = alpha2 * filter_derivative + (1 - alpha2) * derivative; // low-pass filter

  last_error = error;
  output = kp * error + ki * integral + kd * filter_derivative; // PID control
  output = constrain(output, -Imax, Imax);

  drive_motor(output);
}

void readmpu(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

float get_angle(){
  readmpu();
  float acc_angle, gyro_rate;
  float ax_g, ay_g, az_g;
  ax_g = (ax - acc_offset_x) / 16384.0;
  ay_g = (ay - acc_offset_y) / 16384.0;
  az_g = (az - acc_offset_z) / 16384.0;

  float gx_dps, gy_dps, gz_dps;

  gx_dps = (gx - gyro_offset_x) / 131.0;
  gy_dps = (gy - gyro_offset_y) / 131.0;
  gz_dps = (gz - gyro_offset_z) / 131.0;

  //if (abs(gy_dps) < 0.4) gy_dps = 0.0; // gyro dead zone

  acc_angle = atan2(ay_g, az_g) * 180.0 / PI;
  gyro_rate = gx_dps; 
  angle = alpha1 * (angle + gyro_rate * Ts) + (1.0 - alpha1) * acc_angle;
  return angle;
}

void drive_motor(float pwm){
  pwm = constrain(pwm, -Imax, Imax);

  if ((pwm > 0 && pwm < basePWM)) pwm = basePWM;
  else if (pwm < 0 && pwm > -basePWM) pwm = -basePWM;

  if(pwm > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW); 
  } else if (pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    pwm = -pwm;
  }else {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    return;
  }

  float pwmA = pwm;
  float pwmB = pwm * gainB;

  pwmA = constrain(pwmA, 0, 255);
  pwmB = constrain(pwmB, 0, 255);

  analogWrite(ENA, pwmA);
  analogWrite(ENB, pwmB);
}



