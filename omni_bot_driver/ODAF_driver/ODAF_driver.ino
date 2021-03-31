#include <Encoder.h>
//#include <NewPing.h>
#include "NewPing.h"
#include <PID_v1.h>
#include <math.h>

#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>
#include <Filters/MedianFilter.hpp>

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_LSM303_U.h"

#define ECHO_1 17
#define ECHO_2 16
#define ECHO_3 15
#define ECHO_4 14
#define ECHO_5 13
#define ECHO_6 39
#define ECHO_7 36
#define ECHO_8 35
#define ECHO_9 34

#define TRIG_1 24
#define TRIG_2 25
#define TRIG_3 26
#define TRIG_4 27
#define TRIG_5 28
#define TRIG_6 29
#define TRIG_7 30
#define TRIG_8 31
#define TRIG_9 32

#define M1_HALL_B 23
#define M1_HALL_A 22
#define M2_HALL_B 21
#define M2_HALL_A 20
#define M3_HALL_B 19
#define M3_HALL_A 18

#define M3_IN_A 6
#define M3_IN_B 7
#define M2_IN_A 4
#define M2_IN_B 5
#define M1_IN_A 2
#define M1_IN_B 3

#define MAX_ULTRASONIC_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define NUM_ULTRASONICS 9
#define NUM_MOTORS 3

#define ENCODER_CHECK_FREQ 50 //hz
#define ENCODER_CHECK_PERIOD ((1.0/ENCODER_CHECK_FREQ) * 1000) //ms

#define ULTRASONIC_CHECK_FREQ 30 // hz
#define ULTRASONIC_CHECK_PERIOD ((1.0/ENCODER_CHECK_FREQ) * 1000) //ms
#define ULTRASONIC_FILTER_ORDER 15
#define NUM_ULTRASONIC_SENSORS 9

typedef struct {
  uint8_t pwm_pin;
  uint8_t dir_pin;
} motor_pins_t;

static const motor_pins_t MOTOR_PINS[NUM_MOTORS] = {
  {M1_IN_A, M1_IN_B},
  {M2_IN_A, M2_IN_B},
  {M3_IN_A, M3_IN_B}
};

// Motor PID coefficients
double Kp = 0.8, Ki = 7, Kd = 0;

double M_input[NUM_MOTORS];
double M_output[NUM_MOTORS];
double M_setpoint[NUM_MOTORS];

static Encoder M_encoders[NUM_MOTORS] = {
  Encoder(M1_HALL_A, M1_HALL_B),
  Encoder(M2_HALL_A, M2_HALL_B),
  Encoder(M3_HALL_A, M3_HALL_B)
};

static PID M_RPM_PID[NUM_MOTORS] = {
  PID(&M_input[0], &M_output[0], &M_setpoint[0], Kp, Ki, Kd, DIRECT),
  PID(&M_input[1], &M_output[1], &M_setpoint[1], Kp, Ki, Kd, DIRECT),
  PID(&M_input[2], &M_output[2], &M_setpoint[2], Kp, Ki, Kd, DIRECT),
};

static NewPing ultrasonics[NUM_ULTRASONICS] = {
  NewPing(TRIG_1, ECHO_1, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_2, ECHO_2, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_3, ECHO_3, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_4, ECHO_4, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_5, ECHO_5, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_6, ECHO_6, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_7, ECHO_7, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_8, ECHO_8, MAX_ULTRASONIC_DISTANCE),
  NewPing(TRIG_9, ECHO_9, MAX_ULTRASONIC_DISTANCE),
};


typedef MedianFilter<ULTRASONIC_FILTER_ORDER, int16_t> median_filter;
using median_filter_t = decltype(median_filter());

static median_filter_t ULTRASONIC_FILTER[NUM_ULTRASONIC_SENSORS] = {median_filter()};

static Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(42069);
const double Pi = 3.14159;

elapsedMillis M_RPM_CHECK[3];

elapsedMillis graphing_check;
elapsedMillis ultrasonic_check;

// Globals for current directional state
float x_vec = 0;
float y_vec = 0;
float r_vec = 0;

void setup() {
  Serial.begin(2000000);

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    pinMode(MOTOR_PINS[i].dir_pin, OUTPUT);
    M_RPM_PID[i].SetOutputLimits(-255, 255);
    M_RPM_PID[i].SetMode(AUTOMATIC);
  }

  if (!mag.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1) {
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);
    }
  }

}

void loop()
{

  if (Serial.available()) {

    String command = Serial.readStringUntil('\n');

    if (command == "HEADING") {

      sensors_event_t event;
      mag.getEvent(&event);

      // Calculate heading by taking arctan of x and y vector and convert to degrees
      double heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

      if (heading < 0)
        heading += 360;

      Serial.println(heading);
    } else if (command == "ULTRASONIC") {
      for (int i = 0; i < NUM_ULTRASONICS; ++i) {
        Serial.print(ULTRASONIC_FILTER[i].getValue());
        Serial.print(",");
      }
      Serial.println();
    } else {
      // Get command and delimiter (expected input => 0,1,0 OR 1,0,0 etc...
      int comma_one_index = command.indexOf(',');
      int comma_two_index = command.indexOf(',', comma_one_index + 1);

      // Convert each component to double
      x_vec = (command.substring(0, comma_one_index)).toFloat();
      y_vec = (command.substring(comma_one_index + 1, comma_two_index)).toFloat();
      r_vec = (command.substring(comma_two_index + 1)).toFloat();

      // Calculate appropriate velocity for each motor
      M_setpoint[0] =  ((0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_vec)) * 240; // 240 is max RPM @ 12V
      M_setpoint[1] = ((-0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_vec)) * 240;
      M_setpoint[2] = (                   (-0.6666 * y_vec) + (0.3333 * r_vec)) * 240;

      Serial.println("OK");
    }
  }

  //  Run motor control loop
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (M_RPM_CHECK[i] >= ENCODER_CHECK_PERIOD) {

      M_RPM_CHECK[i] -= ENCODER_CHECK_PERIOD;

      long motor_delta = M_encoders[i].read();

      // 43.8:1 gear ratio * 64 cycles/revolution = 2803.2 cycles/full wheel revolution
      M_input[i] = motor_delta * ((60 * ENCODER_CHECK_FREQ) / 2803.2);
      M_RPM_PID[i].Compute();

      if (M_output[i] > 0)
        digitalWrite(MOTOR_PINS[i].dir_pin, HIGH);
      else
        digitalWrite(MOTOR_PINS[i].dir_pin, LOW);

      // Suppress movement if the motion is small
      if (abs(M_setpoint[i]) <= 1)
        M_output[i] = 0;

      analogWrite(MOTOR_PINS[i].pwm_pin, fabs(M_output[i]));

      M_encoders[i].write(0);

    }
  }

  // Sample and filter ultrasonic sensor
  if (ultrasonic_check >= ULTRASONIC_CHECK_PERIOD) {
    ultrasonic_check -= ULTRASONIC_CHECK_PERIOD;
    for (int i = 0; i < NUM_ULTRASONICS; ++i)
      ULTRASONIC_FILTER[i](ultrasonics[i].ping_cm());
  }
  
}
