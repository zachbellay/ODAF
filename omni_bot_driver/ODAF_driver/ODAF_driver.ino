#include <Encoder.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <math.h>

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

#define ENCODER_CHECK_FREQ 20
#define ENCODER_CHECK_PERIOD ((1.0/ENCODER_CHECK_FREQ) * 1000)

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

//static NewPing ultrasonics[NUM_ULTRASONICS]={
//  NewPing(TRIG_1, ECHO_1, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_2, ECHO_2, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_3, ECHO_3, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_4, ECHO_4, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_5, ECHO_5, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_6, ECHO_6, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_7, ECHO_7, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_8, ECHO_8, MAX_ULTRASONIC_DISTANCE),
//  NewPing(TRIG_9, ECHO_9, MAX_ULTRASONIC_DISTANCE),
//};


elapsedMillis M_RPM_CHECK[3];

elapsedMillis graphing_check;

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

}

void loop()
{

  if (Serial.available()) {
    // Get command and delimiter (expected input => 0,1,0 OR 1,0,0 etc...
    String command = Serial.readStringUntil('\n');
    int comma_one_index = command.indexOf(',');
    int comma_two_index = command.indexOf(',', comma_one_index + 1);

    // Convert each component to double
    x_vec = (command.substring(0, comma_one_index)).toFloat();
    y_vec = (command.substring(comma_one_index + 1, comma_two_index)).toFloat();
    r_vec = (command.substring(comma_two_index + 1)).toFloat();

    // Calculate appropriate velocity for each motor
    //    M_setpoint[0] =  (0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_vec) * 232; //232 is max RPM for M1
    M_setpoint[0] =  ((0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_vec)) * 240; // 240 is max RPM @ 12V
    M_setpoint[1] = ((-0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_vec)) * 240;
    M_setpoint[2] = (                   (-0.6666 * y_vec) + (0.3333 * r_vec)) * 240;

  }

  //  for(uint8_t i = 0; i < NUM_MOTORS; ++i){
  //    if(M_RPM_CHECK[i] >= ENCODER_CHECK_PERIOD){
  //
  //      M_RPM_CHECK[i] -= ENCODER_CHECK_PERIOD;
  //
  //      long motor_delta = M_encoders[i].read();
  //
  //      M_input[i] = motor_delta * ((60 * ENCODER_CHECK_FREQ) / 2803.2);
  //      M_RPM_PID[i].Compute();
  //
  //      if(M_output[i] > 0)
  //        digitalWrite(MOTOR_PINS[i].dir_pin, HIGH);
  //      else
  //        digitalWrite(MOTOR_PINS[i].dir_pin, LOW);
  //
  //      //digitalWrite(MOTOR_PINS[i].dir_pin, (M_output[i] > 0 ? HIGH : LOW));
  //
  //      if(abs(M_setpoint[i]) <= 1)
  //        M_output[i] = 0;
  //
  //      analogWrite(MOTOR_PINS[i].pwm_pin, fabs(M_output[i]));
  //
  //      M_encoders[i].write(0);
  //
  //    }
  //  }




  if (graphing_check >= 50) {
    graphing_check -= 50;
    Serial.print(M_input[0]);
    Serial.print(" ");
    Serial.print(M_output[0]);
    Serial.print(" ");
    Serial.print(M_setpoint[0]);
    Serial.print(" ");
    Serial.print(M_input[1]);
    Serial.print(" ");
    Serial.print(M_output[1]);
    Serial.print(" ");
    Serial.print(M_setpoint[1]);
    Serial.print(" ");
    Serial.print(M_input[2]);
    Serial.print(" ");
    Serial.print(M_output[2]);
    Serial.print(" ");
    Serial.print(M_setpoint[2]);
    Serial.println(" ");

  }
  //  for(int i = 0; i < NUM_ULTRASONICS; ++i){
  //      Serial.print(ultrasonics[i].ping_cm());
  //      Serial.print(",");
  //    }
  //    Serial.println();


  //  // Determine direction of motor
  //  bool f1_dir = (f1 < 0) ? false : true;
  //  bool f2_dir = (f2 < 0) ? false : true;
  //  bool f3_dir = (f3 < 0) ? false : true;
  //
  //  // Get magnitude of velocity
  //  f1=fabs(f1);
  //  f2=fabs(f2);
  //  f3=fabs(f3);
  //
  //  // Set direction of each motor spinning
  //  digitalWrite(MOTOR_PINS[0].dir_pin, (f1_dir ? HIGH : LOW));
  //  digitalWrite(MOTOR_PINS[1].dir_pin, (f2_dir ? HIGH : LOW));
  //  digitalWrite(MOTOR_PINS[2].dir_pin, (f3_dir ? HIGH : LOW));
  //
  //  analogWrite(MOTOR_PINS[0].pwm_pin, f1*255);
  //  analogWrite(MOTOR_PINS[1].pwm_pin, f2*255);
  //  analogWrite(MOTOR_PINS[2].pwm_pin, f3*255);

}
