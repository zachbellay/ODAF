#define DEBUG_MODE

#define MOTOR_1_RIGHT_PIN   48
#define MOTOR_1_LEFT_PIN    49
#define MOTOR_1_PWM_PIN     11
#define MOTOR_2_RIGHT_PIN   46
#define MOTOR_2_LEFT_PIN    47
#define MOTOR_2_PWM_PIN     12
#define MOTOR_3_RIGHT_PIN   44
#define MOTOR_3_LEFT_PIN    45
#define MOTOR_3_PWM_PIN     13

#define NUM_MOTORS 3

// included for fabs()
#include <math.h> 

// included for LSM 303 interface
#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// included for PID control
#include <PID_v1.h> 

typedef struct {
  uint8_t right_pin;
  uint8_t left_pin;
  uint8_t pwm_pin;
} motor_pins_t;

static const motor_pins_t DIRECTION_PINS[NUM_MOTORS] = {
    {MOTOR_1_RIGHT_PIN, MOTOR_1_LEFT_PIN, MOTOR_1_PWM_PIN},
    {MOTOR_2_RIGHT_PIN, MOTOR_2_LEFT_PIN, MOTOR_2_PWM_PIN},
    {MOTOR_3_RIGHT_PIN, MOTOR_3_LEFT_PIN, MOTOR_3_PWM_PIN}
};

// setup LSM303 interface to magnetometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Variables for moving average filter
const int numReadings = 1;
double readings[numReadings];    
int readIndex = 0;              
double total = 0;                  
double average = 0;                

// Globals for current directional state
double x_vec = 0; 
double y_vec = 0; 
double r_vec = 0;

// Variables for compass heading PID correction
const double Pi = 3.14159;
const int sample_period = 100; // in ms 
const double setpoint_alpha = 3.0; // how quickly rotation command should change r_setpoint
double r_setpoint, r_input, r_output;

// PID Coefficients
const int Kp = 70;
const int Ki = 10;
const int Kd = 10;

// Explicit PID Min/Max output
const int pid_min = -1024;
const int pid_max = 1023;

// Instantiating PID object
PID r_PID(&r_input, &r_output, &r_setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
{
  
  Serial.begin(2000000);

  // Check LSM 303 wiring
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  // set motor direction pins as outputs
  for(int i = 0; i < NUM_MOTORS; ++i){
    pinMode(DIRECTION_PINS[i].right_pin, OUTPUT);
    pinMode(DIRECTION_PINS[i].left_pin, OUTPUT);
  }  

  // initialize current direction as rotational setpoint
  sensors_event_t event; 
  mag.getEvent(&event);
  r_setpoint = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // initialize all the readings to first value (helps prevent initial oscillation)
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  // turn the PID on
  r_PID.SetMode(AUTOMATIC);
  r_PID.SetSampleTime(sample_period);
  r_PID.SetOutputLimits(pid_min, pid_max);
      
}

void loop() 
{
  if(Serial.available())
  {
    // Get command and delimiter (expected input => 0,1,0 OR 1,0,0 etc...
    String command = Serial.readStringUntil('\n');
    int comma_one_index = command.indexOf(',');
    int comma_two_index = command.indexOf(',', comma_one_index + 1);

    // Convert each component to double
    x_vec = (command.substring(0, comma_one_index)).toDouble();
    y_vec = (command.substring(comma_one_index + 1, comma_two_index)).toDouble();
    r_vec = (command.substring(comma_two_index+1)).toDouble();

    // Adjust global heading setpoint
    r_setpoint = r_setpoint + (r_vec * setpoint_alpha);

//    // Clamp r_setpoint between 0-360 degrees
//    if(r_setpoint > 360)
//      r_setpoint -= 360;
//    else if(r_setpoint < 0)
//      r_setpoint = 0;

  }
  
  // Read from LSM 303 magnetometer
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate heading by taking arctan of x and y vector and convert to degrees
  double heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

  if(heading < 0)
    heading += 360;

  // Calculate moving average for heading (helps filter out the occasional zero readings)
  total = total - readings[readIndex]; // subtract the last reading
  readings[readIndex] = heading;  // read from the sensor
  total = total + readings[readIndex]; // add the reading to the total
  readIndex = readIndex + 1;  // advance to the next position in the array

  // If at end of array, wrap around
  if (readIndex >= numReadings)
    readIndex = 0;  

  // Filtered heading reading
  r_input = total / numReadings;

  // Update PID
  r_PID.Compute();

  double r_dir = r_output / pid_max;// * 0.6666);

#ifdef DEBUG_MODE
  Serial.print("Setpoint: ");
  Serial.print(r_setpoint);
  Serial.print(", Filtered Heading: ");
  Serial.print(r_input);
  Serial.print(", PID Output: ");
  Serial.print(r_output);
  Serial.print(", Error: ");
  Serial.print(r_setpoint-r_input);
  Serial.print(", Squished r_dir: ");
  Serial.println(r_dir);
#endif

  // Calculate appropriate velocity for each motor
  double f1 = (0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_dir);
  double f2 =(-0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_dir);
  double f3 =                   (-0.6666 * y_vec) + (0.3333 * r_dir);

  // Determine direction of motor
  bool f1_dir = (f1 < 0) ? false : true;
  bool f2_dir = (f2 < 0) ? false : true;
  bool f3_dir = (f3 < 0) ? false : true;

  // Get magnitude of velocity
  f1=fabs(f1);
  f2=fabs(f2);
  f3=fabs(f3);

  // Set direction of each motor spinning
  digitalWrite(DIRECTION_PINS[0].right_pin, (f1_dir ? HIGH : LOW)); 
  digitalWrite(DIRECTION_PINS[0].left_pin, (f1_dir ? LOW : HIGH));

  digitalWrite(DIRECTION_PINS[1].right_pin, (f2_dir ? HIGH : LOW)); 
  digitalWrite(DIRECTION_PINS[1].left_pin, (f2_dir ? LOW : HIGH));

  digitalWrite(DIRECTION_PINS[2].right_pin, (f3_dir ? HIGH : LOW)); 
  digitalWrite(DIRECTION_PINS[2].left_pin, (f3_dir ? LOW : HIGH));
  
  analogWrite(DIRECTION_PINS[0].pwm_pin, f1*255);
  analogWrite(DIRECTION_PINS[1].pwm_pin, f2*255);
  analogWrite(DIRECTION_PINS[2].pwm_pin, f3*255);

}
