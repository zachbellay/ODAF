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

#include <math.h> // included for fabs()

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


void setup() 
{
  
  Serial.begin(115200);
  int i;
  for(i = 0; i < NUM_MOTORS; ++i){
    pinMode(DIRECTION_PINS[i].right_pin, OUTPUT);
    pinMode(DIRECTION_PINS[i].left_pin, OUTPUT);
  }  
}

void loop() 
{
  if(Serial.available())
  {
    // Get command and delimiter (expected input => 0,1,0 OR 1,0,0 etc...
    String command = Serial.readStringUntil('\n');
    int comma_one_index = command.indexOf(',');
    int comma_two_index = command.indexOf(',', comma_one_index + 1);

    // Convert each component to float
    float x_vec = (command.substring(0, comma_one_index)).toFloat();
    float y_vec = (command.substring(comma_one_index + 1, comma_two_index)).toFloat();
    float r_dir = (command.substring(comma_two_index+1)).toFloat();

    // Calculate appropriate velocity for each motor
    float f1 = (0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_dir);
    float f2 =(-0.5777 * x_vec) + (0.3333 * y_vec) + (0.3333 * r_dir);
    float f3 =                   (-0.6666 * y_vec) + (0.3333 * r_dir);

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
}
