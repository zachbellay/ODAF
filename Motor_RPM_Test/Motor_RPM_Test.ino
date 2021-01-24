#include <Encoder.h>
#include <PID_v1.h>
#include <math.h> 

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

Encoder M1_enc(M1_HALL_A, M1_HALL_B);
Encoder M2_enc(M2_HALL_A, M2_HALL_B);
Encoder M3_enc(M3_HALL_A, M3_HALL_B);

elapsedMillis since_M1_rpm_check;
elapsedMillis since_M2_rpm_check;
elapsedMillis since_M3_rpm_check;

elapsedMillis graphing_check;

double Setpoint=0;

double M1_input=0, M1_output=0;
double M2_input=0, M2_output=0;
double M3_input=0, M3_output=0;

double Kp=1.5, Ki=0.5, Kd=0.00;

String _setpoint_str;

PID M1_RPM_PID(&M1_input, &M1_output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID M2_RPM_PID(&M2_input, &M2_output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID M3_RPM_PID(&M3_input, &M3_output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);  
  
  pinMode(M1_IN_B, OUTPUT);
  pinMode(M2_IN_B, OUTPUT);
  pinMode(M3_IN_B, OUTPUT);
  
  M1_RPM_PID.SetOutputLimits(-255, 255);
  M1_RPM_PID.SetMode(AUTOMATIC);

  M2_RPM_PID.SetOutputLimits(-255, 255);
  M2_RPM_PID.SetMode(AUTOMATIC);

  M3_RPM_PID.SetOutputLimits(-255, 255);
  M3_RPM_PID.SetMode(AUTOMATIC);
  
}

//0.8,7,0,0
//p,i,d,setpoint

void loop() {

  if(graphing_check >= 50){
    graphing_check -= 50;
//    Serial.print(M1_input);
//    Serial.print(" ");
//    Serial.print(M1_output);
//    Serial.print(" ");
//    Serial.print(M2_input);
//    Serial.print(" ");
//    Serial.print(M2_output);
//    Serial.print(" ");
//    Serial.print(M3_input);
//    Serial.print(" ");
//    Serial.print(M3_output);
//    Serial.print(" ");
//    Serial.println(Setpoint);
    Serial.println(M1_enc.read());
  }

  if(since_M1_rpm_check >= 50){
    since_M1_rpm_check -= 50;

    long M1_delta_x = M1_enc.read();
    M1_input = M1_delta_x * ((60*20)/2803.2);
    M1_RPM_PID.Compute();

    if(M1_output > 0)
      digitalWrite(M1_IN_B, HIGH);
    else
      digitalWrite(M1_IN_B, LOW);

    if(abs(Setpoint) <= 1)
      M1_output=0;
    
    analogWrite(M1_IN_A, fabs(M1_output));

    M1_enc.write(0);
      
  }

  if(since_M2_rpm_check >= 50){
    since_M2_rpm_check -= 50;

    long M2_delta_x = M2_enc.read();
    M2_input = M2_delta_x * ((60*20)/2803.2);
    M2_RPM_PID.Compute();

    if(M2_output > 0)
      digitalWrite(M2_IN_B, HIGH);
    else
      digitalWrite(M2_IN_B, LOW);

    if(abs(Setpoint) <= 1)
        M2_output=0;
    
    analogWrite(M2_IN_A, fabs(M2_output));

    M2_enc.write(0);
      
  }

  if(since_M3_rpm_check >= 50){
    since_M3_rpm_check -= 50;

    long M3_delta_x = M3_enc.read();
    M3_input = M3_delta_x * ((60*20)/2803.2);
    M3_RPM_PID.Compute();

    if(M3_output > 0)
      digitalWrite(M3_IN_B, HIGH);
    else
      digitalWrite(M3_IN_B, LOW);

    if(abs(Setpoint) <= 1)
        M3_output=0;
    
    analogWrite(M3_IN_A, fabs(M3_output));

    M3_enc.write(0);
      
  }

  if(Serial.available()){

    String command = Serial.readStringUntil('\n');
    int comma_one_index = command.indexOf(','); 
    int comma_two_index = command.indexOf(',', comma_one_index + 1);
    int comma_three_index = command.indexOf(',', comma_two_index + 1);

    Kp = (double)(command.substring(0, comma_one_index)).toFloat();
    Ki = (double)(command.substring(comma_one_index + 1, comma_two_index)).toFloat();
    Kd = (double)(command.substring(comma_two_index+1)).toFloat();
    M1_RPM_PID.SetTunings(Kp, Ki, Kd);
    M2_RPM_PID.SetTunings(Kp, Ki, Kd);
    M3_RPM_PID.SetTunings(Kp, Ki, Kd);
    Setpoint = (command.substring(comma_three_index+1)).toFloat();
    _setpoint_str = String(Setpoint);
        
  }



}
