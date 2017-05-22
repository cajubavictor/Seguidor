//sololearn

#define MOTOR_E1    10
#define MOTOR_E2    9
#define MOTOR_D1    6
#define MOTOR_D2    5
void setup(){
  pinMode(MOTOR_E1, OUTPUT);
  pinMode(MOTOR_E2, OUTPUT);
  pinMode(MOTOR_D1, OUTPUT);
  pinMode(MOTOR_D2, OUTPUT);
}
void loop(){
  analogWrite(MOTOR_E1, 200);
  digitalWrite(MOTOR_E2, LOW);
  digitalWrite(MOTOR_D1, LOW);
  analogWrite(MOTOR_D2, 200);
}

