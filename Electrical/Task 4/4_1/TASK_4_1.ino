#define SAFETY_DISTANCE 10 

const int EN_A = 11;  // Motor A speed (PWM)
const int IN1  = 13;  // Motor A direction 1
const int IN2  = 12;  // Motor A direction 2

const int EN_B = 10;  // Motor B speed (PWM)
const int IN3  = 9;   // Motor B direction 1
const int IN4  = 8;   // Motor B direction 2

const int DIST_SENSOR = A0;  // Ultrasonic sensor pin
const int PIR_SENSOR = 2;
const int WEAPON = A3 ;

float distance;
long duration;
int motors_speed;

volatile int motion ;

void firing(){
  motion = digitalRead(PIR_SENSOR) ;
  if (motion)
    digitalWrite(WEAPON,HIGH);
  else
    digitalWrite(WEAPON,LOW);  
}

void setup() {
  Serial.begin(9600);
  // Set motor pins as OUTPUT
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(EN_B, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode (WEAPON , OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_SENSOR), firing, HIGH);
}

void loop() {

  digitalWrite(WEAPON, LOW) ;
  // === Distance Measurement ===
  pinMode(DIST_SENSOR, OUTPUT);
  digitalWrite(DIST_SENSOR, LOW);
  delayMicroseconds(2);
  digitalWrite(DIST_SENSOR, HIGH);
  delayMicroseconds(10);
  digitalWrite(DIST_SENSOR, LOW);

  pinMode(DIST_SENSOR, INPUT);
  duration = pulseIn(DIST_SENSOR, HIGH);
  distance = (duration * 0.03468) / 2.0;
  distance = constrain(distance,0,400) ;
  //Serial.print("DISTANCE IN CMMM IS");
  //Serial.println(distance);

  // === Set Direction: Forward ===
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);


  // === Set Speed ===
  motors_speed  = ((distance - SAFETY_DISTANCE) / 400.0) * 255 ; 
  if (motors_speed < 0 ) motors_speed = 0 ;
  analogWrite(EN_A,motors_speed);
  analogWrite(EN_B,motors_speed);

  digitalWrite(WEAPON, LOW) ;
}
