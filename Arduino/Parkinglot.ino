#include <Servo.h>

// --- 핀 ---
#define OUTSIDE_ULTRASONIC_TRIG 2
#define OUTSIDE_ULTRASONIC_ECHO 3
#define SERVO_PIN 5
#define RGB_RED_PIN 10
#define RGB_GREEN_PIN 9
// #define RGB_BLUE_PIN 10 (필요 시 사용)
#define PIEZO_PIN 7
#define INSIDE_ULTRASONIC_TRIG 13
#define INSIDE_ULTRASONIC_ECHO 11
#define MOTOR_PIN 6
#define GAS_PIN A0

// --- 상수 ---
#define OUTSIDE_DISTANCE_THRESHOLD 30
#define INSIDE_DISTANCE_THRESHOLD 30
#define GATE_DELAY (2 * 1000)
#define GAS_THRESHOLD 250

// --- 전역 변수 ---
struct SystemState {
  bool is_on_inside_sensor = false;
  bool is_on_outside_sensor = false;
  bool is_parking = true;
  bool is_on_motor = false;
  bool is_gas_detected = false;
  bool is_gate_open = false;  // 게이트 열림 여부 추가
};

SystemState ss;
Servo servo;

// --- 함수 정의 ---
long get_distance(int trig, int echo);
int get_gas();
void set_led_color(int red, int green, int blue);
void set_led(char color);
void set_motor(int motor_state);
void set_gate(char gate_state);
void state();
void trigger_alarm(int duration = 10);
void state_outside_sensor();
void state_inside_sensor();
void state_led();
void state_gate();
void state_alarm();
void state_motor();
void state_gas();
void init_pin();
void init_setup();

long get_distance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

int get_gas() {
  int gas = analogRead(GAS_PIN);
  return gas;
}

void set_led_color(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  // analogWrite(RGB_BLUE_PIN, blue);
}

void set_led(char color) {
  switch (color) {
    case 'R':
      set_led_color(255, 0, 0);
      break;
    case 'G':
      set_led_color(0, 255, 0);
      break;
    case 'B':
      set_led_color(0, 0, 255);
      break;
    default:
      set_led_color(0, 0, 0);
      break;
  }
}

void set_motor(int motor_state) {
  switch (motor_state) {
    case 0:
      analogWrite(MOTOR_PIN, 0);
      break;
    case 1:
      analogWrite(MOTOR_PIN, 63);
      break;
    case 2:
      analogWrite(MOTOR_PIN, 127);
      break;
    case 3:
      analogWrite(MOTOR_PIN, 191);
      break;
    case 4:
      analogWrite(MOTOR_PIN, 255);
      break;
    default:
      analogWrite(MOTOR_PIN, 0);
      break;
  }
}

void set_gate(char gate_state) {
  switch (gate_state) {
    case 'O':
      for (int i = 0; i <= 80; i++) {
        servo.write(i);
        delay(20);
      }
      ss.is_gate_open = true;
      break;
    case 'C':
      for (int i = 80; i >= 0; i--) {
        servo.write(i);
        delay(20);
      }
      ss.is_gate_open = false;
      break;
  }
}

void trigger_alarm(int duration) {
  tone(PIEZO_PIN, 1000);
  delay(duration * 1000);
  noTone(PIEZO_PIN);
}

void state_inside_sensor() {
  long inside_distance = get_distance(INSIDE_ULTRASONIC_TRIG, INSIDE_ULTRASONIC_ECHO);
  ss.is_on_inside_sensor = (inside_distance <= INSIDE_DISTANCE_THRESHOLD);
}

void state_outside_sensor() {
  long outside_distance = get_distance(OUTSIDE_ULTRASONIC_TRIG, OUTSIDE_ULTRASONIC_ECHO);
  ss.is_on_outside_sensor = (outside_distance <= OUTSIDE_DISTANCE_THRESHOLD);
}

void state_led() {
  // 주차 가능/불가능에 따라 LED 색상 지정 (논리명과 주석 확인 필요)
  if (ss.is_parking)
    set_led('R');
  else
    set_led('G');
}

void state_gate() {
  // 예시: 바깥 센서 감지 시 게이트 열고, 그렇지 않으면 닫기
  if (ss.is_on_outside_sensor && !ss.is_on_inside_sensor) {
    set_gate('O');
    ss.is_parking = false; // 주차 불가 상태 등으로 설정 (원하는 논리에 맞게 수정)
  } else {
    set_gate('C');
    ss.is_parking = true;
  }
}

void state_alarm() {
  if (ss.is_gate_open) {
    trigger_alarm(3);
  }
}

void state_motor() {
  if (ss.is_gas_detected)
    set_motor(4);
  else
    set_motor(1);
}

void state_gas() {
  int gas = get_gas();
  Serial.print("Gas data : ");
  Serial.println(gas);
  ss.is_gas_detected = (gas > GAS_THRESHOLD);
}

void state() {
  state_outside_sensor();
  state_inside_sensor();
  state_led();
  state_gate();
  state_alarm();
  state_motor();
  state_gas();
}

void init_pin() {
  pinMode(OUTSIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(OUTSIDE_ULTRASONIC_ECHO, INPUT);
  pinMode(INSIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(INSIDE_ULTRASONIC_ECHO, INPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(GAS_PIN, INPUT);
}

void init_setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(0);
  set_led('G');
  delay(1000);
}

void setup() {
  init_pin();
  init_setup();
}

void loop() {
  state();
  delay(500);
}
