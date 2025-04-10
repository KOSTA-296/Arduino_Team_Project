#include <Servo.h>

// --- 핀 ---

// 전면 초음파 센서 (차단봉 감지용) Ultrasound
#define OUTSIDE_ULTRASONIC_TRIG 2
#define OUTSIDE_ULTRASONIC_ECHO 3
// 차단기 서보모터 핀
#define SERVO_PIN 4
// 삼색 LED 핀
#define RGB_RED_PIN 5
#define RGB_GREEN_PIN 6
#define RGB_BLUE_PIN 7
// 피에조 스피커 핀
#define PIEZO_PIN 8
// 후단 초음파 센서 (주차 감지용)
#define INSIDE_ULTRASONIC_TRIG 9
#define INSIDE_ULTRASONIC_ECHO 10

// --- 상수 ---

#define OUTSIDE_DISTANCE_THRESHOLD 30 // 바깥 센서 거리, 단위 cm
#define INSIDE_DISTANCE_THRESHOLD 30  // 안쪽 센서 거리, 단위 cm
#define GATE_DELAY 2 * 1000           // 게이트 한번 올리고 닫기 시간 2초

// --- 전역 변수 ---

Servo servo_gate;                  // 서보모터 객체
bool is_open_gate = false;         // 차단기 상태 변수, 게이트 닫힘
bool is_on_inside_sensor = false;  // 안쪽 센서 상태 변수
bool is_on_outside_sensor = false; // 바깥 센서 상태 변수
int open_time_gate = 0;            // 게이트 열리는 시간 계산 상태 변수
bool is_parked = true;             // 주차 상태 변수, 주차 가능

// --- 함수 정의 ---

// 삼색 LED 색상 설정 함수 (0~255 값)
void set_led_color(int red, int green, int blue) {
  // 전달된 red, green, blue 값에 따라 해당 채널의 LED 밝기를 조절
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

// LED 색상 설정 함수 (R, G, B 중 하나)
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

// 초음파 센서를 이용한 거리 측정 함수 (cm 단위)
long get_distance(int trigPin, int echoPin) {
  // trigPin과 echoPin을 매개변수로 받아, 거리(cm)를 리턴
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance =
      duration * 0.034 / 2; // 초당 약 340m, 0.034 cm/µs, 왕복이므로 2로 나눔
  return distance;
}

// 경고 알람 함수
void trigger_alarm(int duration = 10) {
  tone(PIEZO_PIN, 1000);
  delay(duration * 1000);
  noTone(PIEZO_PIN);
}

// 게이트 상태 설정 함수
void set_gate(char gate_state) {
  switch (gate_state) {
  case 'O':
    servo_gate.write(90); // 차단기를 90도로 올려 차량 진입 허용
    break;
  case 'C':
    servo_gate.write(0); // 차단기를 0도로 내려 차량 진입 금지
    break;
  }
}

// 안쪽 센서 감지 함수
void state_inside_sensor() {
  long inside_distance =
      get_distance(INSIDE_ULTRASONIC_TRIG, INSIDE_ULTRASONIC_ECHO);

  if (inside_distance <= INSIDE_DISTANCE_THRESHOLD) { // 안쪽 센서가 감지 될 때
    is_on_inside_sensor = true;                       // 안쪽 센서 감지 됨 설정
  } else {                                            // 안쪽 센서 감지 안될 때
    is_on_inside_sensor = false; // 안쪽 센서 감지 안됨 설정
  }
}

// 바깥쪽 센서 감지 함수
void state_outside_sensor() {
  long outside_distance =
      get_distance(OUTSIDE_ULTRASONIC_TRIG, OUTSIDE_ULTRASONIC_ECHO);

  if (outside_distance <=
      OUTSIDE_DISTANCE_THRESHOLD) { // 바깥쪽 센서 감지 될 때
    is_on_outside_sensor = true;    // 바깥쪽 센서 감지 됨 설정
  } else {                          // 바깥쪽 센서 감지 안될 때
    is_on_outside_sensor = false;   // 바깥쪽 센서 감지 안됨 설정
  }
}

// 게이트 자동 닫힘 상태 관리 함수
void state_open_time_gate() {
  if (is_open_gate) {
    if (open_time_gate == 0) {   // 게이트가 열린 시간이 0일 때
      open_time_gate = millis(); // 게이트 열리는 시간 초기 값
    } else if (millis() - open_time_gate >=
               GATE_DELAY) { // 딜레이 시간만큼이 지났으면
      set_gate('C');         // 차단기 닫음
      open_time_gate = 0;    // 게이트 닫히는 시간 초기화
      is_open_gate = false;  // 게이트 닫힘으로 변경
    }
  }
}

// 주차 상태 관리 함수
void state_parked() {
  if (is_on_inside_sensor) {
    is_parked = true;
    set_gate('O');
  } else {
    if (is_on_outside_sensor) {
      is_parked = false;
      set_gate('C');
    }
  }
}

// LED 상태 관리 함수
void state_led() {
  if (is_parked) {
    set_led('R');
  } else {
    set_led('G');
  }
}

// 게이트 상태 관리 함수
void state_gate() {
  if (is_on_outside_sensor && !is_parked) {
    is_open_gate = true;
    set_gate('O');
  }
}

// 상태 관리 함수
void state() {
  state_outside_sensor(); // 바깥쪽 센서 상태 관리
  state_inside_sensor();  // 안쪽 센서 상태 관리
  state_open_time_gate(); // 게이트 문 상태 관리
  state_parked();         // 주차 상태 관리
  state_led();            // LED 상태 관리
  state_gate();           // 게이트 상태 관리
}

// --- main ---
void setup() {
  Serial.begin(9600);

  // --- 핀 모드 설정 ---

  // 바깥 초음파 센서
  pinMode(OUTSIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(OUTSIDE_ULTRASONIC_ECHO, INPUT);

  // 안쪽 초음파 센서
  pinMode(INSIDE_ULTRASONIC_TRIG, OUTPUT);
  pinMode(INSIDE_ULTRASONIC_ECHO, INPUT);

  // 피에조 스피커
  pinMode(PIEZO_PIN, OUTPUT);

  // 삼색 LED
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  // 차단기 서보모터
  pinMode(SERVO_PIN, OUTPUT);
  servo_gate.attach(SERVO_PIN);
  servo_gate.write(0); // 초기 상태: 차단기가 내려감 (닫힘)

  set_led('G'); // 초기 상태 초록색
}

void loop() {
  state(); // 전체 상태 관리 함수 호출

  delay(1000); // 센서 안정화를 위한 1초 대기
}
