#include <Servo.h>

// --- 핀 ---

// 전면 초음파 센서 (게이트)
constexpr uint8_t OUTSIDE_ULTRASONIC_TRIG = 2;
constexpr uint8_t OUTSIDE_ULTRASONIC_ECHO = 3;
// 차단기 서보모터 핀
constexpr uint8_t SERVO_PIN = 5;
// 삼색 LED 핀
constexpr uint8_t RGB_RED_PIN = 10;
constexpr uint8_t RGB_GREEN_PIN = 9;
// constexpr uint8_t RGB_BLUE_PIN = 10;
// 피에조 스피커 핀
constexpr uint8_t PIEZO_PIN = 7;
// 후단 초음파 센서 (주차 감지용)
constexpr uint8_t INSIDE_ULTRASONIC_TRIG = 13;
constexpr uint8_t INSIDE_ULTRASONIC_ECHO = 11;
// DC 모터
constexpr uint8_t MOTOR_PIN = 6;
// 가스 센서 핀
constexpr uint8_t GAS_PIN = A0;

// --- 상수 ---

constexpr uint8_t OUTSIDE_DISTANCE_THRESHOLD = 30; // 바깥 센서 거리, 단위 cm
constexpr uint8_t INSIDE_DISTANCE_THRESHOLD = 30;  // 안쪽 센서 거리, 단위 cm
constexpr uint16_t GATE_DELAY = 2 * 1000; // 게이트 한번 올리고 닫기 시간 2초
constexpr uint16_t GAS_THRESHOLD = 250;   // 가스 감지 임계값

// --- 전역 변수 ---

struct SystemState {
  bool is_gate_open = false;         // 게이트 상태
  bool auto_close_gate = false;      // 자동 닫기 모드
  bool is_on_inside_sensor = false;  // 안쪽 센서 상태
  bool is_on_outside_sensor = false; // 바깥쪽 센서 상태
  int time_gate_open = 0;            // 게이트 열린 시간
  bool is_parking_available = true;  // 주차 상태 변수, 주차 가능
  bool is_on_motor = false;          // 모터 상태
  bool is_gas_detected = false;      // 가스 감지 상태
};

SystemState ss;
Servo servo; // 서보모터 객체

// --- 함수 정의 ---

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

// 가스 센서 값을 읽어와 리턴하는 함수
int get_gas() {
  int gas = analogRead(GAS_PIN); // 가스 센서 값 읽기
  return gas;
}

// 삼색 LED 색상 설정 함수 (0~255 값)
void set_led_color(int red, int green, int blue) {
  // 전달된 red, green, blue 값에 따라 해당 채널의 LED 밝기를 조절
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  // analogWrite(RGB_BLUE_PIN, blue);
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

// 0 ~ 4
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

// 게이트 상태 설정 함수
void set_gate(char gate_state) {
  switch (gate_state) {
  case 'O':
    servo.write(90); // 차단기를 90도로 올려 차량 진입 허용
    ss.is_gate_open = true;
    break;
  case 'C':
    servo.write(0); // 차단기를 0도로 내려 차량 진입 금지

    ss.is_gate_open = false;
    break;
  }
}

// 경고 알람 함수
void trigger_alarm(int duration = 10) {
  tone(PIEZO_PIN, 1000);
  delay(duration * 1000);
  noTone(PIEZO_PIN);
}

// 안쪽 센서 감지 함수
void state_inside_sensor() {
  long inside_distance =
      get_distance(INSIDE_ULTRASONIC_TRIG, INSIDE_ULTRASONIC_ECHO);

  if (inside_distance <= INSIDE_DISTANCE_THRESHOLD) { // 안쪽 센서가 감지 될 때
    ss.is_on_inside_sensor = true;                    // 안쪽 센서 감지 됨 설정
  } else {                                            // 안쪽 센서 감지 안될 때
    ss.is_on_inside_sensor = false; // 안쪽 센서 감지 안됨 설정
  }
}

// 바깥쪽 센서 감지 함수
void state_outside_sensor() {
  long outside_distance =
      get_distance(OUTSIDE_ULTRASONIC_TRIG, OUTSIDE_ULTRASONIC_ECHO);

  if (outside_distance <=
      OUTSIDE_DISTANCE_THRESHOLD) {  // 바깥쪽 센서 감지 될 때
    ss.is_on_outside_sensor = true;  // 바깥쪽 센서 감지 됨 설정
  } else {                           // 바깥쪽 센서 감지 안될 때
    ss.is_on_outside_sensor = false; // 바깥쪽 센서 감지 안됨 설정
  }
}

// 게이트 자동 닫힘 상태 관리 함수
void state_time_gate_open() {
  if (ss.auto_close_gate) {
    if (ss.time_gate_open == 0) {   // 게이트가 열린 시간이 0일 때
      ss.time_gate_open = millis(); // 게이트 열리는 시간 초기 값
    } else if (millis() - ss.time_gate_open >=
               GATE_DELAY) {      // 딜레이 시간만큼이 지났으면
      set_gate('C');              // 차단기 닫음
      ss.time_gate_open = 0;      // 게이트 닫히는 시간 초기화
      ss.auto_close_gate = false; // 게이트 자동 닫힘 비활성화
    }
  }
}

// 주차 상태 관리 함수
void state_parked() {
  if (ss.is_on_inside_sensor) { // 안쪽 센서 감지 됨
    ss.is_parking_available = false;
  } else {
    if (!ss.is_parking_available && ss.is_on_outside_sensor) {
      ss.is_parking_available = true;
    }
  }
}

// LED 상태 관리 함수
void state_led() {
  if (!ss.is_parking_available) { // 주차 불가능
    set_led('R');
  } else {
    set_led('G');
  }
}

// 게이트 상태 관리 함수
void state_gate() {
  if (ss.is_on_outside_sensor &&
      ss.is_parking_available) { // 바깥쪽 센서 감지 && 주차 가능
    set_gate('O');
  }
  if (ss.is_gate_open &&
      !ss.is_on_outside_sensor) { // 게이트 열림 && 바깥쪽 센서 감지 안됨
    ss.auto_close_gate = true;    // 자동 닫힘 설정
  }
  if (!ss.is_on_inside_sensor &&
      !ss.is_parking_available) { // 안쪽 센서 감지 안됨 && 주차 불가능
    set_gate('O');
  }
  if (ss.is_on_inside_sensor &&
      !ss.is_parking_available) { // 안쪽 센서 감지 && 주차 불가능
    set_gate('C');
  }
}

void state_alarm() {
  if (ss.is_gate_open) {
    trigger_alarm(3);
  }
}

void state_motor() {
  if (ss.is_gas_detected) {
    set_motor(4); // 가스 배출
  } else {
    set_motor(1); // 평상시 공기 정화
  }
}

// 가스 센서 감지 상태 함수
void state_gas() {
  int gas = get_gas();
  if (gas > GAS_THRESHOLD) {
    ss.is_gas_detected = true;
  } else {
    ss.is_gas_detected = false;
  }
}

// 상태 관리 함수
void state() {
  state_outside_sensor(); // 바깥쪽 센서 상태 관리
  state_inside_sensor();  // 안쪽 센서 상태 관리
  state_time_gate_open(); // 게이트 문 상태 관리
  state_parked();         // 주차 상태 관리
  state_led();            // LED 상태 관리
  state_gate();           // 게이트 상태 관리
  state_alarm();          // 경고 상태 관리
  state_motor();          // 모터 상태 관리
  state_gas();            // 가스 센서 감지 상태 관리
}

void init_pin() {
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
  // pinMode(RGB_BLUE_PIN, OUTPUT);

  // 차단기 서보모터
  pinMode(SERVO_PIN, OUTPUT);

  // DC 모터
  pinMode(MOTOR_PIN, OUTPUT);

  // 가스 센서 핀
  pinMode(GAS_PIN, INPUT);
}

void init_setup() {
  Serial.begin(9600);

  servo.attach(SERVO_PIN);
  servo.write(0); // 초기 상태: 차단기가 내려감 (닫힘)

  set_led('G'); // 초기 상태 초록색

  delay(1000); // 센서 초기 안정화를 위해 추가
}

// --- main ---
void setup() {
  init_pin();
  init_setup();
}

void loop() {
  state(); // 전체 상태 관리 함수 호출

  delay(500); // 센서 안정화를 위한 0.5초 대기
}
