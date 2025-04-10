#include <Servo.h>
// #include <U8glib.h> // OLED 안씀

// --- 핀 ---

// 전면 초음파 센서 (차단봉 감지용) Ultrasound
#define OUTSIDE_ULTRASONIC_TRIG 2
#define OUTSIDE_ULTRASONIC_ECHO 3

// 차단기 서보모터 핀
#define SERVO_PIN 4

// 삼색 LED 핀
#define RGB_RED_PIN 5
#define RGB_BLUE_PIN 6
#define RGB_GREEN_PIN 7

// 피에조 스피커 핀
#define PIEZO_PIN 8

// 후단 초음파 센서 (주차 감지용)
#define INSIDE_ULTRASONIC_TRIG 9
#define INSIDE_ULTRASONIC_ECHO 10

// --- 상수 ---

// 임계 거리 설정 (cm)
#define OUTSIDE_DISTANCE_THRESHOLD 30 // 바깥 센서 거리 임계
#define INSIDE_DISTANCE_THRESHOLD                                              \
  30 // 안쪽 주차장 센서 거리 임계 (차가 주차되었는지 확인)

// 차단기 하강 딜레이: 차량이 빠져나간 후 10초(10000ms) 대기
#define GATE_DELAY 10000

// --- 전역 변수 ---
Servo servo_gate;                // 차단기 서보모터 제어 객체
bool gate_is_open = false;       // 차단기가 현재 올라갔는지
unsigned long car_left_time = 0; // 차량이 전면 센서에서 빠져나간 시간 기록
bool alarm_triggered = false;    // 경고음이 한 번 발생했는지 체크
// U8GLIB_SSD1306_128X64 // 디스플레이 설정
// u8g(U8G_I2C_OPT_NONE); // OLED 디스플레이 (0.96인치, 128x64)

// --- 함수 정의 ---

// 삼색 LED 색상 설정 함수 (0~255 값)
void set_led_color(int red, int green, int blue) {
  // 전달된 red, green, blue 값에 따라 해당 채널의 LED 밝기를 조절
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
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

// 주차장 게이트 제어 함수
void control_gate() {
  long outside_distance = get_distance(
      OUTSIDE_ULTRASONIC_TRIG,
      OUTSIDE_ULTRASONIC_ECHO); // 바깥 초음파 센서로 차량 접근 감지
  long inside_distance = get_distance(
      INSIDE_ULTRASONIC_TRIG,
      INSIDE_ULTRASONIC_ECHO); // 안쪽 주차장 초음파 센서로 주차 여부 감지

  // 차단기 제어
  if (outside_distance <
      OUTSIDE_DISTANCE_THRESHOLD) { // 차량이 전면에서 30cm 이내로 접근한 경우
    // 차단기가 아직 올라가지 않았다면 바로 올리기
    if (!gate_is_open) {
      servo_gate.write(90); // 차단기를 90도로 올려 차량 진입 허용
      gate_is_open = true;
      car_left_time = 0; // 차량이 떠난 시간을 초기화
    }

    // 경고 알림: 아직 알림이 발생하지 않았다면 피에조로 1000Hz 음을 3초 동안
    if (!alarm_triggered) { // 울림
      tone(PIEZO_PIN, 1000);
      delay(3000);
      noTone(PIEZO_PIN);
      alarm_triggered = true;
    }
  } else { // 차량이 전면 센서에서 30cm 이상일 때 → 차량이 주차장으로 들어가지
           // 않은 상태 (또는 떠난 상태)
    if (gate_is_open) {
      if (car_left_time == 0) {
        car_left_time = millis(); // 차량이 빠져나간 시점을 기록 (최초 한 번만)
      } else {
        // 10초(10000ms)가 지난 후 차단기를 내려서 닫음
        if (millis() - car_left_time >= GATE_DELAY) {
          servo_gate.write(0);     // 차단기를 0도로 내려 닫음
          gate_is_open = false;    // 게이트 닫힘
          alarm_triggered = false; // 다음 차량을 위한 알림 플래그 리셋
          car_left_time = 0;       // 차량이 떠난 시간을 초기화
        }
      }
    }
  }
}

// OLED에 주차 상태를 출력하는 함수
// void get_status_display(bool parked) {
//   // 만차: "만차" 출력, 빈 자리: "자리 비었음" 출력
//   u8g.firstPage();
//   do {
//     u8g.setFont(u8g_font_fub14);
//     u8g.setPrintPos(10, 30);
//     if (parked)
//       u8g.print("만차");
//     else
//       u8g.print("자리 비었음");
//   } while (u8g.nextPage());
// }

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

  // 초기 LED 상태 및 OLED 초기화: 주차장 기본 상태 = "자리 비었음" -> LED:
  set_led_color(255, 0, 0); // 빨간색
  // displayParkingStatus(false);
}

void loop() {
  control_gate(); // 주차장 게이트 관리 함수 호출

  // // 주차 상태 업데이트 (후단 센서 기준)
  // // 후단 센서가 30cm 미만이면 차량이 주차되었다고 판단
  // bool parked = (parkDistance > 0 && parkDistance < PARK_THRESHOLD);
  // if (parked) {
  //   // 차량 주차: OLED에 "만차" 출력, RGB LED를 초록색으로 (예: (0,255,0))
  //   displayParkingStatus(true);
  //   set_led_color(0, 255, 0);
  // } else {
  //   // 주차 공간 비어있음: OLED에 "자리 비었음" 출력, RGB LED를 빨간색으로
  //   (예:
  //   // (255,0,0))
  //   displayParkingStatus(false);
  //   set_led_color(255, 0, 0);
  // }

  delay(100); // 센서 안정화를 위한 0.1초 대기
}
