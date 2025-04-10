#include <Servo.h>
#include <U8glib.h>

// -------------- 핀 정의 --------------
// 전면 초음파 센서 (차단봉 감지용)
#define TRIG_FRONT 2
#define ECHO_FRONT 3

// 후단 초음파 센서 (주차 감지용)
#define TRIG_PARK 6
#define ECHO_PARK 7

// 피에조 스피커 핀
#define PIEZO_PIN 12

// RGB LED 핀 (공통 캐소드 기준; analogWrite 사용)
#define RGB_RED_PIN 11
#define RGB_GREEN_PIN 10
#define RGB_BLUE_PIN 9

// 차단기 서보모터 핀
#define BARRIER_SERVO_PIN 8

// 임계 거리 설정 (cm)
#define DIST_THRESHOLD 30 // 전면 센서 기준
#define PARK_THRESHOLD 30 // 후단 센서 기준 (차가 주차되었는지 확인)

// 차단기 하강 딜레이: 차량이 빠져나간 후 10초(10000ms) 대기
#define DELAY_BEFORE_LOWER 10000

// -------------- 객체 및 상태 변수 --------------
Servo barrierServo; // 차단기 서보모터 제어 객체
U8GLIB_SSD1306_128X64
    u8g(U8G_I2C_OPT_NONE); // OLED 디스플레이 (0.96인치, 128x64)

// 상태 변수
bool barrierRaised = false; // 차단기가 현재 올라갔는지
unsigned long carLeftTime = 0; // 차량이 전면 센서에서 빠져나간 시간 기록
bool alarmTriggered = false; // 경고음이 한 번 발생했는지 체크

// -------------- 함수 정의 --------------

// RGB LED 색상 설정 함수 (0~255 값)
// 전달된 red, green, blue 값에 따라 해당 채널의 LED 밝기를 조절
void setLEDColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

// 초음파 센서를 이용한 거리 측정 함수 (cm 단위)
// trigPin과 echoPin을 매개변수로 받아, 거리(cm)를 리턴
long measureDistance(int trigPin, int echoPin) {
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

// OLED에 주차 상태를 출력하는 함수
// 만차: "만차" 출력, 빈 자리: "자리 비었음" 출력
void displayParkingStatus(bool parked) {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_fub14);
    u8g.setPrintPos(10, 30);
    if (parked)
      u8g.print("만차");
    else
      u8g.print("자리 비었음");
  } while (u8g.nextPage());
}

// -------------- setup() --------------
void setup() {
  Serial.begin(9600);

  // 핀 모드 설정
  // 전면 센서
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // 후단 센서
  pinMode(TRIG_PARK, OUTPUT);
  pinMode(ECHO_PARK, INPUT);

  // 피에조 스피커
  pinMode(PIEZO_PIN, OUTPUT);

  // RGB LED
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  // 차단기 서보모터
  pinMode(BARRIER_SERVO_PIN, OUTPUT);
  barrierServo.attach(BARRIER_SERVO_PIN);
  barrierServo.write(0); // 초기 상태: 차단기가 내려감 (닫힘)

  // 초기 LED 상태 및 OLED 초기화: 주차장 기본 상태 = "자리 비었음" -> LED:
  // 빨간색
  setLEDColor(255, 0, 0);
  displayParkingStatus(false);
}

// -------------- loop() --------------
void loop() {
  // 1. 전면 초음파 센서로 차량 접근 감지
  long frontDistance = measureDistance(TRIG_FRONT, ECHO_FRONT);
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  // 2. 후단 초음파 센서로 주차 여부 측정
  long parkDistance = measureDistance(TRIG_PARK, ECHO_PARK);
  Serial.print("Park Distance: ");
  Serial.print(parkDistance);
  Serial.println(" cm");

  // 3. 차단기(서보모터) 제어 및 경고 알림 처리
  if (frontDistance > 0 && frontDistance < DIST_THRESHOLD) {
    // 차량이 전면에서 30cm 이내로 접근한 경우

    // 차단기가 아직 올라가지 않았다면 바로 올리기
    if (!barrierRaised) {
      barrierServo.write(90); // 차단기를 90도로 올려 차량 진입 허용
      barrierRaised = true;
      carLeftTime = 0; // 차량이 떠난 시간을 초기화
      Serial.println("Barrier raised!");
    }

    // 경고 알림: 아직 알림이 발생하지 않았다면 피에조로 1000Hz 음을 3초 동안
    // 울림
    if (!alarmTriggered) {
      tone(PIEZO_PIN, 1000);
      delay(3000);
      noTone(PIEZO_PIN);
      alarmTriggered = true;
    }
  } else {
    // 차량이 전면 센서에서 30cm 이상일 때 → 차량이 주차장으로 들어가지 않은
    // 상태 (또는 떠난 상태)
    if (barrierRaised) {
      // 차량이 빠져나간 시점을 기록 (최초 한 번만)
      if (carLeftTime == 0) {
        carLeftTime = millis();
        Serial.println("Car left, starting timer to lower barrier.");
      } else {
        // 10초(10000ms)가 지난 후 차단기를 내려서 닫음
        if (millis() - carLeftTime >= DELAY_BEFORE_LOWER) {
          barrierServo.write(0); // 차단기를 0도로 내려 닫음
          barrierRaised = false;
          alarmTriggered = false; // 다음 차량을 위한 알림 플래그 리셋
          carLeftTime = 0;
          Serial.println("Barrier lowered!");
        }
      }
    }
  }

  // 4. 주차 상태 업데이트 (후단 센서 기준)
  // 후단 센서가 30cm 미만이면 차량이 주차되었다고 판단
  bool parked = (parkDistance > 0 && parkDistance < PARK_THRESHOLD);
  if (parked) {
    // 차량 주차: OLED에 "만차" 출력, RGB LED를 초록색으로 (예: (0,255,0))
    displayParkingStatus(true);
    setLEDColor(0, 255, 0);
  } else {
    // 주차 공간 비어있음: OLED에 "자리 비었음" 출력, RGB LED를 빨간색으로 (예:
    // (255,0,0))
    displayParkingStatus(false);
    setLEDColor(255, 0, 0);
  }

  delay(100); // 센서 안정화를 위한 0.1초 대기
}
