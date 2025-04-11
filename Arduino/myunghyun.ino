#include <Servo.h>
#include <U8glib.h>

// -------------- 핀 정의 --------------
// 전면 초음파 센서 (차단봉 감지용)
#define OUTSIDE_ULTRASONIC_TRIG 2
#define OUTSIDE_ULTRASONIC_ECHO 3

// 후단 초음파 센서 (주차 감지용)
#define INSIDE_ULTRASONIC_TRIG 6
#define INSIDE_ULTRASONIC_ECHO 7

// 피에조 스피커 핀
#define PIEZO_PIN 12

// RGB LED 핀 (공통 캐소드 기준; analogWrite 사용)
#define RGB_RED_PIN 11
#define RGB_GREEN_PIN 10
#define RGB_BLUE_PIN 9

// 차단기 서보모터 핀
#define BARRIER_SERVO_PIN 8

// 임계 거리 설정 (cm)
#define GATE_THRESHOLD 30   // 전면 센서 기준
#define PARK_THRESHOLD 30   // 후단 센서 기준 (차가 주차되었는지 확인)

// 차단기 하강 딜레이: 차량이 빠져나간 후 10초(10000ms) 대기
#define GATE_DELAY 10000

// -------------- 객체 및 상태 변수 --------------
Servo barrierServo;    // 차단기 서보모터 제어 객체
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // OLED 디스플레이 (0.96인치, 128x64)

// 상태 변수
bool parked = false;                // 주차장에 차량 여부
unsigned long carLeftTime = 0;      // 차량이 전면 센서에서 빠져나간 시간 기록
long CarCatch = 0;                  // 주차장에서 나가는 차량 감지용(게이트 초음파 센서 사용)

// -------------- 함수 정의 --------------
void setLEDColor(int red, int green, int blue);
long measureDistance(int trigPin, int echoPin);
void displayParkingStatus(bool parked);
void control_parkinglot();
void displayCarout();

// -------------- setup() --------------
void setup() {
    Serial.begin(9600);
    
    // 핀 모드 설정
    // 전면 센서
    pinMode(OUTSIDE_ULTRASONIC_TRIG, OUTPUT);
    pinMode(OUTSIDE_ULTRASONIC_ECHO, INPUT);
    
    // 후단 센서
    pinMode(INSIDE_ULTRASONIC_TRIG, OUTPUT);
    pinMode(INSIDE_ULTRASONIC_ECHO, INPUT);
    
    // 피에조 스피커
    pinMode(PIEZO_PIN, OUTPUT);
    
    // RGB LED
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    
    // 차단기 서보모터
    barrierServo.attach(BARRIER_SERVO_PIN);
    barrierServo.write(0);  // 초기 상태: 차단기가 내려감 (닫힘)
    
    // 초기 LED 상태 및 OLED 초기화: 주차장 기본 상태 = "자리 비었음" -> LED: 빨간색
    setLEDColor(255, 0, 0);
    displayParkingStatus(parked);
  }
// -------------- loop() --------------
void loop() {
  control_parkinglot();
  if (parked) {
    // 차량 주차: OLED에 "만차" 출력, RGB LED를 초록색으로 (예: (0,255,0))
    displayParkingStatus(parked);
    setLEDColor(0, 255, 0);
  }
  else {
    // 주차 공간 비어있음: OLED에 "자리 비었음" 출력, RGB LED를 빨간색으로 (예: (255,0,0))
    displayParkingStatus(parked);
    setLEDColor(255, 0, 0);
  }
  delay(100); // 센서 안정화를 위한 0.1초 대기
}

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
  long distance = duration * 0.034 / 2; // 초당 약 340m, 0.034 cm/µs, 왕복이므로 2로 나눔
  return distance;
}

// OLED에 주차 상태를 출력하는 함수
// 만차: "만차" 출력, 빈 자리: "자리 비었음" 출력
void displayParkingStatus(bool parked) {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_fub14);
    u8g.setPrintPos(10,30);
    if (parked)
      u8g.print("만차");
    else
      u8g.print("자리 비었음");
  } while(u8g.nextPage());
}

void displayCarout() {
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_fub14);
      u8g.setPrintPos(10,30);
      u8g.print("출차중");
    } while(u8g.nextPage());
  }
  

void control_parkinglot() {
  // 1. 전면 초음파 센서로 차량 접근 감지
  long frontDistance = measureDistance(OUTSIDE_ULTRASONIC_TRIG, OUTSIDE_ULTRASONIC_ECHO);
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");
  
  // 2. 후단 초음파 센서로 주차 여부 측정
  long parkDistance = measureDistance(INSIDE_ULTRASONIC_TRIG, INSIDE_ULTRASONIC_ECHO);
  Serial.print("Park Distance: ");
  Serial.print(parkDistance);
  Serial.println(" cm");
  if (parkDistance > 0 && parkDistance < PARK_THRESHOLD){
    parked = true;
    Serial.println("만차입니다!");
    return;
  }
  else{
    if (parked){        // 이미 주차되어 있던 경우 -> 차량이 빠져나가려는 상황
        int cnt = 0;
        Serial.println("출차입니다!");
        barrierServo.write(90);
        tone(PIEZO_PIN, 1000);
        delay(3000);
        noTone(PIEZO_PIN);
        while(cnt < 10){
            CarCatch = measureDistance(OUTSIDE_ULTRASONIC_TRIG, OUTSIDE_ULTRASONIC_ECHO);
            Serial.print("나가는 차량 감지 중: ");
            Serial.print(CarCatch);
            Serial.println(" cm");
            if (CarCatch < GATE_THRESHOLD) {
                break;
            }
            cnt++;
            delay(1000);
        }
        barrierServo.write(0);
        Serial.print("나가는 차량 감지 완료 차단기 닫겠습니다.");
    }
    else{
        Serial.println("주차 공간이 있습니다!");
    }
    parked = false;
  }

  // 3. 차단기(서보모터) 제어 및 경고 알림 처리
  if (frontDistance > 0 && frontDistance < GATE_THRESHOLD && parked == false) {
    // 차량이 전면에서 30cm 이내로 접근한 경우
    barrierServo.write(90);  // 차단기를 90도로 올려 차량 진입 허용
    carLeftTime = 0;         // 차량이 떠난 시간을 초기화
    Serial.println("Barrier raised!");

    tone(PIEZO_PIN, 1000);
    delay(3000);
    noTone(PIEZO_PIN);
  }
  else if (frontDistance > GATE_THRESHOLD && parked == false) {
    // 차량이 전면 센서에서 30cm 이상일 때 → 차량이 주차장으로 들어가지 않은 상태 (또는 떠난 상태)
    if (carLeftTime == 0) {
      carLeftTime = millis();
      Serial.println("Car left, starting timer to lower barrier.");
    }
    else {
      // 10초(10000ms)가 지난 후 차단기를 내려서 닫음
      if (millis() - carLeftTime >= GATE_DELAY) {
        barrierServo.write(0);  // 차단기를 0도로 내려 닫음
        carLeftTime = 0;
        Serial.println("Barrier lowered!");
      }
    }
  }
}
