# RC_Car 자율주행 코드 설명

이 문서는 RC_Car 아두이노 프로젝트 코드의 구조 및 함수별 역할에 대한 설명을 담고 있습니다.

---

## 📁 주요 클래스: `RC_Car`

`RC_Car` 클래스는 RC카의 주요 기능(수동 제어, 자율주행, 모터 및 서보 제어 등)을 담당합니다.

### 생성자
```cpp
RC_Car(Stream* serial)
```
- 블루투스 수신용 시리얼 객체를 받아 내부에서 통신에 사용합니다.

---

## 🔧 주요 함수 설명

### `void begin()`
- DC 모터, 서보모터, OLED, LED 초기화
- 기본 속도 설정
- OLED에 모드 안내 메시지 출력

---

### `void update()`
- 블루투스 데이터 수신 처리
- 데이터가 없을 경우 초음파 센서 값을 기반으로 자율주행 함수 `autoDrive()` 실행
- 데이터가 있는 경우 숫자 명령에 따라 수동 제어 함수 실행

---

## 🚗 자동차 동작 함수

각 함수는 특정 상황에 따른 차량의 움직임을 정의합니다.

| 함수명 | 설명 |
|--------|------|
| `leftForward()` | 좌측으로 조향하며 전진 |
| `forward()` | 전방 직진 |
| `rightForward()` | 우측으로 조향하며 전진 |
| `turnLeft()` | 제자리 좌회전 |
| `turnRight()` | 제자리 우회전 |
| `leftBack()` | 좌측으로 조향하며 후진 |
| `back()` | 후진 |
| `rightBack()` | 우측으로 조향하며 후진 |
| `stop()` | 차량 정지, 모든 LED 끔 |

---

## 🤖 자율 주행 관련 함수

### `void autoDrive(long left, long front, long right)`

- 초음파 센서 3개의 값을 기반으로 장애물 판단 및 조향을 수행합니다.
- 판단 순서 (우선순위):
  1. 세 방향 모두 장애물 → 후진 2초
  2. 전방 + 오른쪽 장애물 → 좌회전
  3. 전방 + 왼쪽 장애물 → 우회전
  4. 전방 장애물 → 좌/우 중 여유가 있는 방향으로 회피
  5. 좌/우만 장애물 → 반대 방향으로 조향
  6. 장애물 없음 → 전방 유지

### `void autoLeft()` / `void autoRight()`
- 자동 좌/우 회전을 수행하는 보조 함수
- 후진 후 방향 전환을 통해 장애물 회피

---

## 🔌 블루투스 명령 매핑

| 명령 숫자 | 수행 동작 |
|-----------|------------|
| 1 | `leftForward()` |
| 2 | `forward()` |
| 3 | `rightForward()` |
| 4 | `turnLeft()` |
| 5 | `stop()` |
| 6 | `turnRight()` |
| 7 | `leftBack()` |
| 8 | `back()` |
| 9 | `rightBack()` |
| 그 외 | `stop()` |

---

## 🪛 main 루프 구조

```cpp
void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);
  Wire.begin();       // I2C 초기화
  pcf.begin();        // PCF8574 확장 모듈 초기화
  car.begin();        // 차량 구성요소 초기화
}

void loop() {
  car.update();       // 매 루프마다 블루투스 또는 자율주행 모드 처리
}
```

---

## 📌 상수 정의

```cpp
#define OBSTACLE_THRESHOLD 30
#define LR_THRESHOLD 15
#define TXD 2
#define RXD 3
```
- 초음파 센서 감지 거리 기준
- 블루투스 연결 핀 설정

---

## 📎 기타 구성 요소

- `motor`, `servo`, `led`: 각각 DC모터, 서보모터, LED 제어를 위한 객체
- `btSerial`: 블루투스 통신용 SoftwareSerial 객체

---

## 📝 비고

- 장애물 회피 시 `stop()` 호출 후 방향 전환
- 자율주행과 수동 조작이 자연스럽게 전환됨 (입력 감지 기반)
- 추후 기능 확장을 위한 코드 구조가 잘 정리되어 있음
