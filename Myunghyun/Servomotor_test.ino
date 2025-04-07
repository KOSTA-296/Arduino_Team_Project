#include <Servo.h>

Servo myservo;  // 서보 모터 객체 생성
int pos;        // 서보 모터 위치 값을 저장하는 변수

void setup() {
  myservo.attach(9);  // 서보 모터를 9번 핀에 연결
}

void loop() {
  // 서보 모터 회전
  for(pos = 15; pos <= 30; pos += 1){  // 15에서 30까지 1도씩 증가
    myservo.write(pos);    // 서보 모터 위치 설정
    delay(100);            // 0.1초 대기
  }
  delay(3000);             // 3초 대기

  for(pos = 30; pos >= 15; pos -= 1){  // 30에서 15까지 1도씩 감소
    myservo.write(pos);    // 서보 모터 위치 설정
    delay(100);            // 0.1초 대기
  }
  delay(3000);             // 3초 대기
}
