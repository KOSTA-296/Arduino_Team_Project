#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);

String ms = "";
int sp=100;
int a;
int num=5;

void setup() {
  mySerial.begin(9600);

  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
}

void loop() {
  if(mySerial.available()){
    ms = mySerial.readStringUntil('c');
    a=ms.toInt();
    if(a>9){
      sp=a;
    }else{
      num=a;
    }

    if(num==1){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(7,LOW);
      analogWrite(6,sp);
    }else if(num==2){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,LOW);
      digitalWrite(8,HIGH);
      digitalWrite(7,LOW);
      analogWrite(6,sp);
    }else if(num==3){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,LOW);
      digitalWrite(8,HIGH);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }else if(num==4){
      analogWrite(11,sp);
      digitalWrite(10,LOW);
      digitalWrite(9,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(7,LOW);
      analogWrite(6,sp);
    }else if(num==5){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }else if(num==6){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,LOW);
      digitalWrite(8,LOW);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }else if(num==7){
      analogWrite(11,sp);
      digitalWrite(10,HIGH);
      digitalWrite(9,HIGH);
      digitalWrite(8,LOW);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }else if(num==8){
      analogWrite(11,sp);
      digitalWrite(10,LOW);
      digitalWrite(9,HIGH);
      digitalWrite(8,LOW);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }else if(num==9){
      analogWrite(11,sp);
      digitalWrite(10,LOW);
      digitalWrite(9,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(7,HIGH);
      analogWrite(6,sp);
    }
  }
}
