#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

int rm1 =8,rm2 =9,rme =10,lm1 =12,lm2 =13,lme =11;
int lctr=0,rctr=0,lrpm,rrpm,duration,prev=0,cur=0;
volatile int lpos=0,rpos=0;
float rrot=0,lrot=0;
int lsp=255,rsp=255;

void lm()
{
  lpos=1;rpos=0;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
}


void rm()
{
  rpos=1;lpos=0;
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void forward()
{
  lpos=rpos=1;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void reverse()
{
  lpos=rpos=2;
  analogWrite(lme,lsp);
  digitalWrite(lm2,HIGH);
  digitalWrite(lm1,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,LOW);
}

void left()
{
  lpos=2;
  rpos=1;
  analogWrite(lme,lsp);
  digitalWrite(lm2,HIGH);
  digitalWrite(lm1,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm1,HIGH);
  digitalWrite(rm2,LOW);
}

void right()
{
  rpos=2;
  lpos=1;
  analogWrite(lme,lsp);
  digitalWrite(lm1,HIGH);
  digitalWrite(lm2,LOW);
  analogWrite(rme,rsp);
  digitalWrite(rm2,HIGH);
  digitalWrite(rm1,LOW);
}

void halt()
{
  lpos=rpos=0;
  digitalWrite(lme,LOW);
  digitalWrite(rme,LOW);
}

ISR(INT1_vect)
{
  lctr++;
  delay(10);
  //Serial.print("left ");Serial.println(lctr/32);
}


ISR(INT0_vect)
{
  rctr++;
  delay(10);
  //Serial.print("right ");Serial.println(rctr/30);
}
/*
void l()
{
  lctr++;
  _delay_ms(10);
}


void r()
{
  rctr++;
  _delay_ms(10);
}
*/
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  sei();
  EIMSK|=(1<<INT0)|(1<<INT1);   
  EICRA|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);  
  pinMode (rm1, OUTPUT);
  pinMode (rm2, OUTPUT);
  pinMode (rme, OUTPUT);
  pinMode (lm1, OUTPUT);
  pinMode (lm2, OUTPUT);
  pinMode (lme, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(2), l , RISING);
  //attachInterrupt(digitalPinToInterrupt(3), r , RISING);
  pinMode (2,INPUT);
  pinMode (3,INPUT);
}
 
void loop() 
{
  // put your main code here, to run repeatedly:
  //lm();
  //rm();
  forward();
  //reverse();
  lrot=lctr/30.0;
  rrot=rctr/30.0;
  Serial.print(lrot);
  Serial.print(" ");
  Serial.println(rrot);
}
