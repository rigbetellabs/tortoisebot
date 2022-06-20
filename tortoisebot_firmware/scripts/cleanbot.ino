#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

int cleanmotorRightPwm = 13 ; //White
int cleanmotorRightD1  = 22 ; //Purple
int cleanmotorRightD2  = 23 ; //Blue

int cleanmotorLeftPwm = 12 ; //Orange
int cleanmotorLeftD1  = 24 ; //Yellow
int cleanmotorLeftD2  = 25 ; //Green

int LeftFrontPwm = 11 ; //Red
int LeftFrontD1  = 26 ; //Brown
int LeftFrontD2  = 27 ; //Black

int LeftBackPwm = 10 ; //Purple
int LeftBackD1  = 28 ; //Grey
int LeftBackD2  = 29 ; //White

int RightFrontPwm = 9 ; //Orange
int RightFrontD1  = 30 ; //Yellow
int RightFrontD2  = 31 ; //Green

int RightBackPwm = 8 ; //White
int RightBackD1  = 32 ; //Purple
int RightBackD2  = 33 ; //Blue

int SprayerPwm = 7 ; //White
int SprayerD1  = 34 ; //Brown
int SprayerD2  = 35 ; //Black

int forward = 1 ;
int backward = 0 ;

int rigpwm = 0;
int lefpwm = 0;

ros::NodeHandle nh;

void lpCb(const std_msgs::Int32& toggle_msg){
  left_pwm(toggle_msg.data);
  lefpwm = toggle_msg.data;
}

void ldCb(const std_msgs::Bool& toggle_msg){
  left_dir(toggle_msg.data);
}

void rpCb(const std_msgs::Int32& toggle_msg){
  right_pwm(toggle_msg.data);
  rigpwm = toggle_msg.data;
}

void rdCb(const std_msgs::Bool& toggle_msg){
  right_dir(toggle_msg.data);
}

ros::Subscriber<std_msgs::Int32> sub_lp("lpwm", lpCb );
ros::Subscriber<std_msgs::Bool> sub_ld("ldir", ldCb );
ros::Subscriber<std_msgs::Int32> sub_rp("rpwm", rpCb );
ros::Subscriber<std_msgs::Bool> sub_rd("rdir", rdCb );


void setup() {
  
  pinMode (cleanmotorRightPwm, OUTPUT);
  pinMode (cleanmotorRightD1, OUTPUT);
  pinMode (cleanmotorRightD2, OUTPUT);
  
  pinMode (cleanmotorLeftPwm, OUTPUT);
  pinMode (cleanmotorLeftD1, OUTPUT);
  pinMode (cleanmotorLeftD2, OUTPUT);

  pinMode (LeftFrontPwm, OUTPUT);
  pinMode (LeftFrontD1, OUTPUT);
  pinMode (LeftFrontD2, OUTPUT);

  pinMode (LeftBackPwm, OUTPUT);
  pinMode (LeftBackD1, OUTPUT);
  pinMode (LeftBackD2, OUTPUT);

  pinMode (RightFrontPwm, OUTPUT);
  pinMode (RightFrontD1, OUTPUT);
  pinMode (RightFrontD2, OUTPUT);

  pinMode (RightBackPwm, OUTPUT);
  pinMode (RightBackD1, OUTPUT);
  pinMode (RightBackD2, OUTPUT);

  pinMode (SprayerPwm, OUTPUT);
  pinMode (SprayerD1, OUTPUT);
  pinMode (SprayerD2, OUTPUT);

  analogWrite(cleanmotorRightPwm, 0);
  digitalWrite(cleanmotorRightD1, HIGH);
  digitalWrite(cleanmotorRightD2, HIGH);

  analogWrite(cleanmotorLeftPwm, 0);
  digitalWrite(cleanmotorLeftD1, HIGH);
  digitalWrite(cleanmotorLeftD2, HIGH);

  analogWrite(LeftFrontPwm, 0);
  digitalWrite(LeftFrontD1, HIGH);
  digitalWrite(LeftFrontD2, HIGH);

  analogWrite(LeftBackPwm, 0);
  digitalWrite(LeftBackD1, HIGH);
  digitalWrite(LeftBackD2, HIGH);

  analogWrite(RightFrontPwm, 0);
  digitalWrite(RightFrontD1, HIGH);
  digitalWrite(RightFrontD2, HIGH);

  analogWrite(RightBackPwm, 0);
  digitalWrite(RightBackD1, HIGH);
  digitalWrite(RightBackD2, HIGH);

  analogWrite (SprayerPwm, 0);
  digitalWrite (SprayerD1, HIGH);
  digitalWrite (SprayerD2, HIGH);

  nh.initNode();
  nh.subscribe(sub_lp);
  nh.subscribe(sub_ld);
  nh.subscribe(sub_rp);
  nh.subscribe(sub_rd);
  
}



void loop() {

  nh.spinOnce();

  if (rigpwm <= 10 && lefpwm <= 10)
  {
    stop_clean();
    }

  else
  {
    clean();
    }
  
  delay(10);

 
}

void left_pwm (int pwm)
{
  analogWrite(LeftFrontPwm, pwm);
  analogWrite(LeftBackPwm, pwm);

}

void left_dir (bool dir)
{
  if (dir == 1)
  {
    digitalWrite(LeftFrontD1, HIGH);
    digitalWrite(LeftFrontD2, LOW);
    digitalWrite(LeftBackD1, HIGH);
    digitalWrite(LeftBackD2, LOW);
  }

  else
  {
    digitalWrite(LeftFrontD1, LOW);
    digitalWrite(LeftFrontD2, HIGH);
    digitalWrite(LeftBackD1, LOW);
    digitalWrite(LeftBackD2, HIGH);
  }
  
}

void right_pwm (int pwm)
{
  analogWrite(RightFrontPwm, pwm);
  analogWrite(RightBackPwm, pwm);
}

void right_dir (bool dir)
{
  if (dir == 1)
  {
    digitalWrite(RightFrontD1, LOW);
    digitalWrite(RightFrontD2, HIGH);
    digitalWrite(RightBackD1, LOW);
    digitalWrite(RightBackD2, HIGH);
  }

  else
  {
    digitalWrite(RightFrontD1, HIGH);
    digitalWrite(RightFrontD2, LOW);
    digitalWrite(RightBackD1, HIGH);
    digitalWrite(RightBackD2, LOW);
  }
  
}

void clean()

{

  analogWrite (SprayerPwm, 110);
  digitalWrite (SprayerD1, HIGH);
  digitalWrite (SprayerD2, LOW);
  
  analogWrite(cleanmotorRightPwm, 255);
  digitalWrite(cleanmotorRightD1, HIGH);
  digitalWrite(cleanmotorRightD2, LOW);

    analogWrite(cleanmotorLeftPwm, 255);
  digitalWrite(cleanmotorLeftD1, HIGH);
  digitalWrite(cleanmotorLeftD2, LOW);
}

void stop_clean()

{

  analogWrite (SprayerPwm, 0);
  digitalWrite (SprayerD1, HIGH);
  digitalWrite (SprayerD2, HIGH);
  
  analogWrite(cleanmotorRightPwm, 0);
  digitalWrite(cleanmotorRightD1, HIGH);
  digitalWrite(cleanmotorRightD2, HIGH);

    analogWrite(cleanmotorLeftPwm, 0);
  digitalWrite(cleanmotorLeftD1, HIGH);
  digitalWrite(cleanmotorLeftD2, HIGH);
}
