#include <ros.h>
#include <rblemi/Diff.h>
#include <WiFi.h>
#include <analogWrite.h>

#define DEBUG 0

// 25 lb 26lf
// 27 rf 13rb

#define lpwmPin 17 //33 //m1enable , f26, b25
#define lfwdPin 19  
#define lbackPin 18

#define rpwmPin 23 //m2enable , f27, 13
#define rfwdPin 21
#define rbackPin 22

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
//wifi
const char* ssid = "Rigbetel Labs HQ";
const char* password = "Starsoforion2020";
WiFiClient client;
IPAddress server(192,168,0,212);

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);
    }
    int read() {
      return client.read();
    }
    void write(uint8_t* data, int length) {
      for (int i = 0; i < length ; i++)
        client.write(data[i]);
    }
    unsigned long time() {
      return millis();
    }
};

void setupWiFi() {
    
    WiFi.begin(ssid, password);
    Serial.print("\nConnecting to "); Serial.println(ssid);
    uint8_t i = 0;
    while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
    if (i == 21) {
      Serial.print("Could not connect to"); Serial.println(ssid);
      while (1) delay(500);
    }
    Serial.print("Ready! Use ");
    Serial.print(WiFi.localIP());
    Serial.println(" to access client");
  }

int rigpwm = 0;
int lefpwm = 0;

ros::NodeHandle_<WiFiHardware> nh;

void diffCb(const rblemi::Diff& toggle_msg){

  Serial.print("toggle_msg.rpwm.data: ");
  Serial.println(toggle_msg.rpwm.data);

  
  
  analogWrite(rpwmPin, toggle_msg.rpwm.data);
  analogWrite(lpwmPin, toggle_msg.lpwm.data);

  digitalWrite(lfwdPin, !(toggle_msg.ldir.data));
  digitalWrite(lbackPin, toggle_msg.ldir.data);

  Serial.print("toggle_msg.rdir.data: ");
  Serial.println(toggle_msg.rdir.data);
  
  digitalWrite(rfwdPin, toggle_msg.rdir.data);
  digitalWrite(rbackPin, !(toggle_msg.rdir.data));
  
}

ros::Subscriber<rblemi::Diff> sub_diff("diff", diffCb );

void setup() {
  
  pinMode (lpwmPin, OUTPUT);
  pinMode (rpwmPin, OUTPUT);
  pinMode (lfwdPin, OUTPUT);
  pinMode (lbackPin, OUTPUT);
  pinMode (rfwdPin, OUTPUT);
  pinMode (rbackPin, OUTPUT);

  Serial.begin(115200);
  
  setupWiFi();

  ledcSetup(ledChannel, freq, resolution);

  ledcAttachPin(rpwmPin, ledChannel);
  ledcAttachPin(lpwmPin, ledChannel);

  ledcWrite(rpwmPin, 0);
  ledcWrite(lpwmPin, 0);
  
  nh.initNode();
  nh.subscribe(sub_diff);
  
}

void loop() {

  nh.spinOnce();
  delay(10);

}
