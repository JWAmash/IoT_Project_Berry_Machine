#include <WiFi.h>          // UNO R4 WiFi용 Wi-Fi 라이브러리
#include <PubSubClient.h>    // MQTT 통신을 위한 라이브러리

// Wi-Fi 및 MQTT 브로커 설정
const char* ssid = "KT_GiGA_0BF0";               // Wi-Fi SSID
const char* password = "bke72cg443";             // Wi-Fi 비밀번호
const char* mqtt_server = "test.mosquitto.org";  // MQTT 브로커 주소
const int mqtt_port = 1883;                      // MQTT 포트 번호 (일반적으로 1883)

// MQTT 토픽 설정
const char* mqtt_topic_status = "light/status";    // 조명 상태 토픽
const char* mqtt_topic_control = "light/control";  // 조명 제어 토픽

// 하드웨어 핀 설정
const int soundSensor = 2;  // 사운드 센서 핀 (인터럽트 지원 핀)
const int relayPin = 7;     // 릴레이 제어 핀

// 변수 선언
volatile bool clapDetected = false;   // 박수 감지 플래그 (인터럽트에서 사용)
bool lightState = false;              // 조명 상태 (켜짐/꺼짐)
int clapCount = 0;                    // 박수 횟수
unsigned long lastClapTime = 0;       // 마지막 박수 시간
const unsigned long clapInterval = 1000; // 박수 간 최대 간격 (밀리초)

// Wi-Fi 및 MQTT 클라이언트 객체 생성
WiFiClient espClient;
PubSubClient client(espClient);

// 박수 감지 인터럽트 핸들러
void detectClap() {
  clapDetected = true;
}

// Wi-Fi 연결 함수
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT 콜백 함수
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  message.trim(); // 메시지의 앞뒤 공백 제거

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  Serial.print("Topic as String: ");
  Serial.println(String(topic));
  Serial.print("mqtt_topic_control: ");
  Serial.println(mqtt_topic_control);

  // 문자열 비교 시 equals() 사용
  if (String(topic).equals(mqtt_topic_control)) {
    if (message.equals("ON")) {
      lightState = true;
      digitalWrite(relayPin, HIGH);
      bool publishResult = client.publish(mqtt_topic_status, "ON");
      Serial.println("Light turned ON by MQTT");
      Serial.print("Publish result: ");
      Serial.println(publishResult ? "Success" : "Failure");
    } else if (message.equals("OFF")) {
      lightState = false;
      digitalWrite(relayPin, LOW);
      bool publishResult = client.publish(mqtt_topic_status, "OFF");
      Serial.println("Light turned OFF by MQTT");
      Serial.print("Publish result: ");
      Serial.println(publishResult ? "Success" : "Failure");
    } else {
      Serial.println("Unknown command received");
    }
  } else {
    Serial.print("Message received on unknown topic: ");
    Serial.println(topic);
  }
}

// MQTT 재연결 함수
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // 클라이언트 ID를 고유하게 설정어제

    if (client.connect("ArduinoClient123")) {
      Serial.println("connected");
      bool subscribeResult = client.subscribe(mqtt_topic_control);
      Serial.print("Subscribe result: ");
      Serial.println(subscribeResult ? "Success" : "Failure");
      bool publishResult = client.publish(mqtt_topic_status, lightState ? "ON" : "OFF");
      Serial.print("Initial status publish result: ");
      Serial.println(publishResult ? "Success" : "Failure");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// 설정 함수
void setup() {
  Serial.begin(115200);
  pinMode(soundSensor, INPUT_PULLUP);  // 사운드 센서 핀 설정 (풀업 저항 사용)
  pinMode(relayPin, OUTPUT);           // 릴레이 핀 출력 설정
  digitalWrite(relayPin, LOW);         // 초기 조명 상태: 꺼짐

  setup_wifi();                        // Wi-Fi 연결
  client.setServer(mqtt_server, mqtt_port);  // MQTT 서버 및 포트 설정
  client.setCallback(callback);        // MQTT 콜백 함수 설정

  attachInterrupt(digitalPinToInterrupt(soundSensor), detectClap, RISING);  // 박수 감지 인터럽트 설정
}

// 메인 루프 함수
void loop() {
  if (!client.connected()) {
    reconnect();  // MQTT 연결이 끊어졌다면 재연결
  }
  client.loop();  // MQTT 클라이언트 루프 실행

  if (clapDetected) {
    clapDetected = false;  // 플래그 초기화

    unsigned long currentMillis = millis();
    if (currentMillis - lastClapTime <= clapInterval) {
      clapCount++;
    } else {
      clapCount = 1;
    }
    lastClapTime = currentMillis;

    Serial.print("Clap detected! Clap count: ");
    Serial.println(clapCount);

    if (clapCount == 3) {
      lightState = !lightState;
      digitalWrite(relayPin, lightState ? HIGH : LOW);
      bool publishResult = client.publish(mqtt_topic_status, lightState ? "ON" : "OFF");
      Serial.print("Light state changed by clap: ");
      Serial.println(lightState ? "ON" : "OFF");
      Serial.print("Publish result: ");
      Serial.println(publishResult ? "Success" : "Failure");
      clapCount = 0;
    }
  }