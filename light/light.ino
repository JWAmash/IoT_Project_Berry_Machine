#include <WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <ArduinoJson.h>

// WiFi 설정
const char* ssid = "KT_GiGA_0BF0";          // Wi-Fi SSID
const char* password = "bke72cg443";  // Wi-Fi 비밀번호

// MQTT 브로커 설정
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttClientID = "berryblind";
const char* controlTopic = "blinds/control";
const char* settingsTopic = "blinds/settings";

// WiFi 및 MQTT 클라이언트 객체 생성
WiFiClient espClient;
PubSubClient client(espClient);

// 서보 모터 설정
Servo continuousServo;
const int servoPin = 9;  // 서보 모터 핀 번호

// 핀 설정
const int trigPin = 10;   // 초음파 센서 Trig 핀
const int echoPin = 11;   // 초음파 센서 Echo 핀
const int ldrPin = A0;    // 조도 센서 핀

// 변수 및 임계값 설정
int lightThresholdDark = 800;  // 어두울 때의 조도 임계값
int lightThresholdBright = 500;  // 밝을 때의 조도 임계값
bool autoMode = false;  // 자동 모드 플래그

float minDistance = 5.0;   // 최소 거리 (센티미터)
float maxDistance = 40.0;  // 최대 거리 (센티미터)

// 모터 상태 변수
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN };
MotorState motorState = STOPPED;

// 거리 측정값 저장을 위한 배열 및 변수
const int numReadings = 5;
float distanceReadings[numReadings];
int readIndex = 0;
float totalDistance = 0;
float averageDistance = 0;

void setup() {
  Serial.begin(115200);

  // WiFi 연결
  setup_wifi();

  // MQTT 설정
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  // 서보 모터 및 핀 설정
  continuousServo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  stopMotor();

  // 거리 측정값 배열 초기화
  for (int i = 0; i < numReadings; i++) {
    distanceReadings[i] = 0;
  }

  Serial.println("시리얼 명령어: 'U' - 올리기, 'D' - 내리기, 'S' - 정지, 'AF' - 자동모드 켜기, 'AN' - 자동모드 끄기");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 자동 모드에서만 handleAutoMode 실행
  if (autoMode) {
    handleAutoMode();
  }

  updateMotor();

  // 시리얼 모니터 명령어 처리
  processSerialCommand();
}

// WiFi 연결 함수
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi 연결 성공");
    Serial.print("IP 주소: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi 연결 실패");
  }
}

// MQTT 재연결 함수
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT 연결 시도...");
    if (client.connect(mqttClientID)) {
      Serial.println("연결됨");
      client.subscribe(controlTopic);
      client.subscribe(settingsTopic);
    } else {
      Serial.print("실패, rc=");
      Serial.print(client.state());
      Serial.println(" 5초 후 재시도");
      delay(5000);
    }
  }
}

// 시리얼 명령어 처리 함수
void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "U") {
      moveBlindUp();
    } else if (command == "D") {
      moveBlindDown();
    } else if (command == "S") {
      stopMotor();
    } else if (command == "AF") {
      autoMode = true;
      Serial.println("자동 모드 활성화");
    } else if (command == "AN") {
      autoMode = false;
      stopMotor();
      Serial.println("자동 모드 비활성화");
    } else {
      Serial.println("잘못된 명령어입니다.");
    }
  }
}

// MQTT 메시지 콜백 함수
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;

  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }

  Serial.print("메시지 수신 [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(messageTemp);

  if (String(topic) == controlTopic) {
    handleControlMessage(messageTemp);
  } else if (String(topic) == settingsTopic) {
    handleSettingsMessage(messageTemp);
  }
}

// 제어 메시지 처리 함수
void handleControlMessage(String cmd) {
  if (cmd == "UP_PRESS") {
    moveBlindUp();
  } else if (cmd == "UP_RELEASE") {
    stopMotor();
  } else if (cmd == "DOWN_PRESS") {
    moveBlindDown();
  } else if (cmd == "DOWN_RELEASE") {
    stopMotor();
  } else if (cmd == "FULL_UP") {
    moveBlindToPosition(maxDistance);
  } else if (cmd == "FULL_DOWN") {
    moveBlindToPosition(minDistance);
  } else if (cmd == "STOP") {
    stopMotor();
  } else if (cmd == "AUTO_ON") {
    autoMode = true;
  } else if (cmd == "AUTO_OFF") {
    autoMode = false;
    stopMotor();
  } else {
    Serial.println("알 수 없는 명령어입니다.");
  }
}

// 설정 메시지 처리 함수
void handleSettingsMessage(String json) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("deserializeJson() 실패: ");
    Serial.println(error.f_str());
    return;
  }

  if (doc.containsKey("maxDistance")) {
    maxDistance = doc["maxDistance"];
    Serial.print("maxDistance 업데이트: ");
    Serial.println(maxDistance);
  }
  if (doc.containsKey("minDistance")) {
    minDistance = doc["minDistance"];
    Serial.print("minDistance 업데이트: ");
    Serial.println(minDistance);
  }
  if (doc.containsKey("lightThresholdDark")) {
    lightThresholdDark = doc["lightThresholdDark"];
    Serial.print("lightThresholdDark 업데이트: ");
    Serial.println(lightThresholdDark);
  }
  if (doc.containsKey("lightThresholdBright")) {
    lightThresholdBright = doc["lightThresholdBright"];
    Serial.print("lightThresholdBright 업데이트: ");
    Serial.println(lightThresholdBright);
  }
}

// 평균 거리 계산 함수
float getAverageDistance() {
  float newDistance = getDistance();
  if (newDistance == -1.0) {
    return averageDistance;
  }

  totalDistance = totalDistance - distanceReadings[readIndex];
  distanceReadings[readIndex] = newDistance;
  totalDistance = totalDistance + distanceReadings[readIndex];

  readIndex = (readIndex + 1) % numReadings;

  averageDistance = totalDistance / numReadings;
  return averageDistance;
}

// 초음파 센서 거리 측정 함수
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) {
    Serial.println("센서 타임아웃");
    return -1.0;
  }
  float distance = duration * 0.034 / 2.0;

  if (distance < 2.0 || distance > 400.0) {
    Serial.println("잘못된 거리 측정값. 무시합니다.");
    return -1.0;
  }

  Serial.print("측정된 거리: ");
  Serial.println(distance);
  return distance;
}

// 모터 상태 업데이트 함수
void updateMotor() {
  if (motorState == MOVING_UP) {
    float distance = getAverageDistance();
    if (distance == -1.0) {
      Serial.println("센서 오류. 모터 정지.");
      stopMotor();
      return;
    }
    if (distance >= maxDistance) {
      Serial.println("최대 거리 도달. 모터 정지.");
      stopMotor();
    } else {
      moveMotor(0);
    }
  } else if (motorState == MOVING_DOWN) {
    float distance = getAverageDistance();
    if (distance == -1.0) {
      Serial.println("센서 오류. 모터 정지.");
      stopMotor();
      return;
    }
    if (distance <= minDistance) {
      Serial.println("최소 거리 도달. 모터 정지.");
      stopMotor();
    } else {
      moveMotor(180);
    }
  } else if (motorState == STOPPED) {
    moveMotor(90);
  }
}

// 블라인드 올리기 함수
void moveBlindUp() {
  if (motorState != MOVING_UP) {
    Serial.println("블라인드 올리기...");
    motorState = MOVING_UP;
  }
}

// 블라인드 내리기 함수
void moveBlindDown() {
  if (motorState != MOVING_DOWN) {
    Serial.println("블라인드 내리기...");
    motorState = MOVING_DOWN;
  }
}

// 블라인드를 특정 위치로 이동 함수
void moveBlindToPosition(float targetDistance) {
  float currentDistance = getAverageDistance();
  if (currentDistance == -1.0) {
    Serial.println("센서 오류. 블라인드 이동 불가.");
    return;
  }

  if (currentDistance < targetDistance) {
    moveBlindUp();
  } else if (currentDistance > targetDistance) {
    moveBlindDown();
  } else {
    stopMotor();
  }
}

// 모터 정지 함수
void stopMotor() {
  if (motorState != STOPPED) {
    Serial.println("모터 정지.");
    motorState = STOPPED;
  }
}

// 모터 제어 함수
void moveMotor(int angle) {
  continuousServo.write(angle);
}

// 자동 모드 제어 함수
void handleAutoMode() {
  static unsigned long lastCheckTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastCheckTime >= 1000) {
    lastCheckTime = currentTime;

    float distance = getAverageDistance();
    int ldrValue = analogRead(ldrPin);

    Serial.print("LDR 값: ");
    Serial.print(ldrValue);
    Serial.print(", 평균 거리: ");
    Serial.println(distance);

    if (ldrValue >= lightThresholdDark && distance < maxDistance) {
      moveBlindUp();
    } else if (ldrValue <= lightThresholdBright && distance > minDistance) {
      moveBlindDown();
    } else {
      stopMotor();
    }
  }
}


