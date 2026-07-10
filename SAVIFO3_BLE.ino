// =============================================================
//  SAVIFO3 — ESP32-S2  |  BLE + Sensores + Servo + Motores
//  Compatível com App Inventor (LENG2_corrigido_final.aia)
//
//  UUIDs BLE (têm de ser iguais aos da app):
//    SERVICE      : 12345678-1234-1234-1234-123456789abc
//    SENSOR_CHAR  : 12345678-1234-1234-1234-123456789ab1  (notify ESP32→App)
//    MOTOR_CHAR   : 12345678-1234-1234-1234-123456789ab2  (write  App→ESP32)
//    MEASURE_CHAR : 12345678-1234-1234-1234-123456789ab3  (notify ESP32→App)
//    SERVO_CHAR   : 12345678-1234-1234-1234-123456789ab5  (write  App→ESP32)
//
//  Comandos recebidos da App:
//    Motores : "M,<dir>,<vel>"
//              dir =  1 frente | -1 trás | 0 parar/virar
//              vel =  0-255    |  1 virar direita | -1 virar esq.
//    Servo   : "S,<ângulo>"  (0-180)
//              S,90 neutro | S,135 subir | S,45 descer
//    Registar: "R"  → envia medição atual para pontos A/B/C
//
//  Dados enviados para a App (SENSOR_CHAR, a cada 1 s):
//    "temp,humidade,distancia,ir,irmax,tempoVoo\n"
//
//  Dados de medição (MEASURE_CHAR, quando se carrega R na app):
//    "MEAS:temp,ur,ir,dist,hora\n"
// =============================================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "DHT.h"
#include <ESP32Servo.h>

// ---------- UUIDs BLE ----------
#define SERVICE_UUID   "12345678-1234-1234-1234-123456789abc"
#define SENSOR_UUID    "12345678-1234-1234-1234-123456789ab1"
#define MOTOR_UUID     "12345678-1234-1234-1234-123456789ab2"
#define MEASURE_UUID   "12345678-1234-1234-1234-123456789ab3"
#define SERVO_UUID     "12345678-1234-1234-1234-123456789ab5"

// ---------- DHT22 ----------
#define DHTPIN   4
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);

// ---------- HC-SR04 ----------
#define TRIG_PIN 5
#define ECHO_PIN 18

// ---------- KY-026 ----------
#define IR_ANALOGICO 34
#define IR_DIGITAL   35

// ---------- SERVO ----------
#define SERVO_PIN 13
Servo servoMotor;
int anguloServo = 90;

// ---------- L298N / MOTORES ----------
#define IN1 26
#define IN2 27
#define ENA 25
#define IN3 14
#define IN4 32
#define ENB 33
int velocidadeMotor1 = 0;
int velocidadeMotor2 = 0;

// ---------- BLE ----------
BLEServer*         pServer       = nullptr;
BLECharacteristic* pSensorChar   = nullptr;
BLECharacteristic* pMeasureChar  = nullptr;
bool deviceConnected = false;
bool oldConnected    = false;

// ---------- ESTADO ----------
float irMax = 0;
unsigned long tempoInicio   = 0;
unsigned long ultimaLeitura = 0;
const unsigned long INTERVALO = 1000;

// =====================================================================
//  DECLARAÇÕES ANTECIPADAS (forward declarations)
// =====================================================================
void atualizarMotores();
void enviarMedicaoAtual();
float medirDistancia();

// =====================================================================
//  CALLBACKS BLE
// =====================================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    tempoInicio = millis();
    Serial.println("[BLE] App ligada");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("[BLE] App desligada");
  }
};

class MotorCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    Serial.print("[MOTOR CMD] "); Serial.println(cmd);

    // Formato: "M,<dir>,<vel>"
    if (cmd.startsWith("M,")) {
      String params = cmd.substring(2);
      int comma = params.indexOf(',');
      if (comma < 0) return;

      int dir = params.substring(0, comma).toInt();
      int vel = params.substring(comma + 1).toInt();

      if (dir == 1) {
        // Frente
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        velocidadeMotor1 = vel;
        velocidadeMotor2 = vel;
      } else if (dir == -1) {
        // Trás
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        velocidadeMotor1 = vel;
        velocidadeMotor2 = vel;
      } else if (dir == 0 && vel == 0) {
        // Parar
        velocidadeMotor1 = 0;
        velocidadeMotor2 = 0;
      } else if (dir == 0 && vel == 1) {
        // Virar direita
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
        velocidadeMotor1 = 150;
        velocidadeMotor2 = 150;
      } else if (dir == 0 && vel == -1) {
        // Virar esquerda
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
        velocidadeMotor1 = 150;
        velocidadeMotor2 = 150;
      }

      atualizarMotores();
    }
  }
};

class ServoCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = String(pChar->getValue().c_str());
    cmd.trim();
    Serial.print("[SERVO CMD] "); Serial.println(cmd);

    // Formato: "S,<ângulo>"
    if (cmd.startsWith("S,")) {
      int angulo = cmd.substring(2).toInt();
      if (angulo >= 0 && angulo <= 180) {
        anguloServo = angulo;
        servoMotor.write(anguloServo);
        Serial.print("Servo → "); Serial.print(anguloServo); Serial.println("°");
      }
    }

    // "R" → registar posição (envia medição atual)
    if (cmd == "R") {
      enviarMedicaoAtual();
    }
  }
};

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- Sensores ----
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_ANALOGICO, INPUT);
  pinMode(IR_DIGITAL,   INPUT);

  // ---- Servo ----
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(anguloServo);

  // ---- Motores ----
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  atualizarMotores();

  // ---- BLE ----
  BLEDevice::init("SAVIFO3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // SENSOR (notify)
  pSensorChar = pService->createCharacteristic(
    SENSOR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pSensorChar->addDescriptor(new BLE2902());

  // MOTOR (write)
  BLECharacteristic* pMotorChar = pService->createCharacteristic(
    MOTOR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pMotorChar->setCallbacks(new MotorCallbacks());

  // MEASURE (notify)
  pMeasureChar = pService->createCharacteristic(
    MEASURE_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pMeasureChar->addDescriptor(new BLE2902());

  // SERVO (write)
  BLECharacteristic* pServoChar = pService->createCharacteristic(
    SERVO_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pServoChar->setCallbacks(new ServoCallbacks());

  pService->start();

  BLEAdvertising* pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.println("SAVIFO3 BLE iniciado — à espera da app...");
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  unsigned long agora = millis();

  if (deviceConnected && (agora - ultimaLeitura >= INTERVALO)) {
    ultimaLeitura = agora;
    enviarSensores();
  }

  // Reconectar após desconexão
  if (!deviceConnected && oldConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] A fazer advertising novamente...");
    oldConnected = false;
  }
  if (deviceConnected && !oldConnected) {
    oldConnected = true;
  }
}

// =====================================================================
//  FUNÇÕES
// =====================================================================

void enviarSensores() {
  float temperatura = dht.readTemperature();
  float humidade    = dht.readHumidity();
  float distancia   = medirDistancia();
  int   valorIR     = analogRead(IR_ANALOGICO);

  if (isnan(temperatura)) temperatura = 0.0;
  if (isnan(humidade))    humidade    = 0.0;
  if (distancia < 0)      distancia   = 0.0;

  if (valorIR > irMax) irMax = valorIR;

  unsigned long tempoVoo = (millis() - tempoInicio) / 1000;

  // Formato: "temp,humidade,distancia,ir,irmax,tempoVoo"
  String dados = String(temperatura, 1) + "," +
                 String(humidade, 1)    + "," +
                 String(distancia, 1)   + "," +
                 String(valorIR)        + "," +
                 String((int)irMax)     + "," +
                 String(tempoVoo);

  pSensorChar->setValue(dados.c_str());
  pSensorChar->notify();

  Serial.print("→ App: "); Serial.println(dados);
}

void enviarMedicaoAtual() {
  float temperatura = dht.readTemperature();
  float humidade    = dht.readHumidity();
  float distancia   = medirDistancia();
  int   valorIR     = analogRead(IR_ANALOGICO);
  unsigned long tempoVoo = (millis() - tempoInicio) / 1000;

  if (isnan(temperatura)) temperatura = 0.0;
  if (isnan(humidade))    humidade    = 0.0;
  if (distancia < 0)      distancia   = 0.0;

  // Formato: "MEAS:temp,ur,ir,dist,hora"
  String med = "MEAS:" +
               String(temperatura, 1) + "," +
               String(humidade, 1)    + "," +
               String(valorIR)        + "," +
               String(distancia, 1)   + "," +
               String(tempoVoo);

  pMeasureChar->setValue(med.c_str());
  pMeasureChar->notify();

  Serial.print("[MEDIÇÃO] "); Serial.println(med);
}

void atualizarMotores() {
  velocidadeMotor1 = constrain(velocidadeMotor1, 0, 255);
  velocidadeMotor2 = constrain(velocidadeMotor2, 0, 255);
  ledcWrite(ENA, velocidadeMotor1);
  ledcWrite(ENB, velocidadeMotor2);
  Serial.print("Motor1: "); Serial.print(velocidadeMotor1);
  Serial.print(" | Motor2: "); Serial.println(velocidadeMotor2);
}

float medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracao = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracao == 0) return -1.0;
  return duracao * 0.0343 / 2.0;
}
