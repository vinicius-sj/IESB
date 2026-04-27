// ═══════════════════════════════════════════════════
//  Robô autônomo — Arduino Uno R3
//  HC-SR04 + Servo SG90 + L298N com controle de velocidade
//
//## Como compilar e enviar para a placa
//
// 1. Instale a Arduino IDE
// 2. Abra o arquivo `robo_autonomo.ino`
// 3. Instale a biblioteca `Servo.h`
// 4. Selecione a placa: Arduino Uno
// 5. Selecione a porta COM correta
// 6. Clique em Upload

// ═══════════════════════════════════════════════════

#include <Servo.h>

// ── Pinos ───────────────────────────────────────────
#define TRIG      2
#define ECHO      3
#define SERVO_PIN 4

#define ENA  5    // PWM — velocidade motor A (tirar jumper, ligar aqui)
#define ENB  6    // PWM — velocidade motor B (tirar jumper, ligar aqui)

#define IN1  7
#define IN2  8
#define IN3  9
#define IN4  10

#define LED  13

// ── Velocidade ──────────────────────────────────────
// Ajuste aqui: 0 = parado, 255 = máximo
#define VELOCIDADE       130   // velocidade normal
#define VELOCIDADE_GIRO  120   // um pouco menor no giro para mais controle

// ── Constantes ──────────────────────────────────────
#define DIST_MINIMA     28    // cm — distância que considera obstáculo
#define ESPERA_PARADA  150    // ms — aguarda vibração dos motores sumir
#define ESPERA_SERVO   400    // ms — tempo para o servo chegar na posição
#define TEMPO_DESVIO   500    // ms — duração da manobra de desvio

Servo meuServo;

// ════════════════════════════════════════════════════
//  FUNÇÕES DE MOVIMENTO
// ════════════════════════════════════════════════════

void avancar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);
  Serial.println(">> Avançando");
}

void parar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println(">> Motores parados");
}

void girarDireita() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
  Serial.println(">> Girando à direita");
}

void girarEsquerda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
  Serial.println(">> Girando à esquerda");
}

void recuar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);
  Serial.println(">> Recuando");
}

// ════════════════════════════════════════════════════
//  SENSOR ULTRASSÔNICO
// ════════════════════════════════════════════════════

float lerDistancia() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duracao = pulseIn(ECHO, HIGH, 30000); // timeout 30ms (~5m)

  if (duracao == 0) return 999; // sem eco = caminho livre

  float distancia = duracao * 0.034 / 2.0;
  return distancia;
}

// ════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);
  Serial.println("=== Robô iniciando ===");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(LED, OUTPUT);

  // servo centraliza no início
  meuServo.attach(SERVO_PIN);
  meuServo.write(90);
  delay(600);

  Serial.println("=== Pronto para andar ===");
}

// ════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ════════════════════════════════════════════════════

void loop() {

  // ── Passo 1: lê distância à frente ─────────────────
  float distFrente = lerDistancia();
  Serial.print("Distância frente: ");
  Serial.print(distFrente);
  Serial.println(" cm");

  // ── Passo 2: caminho livre → avança ────────────────
  if (distFrente > DIST_MINIMA) {
    digitalWrite(LED, LOW);
    avancar();
    delay(80);
    return;
  }

  // ── Passo 3: obstáculo detectado ───────────────────
  Serial.println("! Obstáculo detectado !");
  digitalWrite(LED, HIGH);
  parar();
  delay(ESPERA_PARADA);

  // ── Passo 4: servo varre para a esquerda (0°) ──────
  meuServo.write(0);
  delay(ESPERA_SERVO);
  float distEsquerda = lerDistancia();
  Serial.print("Distância esquerda: ");
  Serial.print(distEsquerda);
  Serial.println(" cm");

  // ── Passo 5: servo varre para a direita (180°) ─────
  meuServo.write(180);
  delay(ESPERA_SERVO);
  float distDireita = lerDistancia();
  Serial.print("Distância direita: ");
  Serial.print(distDireita);
  Serial.println(" cm");

  // ── Passo 6: servo volta ao centro ─────────────────
  meuServo.write(90);
  delay(ESPERA_SERVO);

  // ── Passo 7: decide a direção ───────────────────────
  if (distEsquerda <= DIST_MINIMA && distDireita <= DIST_MINIMA) {
    Serial.println("! Ambiente bloqueado — recuando !");
    recuar();
    delay(TEMPO_DESVIO);
    parar();

  } else if (distEsquerda > distDireita) {
    girarEsquerda();
    delay(TEMPO_DESVIO);
    parar();

  } else {
    girarDireita();
    delay(TEMPO_DESVIO);
    parar();
  }

  delay(100);
}
