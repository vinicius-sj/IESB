// ═══════════════════════════════════════════════════
//  Robô autônomo — Arduino Uno R3
//  HC-SR04 + Servo SG90 + L298N com controle de velocidade
//
//  Descrição:
//  Este firmware implementa um robô móvel autônomo capaz de
//  detectar obstáculos à frente usando um sensor ultrassônico
//  montado em um servo motor. Ao detectar um obstáculo, o robô
//  para, varre o ambiente para esquerda e direita, e escolhe
//  o lado com mais espaço para desviar.
//
//  Fluxo principal:
//  1. Lê a distância à frente
//  2. Se livre → avança
//  3. Se obstáculo → para, varre esquerda e direita
//  4. Escolhe o lado mais livre e gira
//  5. Se bloqueado dos dois lados → recua
//
//  Como compilar e enviar para a placa:
//  1. Instale a Arduino IDE (https://www.arduino.cc/en/software)
//  2. Abra o arquivo robo_autonomo.ino
//  3. Instale a biblioteca Servo:
//     Sketch → Incluir Biblioteca → Gerenciar Bibliotecas → "Servo"
//  4. Selecione a placa: Ferramentas → Placa → Arduino Uno
//  5. Selecione a porta: Ferramentas → Porta → porta COM correta
//  6. Clique em Upload (seta →)
// ═══════════════════════════════════════════════════

#include <Servo.h>  // Biblioteca para controle do servo motor

// ── Pinos do Sensor Ultrassônico HC-SR04 ────────────
// TRIG: envia o pulso sonoro
// ECHO: recebe o eco e mede o tempo de retorno
#define TRIG      2
#define ECHO      3

// ── Pino do Servo Motor SG90 ────────────────────────
// Responsável por girar o sensor para esquerda e direita
#define SERVO_PIN 4

// ── Pinos de velocidade da Ponte H L298N (PWM) ──────
// ENA controla a velocidade do motor A (roda esquerda)
// ENB controla a velocidade do motor B (roda direita)
// IMPORTANTE: retire os jumpers de ENA e ENB na placa L298N
//             e conecte os fios nesses pinos para o PWM funcionar
#define ENA  5
#define ENB  6

// ── Pinos de direção da Ponte H L298N ───────────────
// IN1/IN2 controlam o sentido de rotação do motor A
// IN3/IN4 controlam o sentido de rotação do motor B
// Lógica: IN1=LOW / IN2=HIGH → motor A gira para frente
//         IN1=HIGH / IN2=LOW → motor A gira para trás
#define IN1  7
#define IN2  8
#define IN3  9
#define IN4  10

// ── LED de status ───────────────────────────────────
// Acende quando um obstáculo é detectado
#define LED  13

// ── Configurações de velocidade ─────────────────────
// Valores entre 0 (parado) e 255 (velocidade máxima)
// Ajuste conforme o peso e as rodas do seu robô
#define VELOCIDADE       130   // Velocidade normal de avanço
#define VELOCIDADE_GIRO  120   // Velocidade de giro (menor = mais preciso)

// ── Configurações de comportamento ──────────────────
#define DIST_MINIMA     28    // Distância em cm para considerar obstáculo
#define ESPERA_PARADA  150    // Tempo (ms) para vibração dos motores sumir após parar
#define ESPERA_SERVO   400    // Tempo (ms) para o servo chegar na posição desejada
#define TEMPO_DESVIO   500    // Duração (ms) da manobra de desvio (giro ou recuo)

// Objeto global do servo motor
Servo meuServo;


// ════════════════════════════════════════════════════
//  FUNÇÕES DE MOVIMENTO
//  Cada função configura os pinos IN1~IN4 para definir
//  o sentido dos motores e aplica PWM via ENA/ENB
//  para controlar a velocidade.
// ════════════════════════════════════════════════════

// Avança: ambos os motores giram para frente
void avancar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);   // Motor A → frente
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);   // Motor B → frente
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);
  Serial.println(">> Avançando");
}

// Para: corta a alimentação dos dois motores
void parar() {
  analogWrite(ENA, 0);        // Desliga PWM do motor A
  analogWrite(ENB, 0);        // Desliga PWM do motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println(">> Motores parados");
}

// Gira à direita: motor A recua, motor B avança
// Isso faz o robô girar no próprio eixo para a direita
void girarDireita() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);    // Motor A → trás
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);   // Motor B → frente
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
  Serial.println(">> Girando à direita");
}

// Gira à esquerda: motor A avança, motor B recua
// Isso faz o robô girar no próprio eixo para a esquerda
void girarEsquerda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);   // Motor A → frente
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);    // Motor B → trás
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);
  Serial.println(">> Girando à esquerda");
}

// Recua: ambos os motores giram para trás
// Usado quando o robô está bloqueado dos dois lados
void recuar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);   // Motor A → trás
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);   // Motor B → trás
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);
  Serial.println(">> Recuando");
}


// ════════════════════════════════════════════════════
//  SENSOR ULTRASSÔNICO
//  Princípio de funcionamento:
//  O pino TRIG envia um pulso de 10µs que gera uma onda
//  sonora. O pino ECHO fica em HIGH pelo tempo que a onda
//  leva para ir e voltar. A distância é calculada por:
//
//  distância (cm) = tempo_echo (µs) × velocidade_som / 2
//  velocidade do som ≈ 0,034 cm/µs
//  divide por 2 pois o som faz o caminho de ida e volta
// ════════════════════════════════════════════════════

float lerDistancia() {
  // Garante que o TRIG começa em LOW antes do pulso
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // Envia o pulso de 10 microssegundos para disparar a medição
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Lê o tempo de retorno do eco (timeout de 30ms = ~5 metros)
  // Se não houver eco dentro do timeout, retorna 0
  long duracao = pulseIn(ECHO, HIGH, 30000);

  // Se não houve eco, considera caminho livre (999 cm)
  if (duracao == 0) return 999;

  // Converte o tempo para distância em centímetros
  float distancia = duracao * 0.034 / 2.0;
  return distancia;
}


// ════════════════════════════════════════════════════
//  SETUP — Executado uma única vez ao ligar o Arduino
// ════════════════════════════════════════════════════

void setup() {
  // Inicia a comunicação serial para monitoramento no PC
  Serial.begin(9600);
  Serial.println("=== Robô iniciando ===");

  // Configura pinos da ponte H como saída
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configura pinos do sensor ultrassônico
  pinMode(TRIG, OUTPUT);  // TRIG envia sinal
  pinMode(ECHO, INPUT);   // ECHO recebe sinal

  // Configura o LED de status
  pinMode(LED, OUTPUT);

  // Centraliza o servo em 90° antes de iniciar
  // Garante que o sensor aponta para frente no início
  meuServo.attach(SERVO_PIN);
  meuServo.write(90);
  delay(600);  // Aguarda o servo chegar à posição central

  Serial.println("=== Pronto para andar ===");
}


// ════════════════════════════════════════════════════
//  LOOP PRINCIPAL — Repetido continuamente
//
//  Máquina de estados simples:
//  [Avançando] → obstáculo? → [Parado] → varre lados
//       ↑                                     ↓
//       └──── esquerda ou direita livres ──────┘
//                                     ↓
//                              bloqueado dos dois lados
//                                     ↓
//                                 [Recuando]
// ════════════════════════════════════════════════════

void loop() {

  // ── Passo 1: mede a distância à frente ─────────────
  float distFrente = lerDistancia();
  Serial.print("Distância frente: ");
  Serial.print(distFrente);
  Serial.println(" cm");

  // ── Passo 2: caminho livre → continua avançando ────
  if (distFrente > DIST_MINIMA) {
    digitalWrite(LED, LOW);   // Apaga LED (sem obstáculo)
    avancar();
    delay(80);                // Pequena pausa antes de medir de novo
    return;                   // Volta ao início do loop
  }

  // ── Passo 3: obstáculo detectado → para o robô ─────
  Serial.println("! Obstáculo detectado !");
  digitalWrite(LED, HIGH);   // Acende LED de alerta
  parar();
  delay(ESPERA_PARADA);      // Aguarda estabilização mecânica

  // ── Passo 4: servo varre para a esquerda (0°) ──────
  meuServo.write(0);
  delay(ESPERA_SERVO);                    // Aguarda servo chegar em 0°
  float distEsquerda = lerDistancia();
  Serial.print("Distância esquerda: ");
  Serial.print(distEsquerda);
  Serial.println(" cm");

  // ── Passo 5: servo varre para a direita (180°) ─────
  meuServo.write(180);
  delay(ESPERA_SERVO);                    // Aguarda servo chegar em 180°
  float distDireita = lerDistancia();
  Serial.print("Distância direita: ");
  Serial.print(distDireita);
  Serial.println(" cm");

  // ── Passo 6: servo volta ao centro (90°) ───────────
  meuServo.write(90);
  delay(ESPERA_SERVO);

  // ── Passo 7: decide a direção com base nas medições ─
  // Caso 1: bloqueado dos dois lados → recua
  if (distEsquerda <= DIST_MINIMA && distDireita <= DIST_MINIMA) {
    Serial.println("! Ambiente bloqueado — recuando !");
    recuar();
    delay(TEMPO_DESVIO);
    parar();

  // Caso 2: esquerda tem mais espaço → gira à esquerda
  } else if (distEsquerda > distDireita) {
    girarEsquerda();
    delay(TEMPO_DESVIO);
    parar();

  // Caso 3: direita tem mais espaço (ou igual) → gira à direita
  } else {
    girarDireita();
    delay(TEMPO_DESVIO);
    parar();
  }

  // Pequena pausa antes do próximo ciclo de leitura
  delay(100);
}
