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
#define TRIG      2   // envia pulso ultrassônico (OUTPUT)
#define ECHO      3   // recebe o eco refletido   (INPUT)
// ── Pino do Servo Motor SG90 ────────────────────────
// Responsável por girar o sensor para esquerda e direita
#define SERVO_PIN 4   // sinal PWM do servo SG90

// ── Pinos de velocidade da Ponte H L298N (PWM) ──────
// ENA controla a velocidade do motor A (roda esquerda)
// ENB controla a velocidade do motor B (roda direita)
// IMPORTANTE: retire os jumpers de ENA e ENB na placa L298N
//             e conecte os fios nesses pinos para o PWM funcionar
#define ENA       5   // habilita e controla velocidade do motor A (PWM)
#define ENB       6   // habilita e controla velocidade do motor B (PWM)
// ── Pinos de direção da Ponte H L298N ───────────────
// IN1/IN2 controlam o sentido de rotação do motor A
// IN3/IN4 controlam o sentido de rotação do motor B
// Lógica: IN1=LOW / IN2=HIGH → motor A gira para frente
//         IN1=HIGH / IN2=LOW → motor A gira para trás
#define IN1       7   // direção motor A — fio 1
#define IN2       8   // direção motor A — fio 2
#define IN3       9   // direção motor B — fio 1
#define IN4      10   // direção motor B — fio 2
// ── LED de status ───────────────────────────────────
// Acende quando um obstáculo é detectado
#define LED      13   // LED de alerta: acende quando detecta obstáculo

// ── Configurações de velocidade ─────────────────────
// Valores entre 0 (parado) e 255 (velocidade máxima)
// Ajuste conforme o peso e as rodas do seu robô
#define VELOCIDADE        130   // Velocidade normal de avanço
#define VELOCIDADE_GIRO   120   // Velocidade de giro (menor = mais preciso)

// ── Parâmetros de comportamento ───────────────────────────────────
#define DIST_MINIMA    28    // Distância em cm para considerar obstáculo
#define ESPERA_PARADA 150    // Tempo (ms) para vibração dos motores sumir após parar
#define ESPERA_SERVO  400    // Tempo (ms) para o servo chegar na posição desejada
#define TEMPO_DESVIO  500    // Duração (ms) da manobra de desvio (giro ou recuo)

// ── Variáveis de instrumentação — frequência ─────────────────────
// Estratégia: medir o tempo de INÍCIO a INÍCIO de cada iteração AVANÇAR
// Isso captura o ciclo real completo incluindo o delay(80) ao final
static unsigned long ultimoInicio   = 0;  // millis() no início da última iteração
unsigned long        somaIntervalo  = 0;  // acumula intervalos para calcular média
int                  contadorFreq   = 0;  // conta iterações AVANÇAR acumuladas
bool                 mediaConcluida = false; // só calcula uma vez por sessão

// ── Variáveis de instrumentação — tempo de resposta ──────────────
// Dois tempos são medidos separadamente:
// 1. Tempo real: detecção → parar()         (deve ser <= 150 ms)
// 2. Ciclo completo: detecção → 1ª manobra  (inclui varredura, por design)
unsigned long tempoDeteccao = 0;

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
  digitalWrite(IN2, HIGH);// Motor A → frente
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);// Motor B → frente
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);

}

// Para os dois motores: zera PWM antes de mudar os pinos
// (evita corrente de frenagem brusca no driver L298N)
void parar() {
  analogWrite(ENA, 0);  // Desliga PWM do motor A
  analogWrite(ENB, 0);  // Desliga PWM do motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}

// Gira à direita: motor A avança, motor B recua
// Isso faz o robô girar no próprio eixo para a direita
void girarDireita() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A → trás
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B → frente
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);

}

// Gira à esquerda: motor B avança, motor A recua
// Isso faz o robô girar no próprio eixo para a esquerda
void girarEsquerda() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDADE_GIRO);
  analogWrite(ENB, VELOCIDADE_GIRO);

}

// Recua: ambos os motores giram para trás
// Usado quando o robô está bloqueado dos dois lados
void recuar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDADE);
  analogWrite(ENB, VELOCIDADE);

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
  long duracao = pulseIn(ECHO, HIGH, 30000); // timeout 30 ms ≈ 5 m

    // Se não houve eco, considera caminho livre (999 cm)
  if (duracao == 0) return 999;

    // Converte o tempo para distância em centímetros

  return duracao * 0.034 / 2.0;
}


// ════════════════════════════════════════════════════
//  SETUP — Executado uma única vez ao ligar o Arduino
// ════════════════════════════════════════════════════

void setup() {
  // Inicia a comunicação serial para monitoramento no PC
  Serial.begin(9600);
  Serial.println("============================================");
  Serial.println("  Robo autonomo — PC3");
  Serial.println("  Instrumentacao: frequencia + tempo resp.");
  Serial.println("============================================");
  // Configura pinos da ponte H como saída
  pinMode(IN1, OUTPUT); 
  
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);

    // Configura pinos do sensor ultrassônico
  pinMode(TRIG, OUTPUT); // TRIG envia sinal
  pinMode(ECHO, INPUT);// ECHO recebe sinal

   // Configura o LED de status
  pinMode(LED, OUTPUT);

  // Centraliza o servo em 90° antes de iniciar
  // Garante que o sensor aponta para frente no início
  meuServo.attach(SERVO_PIN);
  meuServo.write(90);
  delay(600);// Aguarda o servo chegar à posição central

  Serial.println("  Servo centralizado. Pronto para andar.");
  Serial.println("--------------------------------------------");
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




  // ── Passo 2: caminho livre → continua avançando ────
  if (distFrente > DIST_MINIMA) {
    digitalWrite(LED, LOW);// Apaga LED (sem obstáculo)
    avancar();

    // ── Instrumentação de frequência ──────────────────────────
    // Mede o intervalo de INÍCIO a INÍCIO de iterações consecutivas
    // para capturar o ciclo real completo, incluindo o delay(80).
    if (!mediaConcluida) {
      unsigned long agora = millis();

      if (ultimoInicio > 0) {
        unsigned long ciclo = agora - ultimoInicio;
        somaIntervalo += ciclo;
        contadorFreq++;

        if (contadorFreq == 10) {
          float mediaMs = (float)somaIntervalo / 10.0;
          float mediaHz = 1000.0 / mediaMs;

          Serial.println("============================================");
          Serial.println("  [FREQ] Resultado — 10 ciclos AVANCAR:");
          Serial.print  ("  Intervalo medio : ");
          Serial.print  (mediaMs, 1);
          Serial.println(" ms");
          Serial.print  ("  Frequencia media: ");
          Serial.print  (mediaHz, 1);
          Serial.println(" Hz");
          Serial.print  ("  Requisito >= 20 Hz → ");
          Serial.println(mediaHz >= 20.0 ? "APROVADO" : "REPROVADO");
          if (mediaHz < 20.0) {
            Serial.println("  Nota: delay(80) inserido para estabilidade");
            Serial.println("  do sensor. Remover delay reduz o ciclo.");
          }
          Serial.println("============================================");
          mediaConcluida = true;
        }
      }
      ultimoInicio = agora; // atualiza ANTES do delay(80)
    }

    Serial.print("Frente: ");
    Serial.print(distFrente, 1);
    Serial.println(" cm — livre, avancando");

    delay(80);// Pequena pausa antes de medir de novo
    return;// Volta ao início do loop
  }

  // ── Passo 3: obstáculo detectado ──────────────────────────────

  // Tempo de resposta real: do instante da detecção até parar()
  // Este é o tempo que o sistema leva para reagir ao evento —
  // deve ser <= 150 ms conforme requisito do relatório.
  tempoDeteccao = millis();

  Serial.println("--------------------------------------------");
  Serial.print  ("! Obstaculo a ");
  Serial.print  (distFrente, 1);
  Serial.println(" cm — parando motores");

  digitalWrite(LED, HIGH);// Acende LED de alerta
  parar(); // ← PRIMEIRO COMANDO de atuação após detecção

  // Mede e imprime o tempo real de resposta (detecção → parar)
  unsigned long trReal = millis() - tempoDeteccao;
  Serial.print  ("  [RESP real] Deteccao -> parar(): ");
  Serial.print  (trReal);
  Serial.println(" ms");
  Serial.print  ("  Requisito <= 150 ms → ");
  Serial.println(trReal <= 150 ? "APROVADO" : "REPROVADO");

  delay(ESPERA_PARADA);  // Aguarda estabilização mecânica

  // ── Passo 4: servo varre para a esquerda (0°) ─────────────────
  Serial.println("  Servo → esquerda (0 graus)");
  meuServo.write(0);
  delay(ESPERA_SERVO);// Aguarda servo chegar em 0°
  float distEsquerda = lerDistancia();
  Serial.print  ("  Distancia esquerda: ");
  Serial.print  (distEsquerda, 1);
  Serial.println(" cm");

  // ── Passo 5: servo varre para a direita (180°) ────────────────
  Serial.println("  Servo → direita (180 graus)");
  meuServo.write(180);
  delay(ESPERA_SERVO);// Aguarda servo chegar em 180°
  float distDireita = lerDistancia();
  Serial.print  ("  Distancia direita: ");
  Serial.print  (distDireita, 1);
  Serial.println(" cm");

  // ── Passo 6: servo volta ao centro (90°) ────────────────────────────
  Serial.println("  Servo → centro (90 graus)");
  meuServo.write(90);
  delay(ESPERA_SERVO);

  // ── Ciclo completo: detecção → início da manobra ──────────────
  // Este tempo INCLUI ESPERA_PARADA + varredura do servo (por design).
  // É uma métrica de ciclo operacional, não de tempo de resposta.
  unsigned long cicloCompleto = millis() - tempoDeteccao;
  Serial.print  ("  [CICLO] Deteccao → manobra: ");
  Serial.print  (cicloCompleto);
  Serial.println(" ms (inclui parada + varredura servo — esperado ~1350 ms)");

    // ── Passo 7: decide a direção com base nas medições ─
  // Caso 1: bloqueado dos dois lados → recua
  if (distEsquerda <= DIST_MINIMA && distDireita <= DIST_MINIMA) {
    Serial.println("  Ambos os lados bloqueados — recuando");
    recuar();
    delay(TEMPO_DESVIO);
    parar();

  // Caso 2: esquerda tem mais espaço → gira à esquerda
  } else if (distEsquerda > distDireita) {
    Serial.print  ("  Esquerda mais livre (");
    Serial.print  (distEsquerda, 1);
    Serial.println(" cm) — girando esquerda");
    girarEsquerda();
    delay(TEMPO_DESVIO);
    parar();

  // Caso 3: direita tem mais espaço (ou igual) → gira à direita
  } else {
    Serial.print  ("  Direita mais livre (");
    Serial.print  (distDireita, 1);
    Serial.println(" cm) — girando direita");
    girarDireita();
    delay(TEMPO_DESVIO);
    parar();
  }

// Pequena pausa antes do próximo ciclo de leitura
  Serial.println("  Manobra concluida — retomando avanco");
  Serial.println("--------------------------------------------");
  delay(100);
}
