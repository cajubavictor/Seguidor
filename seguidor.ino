#include <QTRSensors.h>
#define MOTOR_E1    6
#define MOTOR_E2    5
#define MOTOR_D1    10
#define MOTOR_D2    9
#define DEBUG         1
#define EMITTER_PIN     2
#define NUM_SENSORES     6
#define NUM_SAMPLES_PER_SENSOR   4
#define VELOCIDADE_BASE           200
#define VELOCIDADE_MAXIMA         254
int POSICAO_MEDIA = ((NUM_SENSORES - 1) * 1000) / 2;
int DISTANCIA_MEDIA = ((NUM_SENSORES - 1) * 10) / 2;
/* São aproximadamente 10 us para realizar uma única conversão analog - digital. Então, se
  NUM_SAMPLES_PER_SENSOR é de 4 e NUM_SENSORES é 6, demorará 4 * 6 * 100 us = ~2.5ms para fazer
  uma leitura completa. Aumentar esse parâmetro aumenta a quantidade de leitura ao custo de
  tempo de processamento.*/
float deltaTime = 0;
int posicao = 0;
unsigned int position = 0;
int  velocidadeMotorDireito    = 0;
int  velocidadeMotorEsquerdo   = 0;
int contadorLinhas = 0;
int kp = 3;
int kd = 1;
int ki = 1;
int correcao        = 0;
int erro            = 0;
int erroAnterior    = 0;
int somatorioDeErro = 0;
unsigned int sensorValues[NUM_SENSORES];
unsigned short binSensors[NUM_SENSORES];
QTRSensorsAnalog sensores((unsigned char[]) {
  5, 4, 3, 2, 1, 0
}, NUM_SENSORES, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
/* QTRSensorsAnalog qtra(unsigned char* pins, unsigned char numSensors,
   unsigned int timeout, unsigned char emitterPin);*/
void setup() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 400; i++) {
    sensores.calibrate(QTR_EMITTERS_ON);
  }
  digitalWrite(13, LOW);
  /* Faz a análise dos valores máximos e mínimos de todos
     os sensores. */
  if (DEBUG) {
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print(sensores.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print(sensores.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    delay(1000);
  }
}
void loop() {
  controle();
  if (DEBUG) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }

    Serial.print("posicao: ");
    Serial.print(posicao);
    Serial.print(" |  erro: ");
    Serial.print(erro);
    Serial.print(" |  esquerdo: ");
    Serial.print(velocidadeMotorEsquerdo);
    Serial.print(" |  direito: ");
    Serial.println(velocidadeMotorDireito);
    delay(250);
  }
}

void controle() {
  correcao = iniciaCorrecao();
  motorEsquerdo(VELOCIDADE_BASE);
  motorDireito(VELOCIDADE_BASE);
  if (erro < 0) {
    velocidadeMotorEsquerdo = abs(correcao - VELOCIDADE_BASE);
    motorEsquerdo(velocidadeMotorEsquerdo);
  }
  else if (erro > 0) {
    velocidadeMotorDireito = abs(correcao + VELOCIDADE_BASE);
    motorDireito(velocidadeMotorDireito);
  }
}

void leituraBin() {
  position = sensores.readLine(sensorValues, true);
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (sensorValues[i] < 50) {
      binSensors[i] = 1;
      contadorLinhas++;
    }
    else binSensors[i] = 0;
  }
}

int posicaoBin() {
  leituraBin();
  if (contadorLinhas == 2) {
    if (binSensors[2] == 1 && binSensors[3] == 1) {
      posicao = 0;
    } else if (binSensors[0] == 1 && binSensors[1] == 1)
      posicao = 20;
    else if (binSensors[1] == 1 && binSensors[2] == 1)
      posicao = 10;
    else if (binSensors[3] == 1 && binSensors[4] == 1)
      posicao = 10;
    else if (binSensors[4] == 1 && binSensors[5] == 1)
      posicao = 20;
  }
  else if (contadorLinhas == 1) {
    posicao = 25;
    for (int i = 0; i <= NUM_SENSORES; i++) {
      if (binSensors[i] == 0)
        posicao += 10;
      else
        posicao += 0;
    }
  }
  else {
    if (binSensors[0] == 1 && binSensors[1] == 1 && binSensors[2] == 1)
      posicao = 15;
    else if (binSensors[1] == 1 && binSensors[2] == 1 && binSensors[3] == 1)
      posicao = 5;
    else if (binSensors[2] == 1 && binSensors[3] == 1 && binSensors[4] == 1)
      posicao = 5;
    else if (binSensors[3] == 1 && binSensors[4] == 1 && binSensors[5] == 1)
      posicao = 15;
  }
  if(flagEsquerda)
   return posicao;
  if(flagDireita)
   return -posicao;
}

boolean flagEsquerda() {
  int contador = 0;
  leituraBin();
  for (int i = 0; i < (NUM_SENSORES / 2) - 1; i++) {
    if (binSensors[i] == 1)
      contador++;
  }
  if (contador > 0)
    return true;
  else return false;
}
boolean flagDireita() {
  int contador = 0;
  leituraBin();
  for (int i = (NUM_SENSORES/2) + 1; i < NUM_SENSORES; i++) {
    if (binSensors[i] == 1)
      contador;
  }
  if(contador > 0)
    return true;
  else return false;
}

int iniciaCorrecao() {
  erro = posicaoBin();
  return  (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
}

//Função para permitir apenas potências abaixo da máxima
int limitadorPotencia(int potencia) {
  int novaPotencia = potencia;
  if (abs(potencia) > VELOCIDADE_MAXIMA) {
    novaPotencia = (potencia / abs(potencia)) * VELOCIDADE_MAXIMA;
  }
  return novaPotencia;
}
//Controla o motor esquerdo para uma dada potência após ser limitada
void motorEsquerdo(int potencia) {
  int pwr = limitadorPotencia(potencia);
  if (DEBUG == 0) {
    if (pwr < 0) {
      analogWrite(MOTOR_E1, abs(pwr));
      digitalWrite(MOTOR_E2, LOW);
    }
    else {
      analogWrite(MOTOR_E2, abs(pwr));
      digitalWrite(MOTOR_E1, LOW);
    }
  }
}
//Controla o motor direito para uma dada potência após ser limitada
void motorDireito(int potencia) {
  int pwr = limitadorPotencia(potencia);
  if (DEBUG == 0) {
    if (pwr < 0) {
      digitalWrite(MOTOR_D1, LOW);
      analogWrite(MOTOR_D2, abs(pwr));
    }
    else {
      digitalWrite(MOTOR_D2, LOW);
      analogWrite(MOTOR_D1, abs(pwr));
    }
  }
}

