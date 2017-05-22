/*
 * 
 * Obs: portas dos verificadores: direita (digital 12)
 *                                esquerda (analogica A7)
 */
#include <QTRSensors.h>
/* São aproximadamente 10 us para realizar uma única conversão analog - digital. Então, se
  NUM_SAMPLES_PER_SENSOR é de 4 e NUM_SENSORES é 6, demorará 4 * 6 * 100 us = ~2.5ms para fazer
  uma leitura completa. Aumentar esse parâmetro aumenta a quantidade de leitura ao custo de
  tempo de processamento.*//* São aproximadamente 10 us para realizar uma única conversão analog - digital. Então, se
  NUM_SAMPLES_PER_SENSOR é de 4 e NUM_SENSORES é 6, demorará 4 * 6 * 100 us = ~2.5ms para fazer
  uma leitura completa. Aumentar esse parâmetro aumenta a quantidade de leitura ao custo de
  tempo de processamento.*/
#define NUM_SAMPLES_PER_SENSOR    4
#define MOTOR_E1                  6
#define MOTOR_E2                  5
#define MOTOR_D1                  10
#define MOTOR_D2                  9
#define DEBUG                     1
#define EMITTER_PIN               2
#define NUM_SENSORES              6
#define VELOCIDADE_BASE           200
#define VELOCIDADE_MAXIMA         254

int arrayPosicoes[] = {-25,-20,-15,-10,-5,0,5,10,15,20,25};
unsigned short binSensors[NUM_SENSORES];
unsigned int sensorValues[NUM_SENSORES];
unsigned int position = 0; 
unsigned int contadorLinhas = 0;
int POSICAO_MEDIA   =     ((NUM_SENSORES - 1) * 1000) / 2;
int DISTANCIA_MEDIA =     ((NUM_SENSORES - 1) * 10) / 2;
float  velocidadeMotorDireito    = 0;
float  velocidadeMotorEsquerdo   = 0;
float deltaTime                  = 0;
float kp                         = 14;
float kd                         = 0;
float ki                         = 0;
float correcao                   = 0;
float erro                       = 0;
float erroAnterior               = 0;
int   parada                     = 0;
float somatorioDeErro            = 0;
/* QTRSensorsAnalog qtra(unsigned char* pins, unsigned char numSensors,
   unsigned int timeout, unsigned char emitterPin);*/
QTRSensorsAnalog sensores((unsigned char[]) { 5, 4, 3, 2, 1, 0}, NUM_SENSORES, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
   
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
      Serial.print(binSensors[i]);
      Serial.print('\t');
    }
    Serial.print("posicao: ");
    Serial.print(posicaoBin());
    Serial.print(" |  erro: ");
    Serial.print(erro);
    Serial.print(" |  Sensores na Linha: ");
    Serial.print(contadorLinhas);
    Serial.print(" |  esquerdo: ");
    Serial.print(velocidadeMotorEsquerdo);
    Serial.print(" |  direito: ");
    Serial.println(velocidadeMotorDireito);
    delay(250);
 }
}

void controle() {
  correcao = iniciaCorrecao();
  if(correcao == 0){
    velocidadeMotorEsquerdo = VELOCIDADE_BASE;
    velocidadeMotorDireito = VELOCIDADE_BASE;
    motorEsquerdo(velocidadeMotorEsquerdo);
    motorDireito(velocidadeMotorDireito);
  }
  else if(correcao < 0) {
    velocidadeMotorEsquerdo = abs(correcao - VELOCIDADE_BASE);
    if(velocidadeMotorEsquerdo > VELOCIDADE_MAXIMA)
      velocidadeMotorEsquerdo = VELOCIDADE_MAXIMA;
    motorEsquerdo(velocidadeMotorEsquerdo);
  }
  else if(correcao > 0) {
    velocidadeMotorDireito = abs(correcao + VELOCIDADE_BASE);
    if(velocidadeMotorDireito > VELOCIDADE_MAXIMA)
      velocidadeMotorDireito = VELOCIDADE_MAXIMA;
    motorDireito(velocidadeMotorDireito);
  }
}
//Funcionando Ok!
void leituraBin() {
  //Nessa linha de comando, os valores, já calibrados, são distribuidos no array "sensorValues", com o comando readLine. "position" neste caso não está sendo utilizado, mas sensorValues
  //têm seu valor alterado
  position = sensores.readLine(sensorValues, true);
  contadorLinhas = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (sensorValues[i] < 50) {
      binSensors[i] = 1;
      contadorLinhas++; // Quantos sensores estão vendo linha
    }
    else binSensors[i] = 0;
  }
}
float posicaoBin() {
  leituraBin();
  int posicao = 0;
  if(contadorLinhas == 3){
    if (binSensors[0] == 1 && binSensors[1] == 1 && binSensors[2] == 1)
      return 1.5;
    else if (binSensors[1] == 1 && binSensors[2] == 1 && binSensors[3] == 1)
      return 0.5;
    else if (binSensors[2] == 1 && binSensors[3] == 1 && binSensors[4] == 1)
      return -0.5;
    else if (binSensors[3] == 1 && binSensors[4] == 1 && binSensors[5] == 1)
      return -1.5;
  }
  else if (contadorLinhas == 2) {
    if (binSensors[0] == 1 && binSensors[1] == 1)
      return 2.0;
    else if (binSensors[1] == 1 && binSensors[2] == 1)
      return 1.0;
    else if (binSensors[2] == 1 && binSensors[3] == 1)
      return 0;
    else if (binSensors[3] == 1 && binSensors[4] == 1)
      return -1.0;
    else if (binSensors[4] == 1 && binSensors[5] == 1)
      return -2.0;
  }
  else if (contadorLinhas == 1) {
    if (binSensors[0] == 1)
      return 2.5;
    if (binSensors[1] == 1)
      return 1.5;
    else if (binSensors[2] == 1)
      return 0.5;
    else if (binSensors[3] == 1)
      return -0.5;
    else if (binSensors[4] == 1)
      return -1.5;
    else if (binSensors[5] == 1)
      return -2.5;
  }
  else if(contadorLinhas == 0){
    return erroAnterior;
  }
}

float iniciaCorrecao() {
  erro = posicaoBin();
  erroAnterior = erro;
  return  (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
}

//Função para permitir apenas potências abaixo da máxima
int limitadorPotencia(int potencia) {
  int novaPotencia = potencia;
  if (abs(potencia) > VELOCIDADE_MAXIMA) {
    novaPotencia = VELOCIDADE_MAXIMA;
  }
  return novaPotencia;
}
//Controla o motor esquerdo para uma dada potência após ser limitada
void motorEsquerdo(int pwr) {
  //int pwr = limitadorPotencia(potencia);
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
void motorDireito(int pwr) {
  //int pwr = limitadorPotencia(potencia);
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


