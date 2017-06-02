/*

   Obs: portas dos verificadores: direita (digital 12)
                                  esquerda (analogica A7)
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
#define VBASE_CURVA               90
#define VBASE_RETA                100
#define VELOCIDADE_MAXIMA         254
#define VERIFICADOR_ESQUERDO      A7
#define KP_CURVA                  120
#define KP_RETA                   45
#define BOTAO                     13
#define TEMPO_FINAL               15000

unsigned short binSensors[NUM_SENSORES];
unsigned int sensorValues[NUM_SENSORES];
unsigned int position = 0;
unsigned int verificadorValue = 0;
unsigned int contadorLinhas = 0;

int POSICAO_MEDIA   =     ((NUM_SENSORES - 1) * 1000) / 2;
int DISTANCIA_MEDIA =     ((NUM_SENSORES - 1) * 10) / 2;
int parada                       = 0;
int base                         = 0;
float  velocidadeMotorDireito    = 0;
float  velocidadeMotorEsquerdo   = 0;
float deltaTime                  = 0;
float kp                         = KP_RETA;
float kd                         = 0.5;
float ki                         = 500;
float correcao                   = 0;
float erro                       = 0;
float erroAnterior               = 0;
float somatorioDeErro            = 0;
boolean flagVerificador          = false;
boolean botao                    = false;

/* QTRSensorsAnalog qtra(unsigned char* pins, unsigned char numSensors,
   unsigned int timeout, unsigned char emitterPin);*/
QTRSensorsAnalog sensores((unsigned char[]) {
  5, 4, 3, 2, 1, 0
}, NUM_SENSORES, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

void setup() {
  delay(500);
  pinMode(13, OUTPUT);
  for (int i = 0; i < 300; i++) {
    sensores.calibrate(QTR_EMITTERS_ON);
  }
  delay(1000);
  /* Faz a análise dos valores máximos e mínimos de todos
     os sensores. */
  if (DEBUG) {
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print(sensores.calibratedMinimumOn[i]);
      Serial.print('\t ');
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
  /*
    do {
    botao = digitalRead(13);
    Serial.print("Botao: ");
    Serial.println(botao);
    sensores.calibrate(QTR_EMITTERS_ON);
    if(botao == 1) {
      botao = true;
      delay(100);
    }
    } while (!botao);

    while (botao == false) {
    botao = digitalRead(13);

    if(botao){
      botao = true;
      delay(200);
    }
  */
  controle();
  if (DEBUG) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      Serial.print(binSensors[i]);
      Serial.print('\t');
    }
    Serial.print("Botão: ");
    Serial.print(botao);
    Serial.print(" |erro: ");
    Serial.print(erro);
    Serial.print(" |Sensores na Linha: ");
    Serial.print(contadorLinhas);
    Serial.print(" |esquerdo: ");
    Serial.print(velocidadeMotorEsquerdo);
    Serial.print(" |direito: ");
    Serial.print(velocidadeMotorDireito);
    Serial.print(" |");
    Serial.print(" TIPO:  ");
    if (flagVerificador)
      Serial.println("CURVA");
    if (!flagVerificador)
      Serial.println("RETA");
  }
}

void controle() {
  verificador();
  base = VBASE_CURVA;
  correcao = iniciaCorrecao();
  /*
    if (flagVerificador)
    base = VBASE_CURVA;
    else
    base = VBASE_RETA;
  */
  if (correcao == 0) {
    velocidadeMotorEsquerdo = base;
    velocidadeMotorDireito = base;
    if (!DEBUG) {
      motorEsquerdo(velocidadeMotorEsquerdo);
      motorDireito(velocidadeMotorDireito);
    }
  }
  else if (correcao < 0) {
    for (int i = 0; i > correcao; i--) {
      velocidadeMotorEsquerdo = abs(i - base);
      velocidadeMotorDireito = abs((i / 2) + base);
      if (velocidadeMotorEsquerdo > VELOCIDADE_MAXIMA)
        velocidadeMotorEsquerdo = VELOCIDADE_MAXIMA;
      if (!DEBUG) {
        motorEsquerdo(velocidadeMotorEsquerdo);
        motorDireito(velocidadeMotorDireito);
      }
    }
  }
  else if (correcao > 0) {
    for (int i = 0; i < correcao; i++) {
      velocidadeMotorDireito = abs(correcao + base);
      velocidadeMotorEsquerdo = abs((correcao / 2) - base);
      if (velocidadeMotorDireito > VELOCIDADE_MAXIMA)
        velocidadeMotorDireito = VELOCIDADE_MAXIMA;
      if (!DEBUG) {
        motorDireito(velocidadeMotorDireito);
        motorEsquerdo(velocidadeMotorEsquerdo);
      }
    }
  }
}

void verificador() {
  verificadorValue = analogRead(VERIFICADOR_ESQUERDO);
  if (verificadorValue < 500 && flagVerificador == false) {
    flagVerificador = true;
    delay(200);
  }
  else if (verificadorValue < 500 && flagVerificador == true) {
    if(posicaoBin() <= -1.5 || posicaoBin() >= 1.5)
      flagVerificador = true;
      else
      flagVerificador = false;
    delay(200);
  }
}

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
/*
  float posicaoBin(){
  int divisor = 0;
  int somadorDeErro = 0;

  for(int contador = 0; contador < NUM_SENSORES; contador++) {
    if(binSensors[contador] == 1){
      somadorDeErro += posicaoErro[contador];
      divisor++;
      if(DEBUG){
        Serial.println(somadorDeErro);
      }
    }
  }
  if(somadorDeErro == 0)
    return 0;
  else
    return somadorDeErro/divisor;
  }
*/

float posicaoBin() {
  leituraBin();
  int posicao = 0;
  if (contadorLinhas == 3) {
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
      return 2.5;
    else if (binSensors[1] == 1 && binSensors[2] == 1)
      return 1.0;
    else if (binSensors[2] == 1 && binSensors[3] == 1)
      return 0;
    else if (binSensors[3] == 1 && binSensors[4] == 1)
      return -1.0;
    else if (binSensors[4] == 1 && binSensors[5] == 1)
      return -2.5;
  }
  else if (contadorLinhas == 1) {
    if (binSensors[0] == 1)
      return 3.0;
    if (binSensors[1] == 1)
      return 1.5;
    else if (binSensors[2] == 1)
      return 0.5;
    else if (binSensors[3] == 1)
      return -0.5;
    else if (binSensors[4] == 1)
      return -1.5;
    else if (binSensors[5] == 1)
      return -3.0;
  }
  else if (contadorLinhas == 0) {
    return erroAnterior;
  }
}

float iniciaCorrecao() {
  erro = posicaoBin();
  erroAnterior = erro;

  //return  (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);

  if (flagVerificador) {
    kp = KP_CURVA;
    return (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
  }
  else
    kp = KP_RETA;
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

void paradaTempo() {
  unsigned long tempoParada = TEMPO_FINAL;
  unsigned long tempoInicio = millis();
  if ((tempoInicio - millis()) >= tempoParada) {
    if (DEBUG)
      Serial.println("Fim do trajeto. Parando");
    else {
      for (int i = base; i > 0; i - 50) {
        motorDireito(i);
        motorEsquerdo(i);
      }
    }
  }
}


