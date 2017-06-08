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
#define TAMANHO_FILA              20
#define NUM_SAMPLES_PER_SENSOR    4
#define MOTOR_E1                  6
#define MOTOR_E2                  5
#define MOTOR_D1                  10
#define MOTOR_D2                  9
#define DEBUG                     0
#define EMITTER_PIN               2
#define NUM_SENSORES              6
#define AUMENTA_VELOCIDADEF       10
#define AUMENTA_VELOCIDADET       5                 
#define VBASE_CALIB               70
#define VBASE_CURVA               60
#define VBASE_RETA                80
#define VELOCIDADE_MAXIMA         200
#define VELOCIDADE_MINIMA         -70
#define VERIFICADOR_ESQUERDO      A7
#define KP_CURVA                  15
#define KP_RETA                   20
#define BOTAO                     13
#define TEMPO_FINAL               1000
#define TEMPO_CALIB               2000
#define T1                        500
#define TEMPO_PARADA_C            50

unsigned short binSensors[NUM_SENSORES];
unsigned int sensorValues[NUM_SENSORES];
unsigned int position = 0;
unsigned int verificadorValue = 0;
unsigned int contadorLinhas = 0;
char VETOR_FILA[TAMANHO_FILA] = {'F','I','X','X','F','T', 'I', 'F', 'I', 'X', 'X','F', 'T', 'I', 'X', 'X', 'F', 'I', 'F', 'P'};
int POSICAO_MEDIA   =     ((NUM_SENSORES - 1) * 1000) / 2;
int DISTANCIA_MEDIA =     ((NUM_SENSORES - 1) * 10) / 2;
int parada                       = 0;
int base                         = 0;
int botao                        = 0;
int t0                           = 0;
int contador                     = 0;
float  velocidadeMotorDireito    = 0;
float  velocidadeMotorEsquerdo   = 0;
float deltaTime                  = 0;
float kp                         = KP_RETA;
float kd                         = 0;
float ki                         = 0;
float correcao                   = 0;
float erro                       = 0;
float erroAnterior               = 0;
float somatorioDeErro            = 0;
boolean flagVerificador          = false;
boolean estadoBotao              = false;
boolean flagCruzamento           = false;
boolean flag_curva               = false;
boolean flag_reta                = false;
/* QTRSensorsAnalog qtra(unsigned char* pins, unsigned char numSensors,
   unsigned int timeout, unsigned char emitterPin);*/
QTRSensorsAnalog sensores((unsigned char[]) {
  5, 4, 3, 2, 1, 0
}, NUM_SENSORES, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

void setup() {
  if (DEBUG) 
    Serial.begin(9600);
  pinMode(4, INPUT_PULLUP);
  delay(500);
  
  //calibraAutomatico();
  calibraManual();
  
  /*delay(500);
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
char primeiraPosicao = leituraVetor();
seguirLinha(primeiraPosicao);
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
    Serial.print(" |Cruzamento: ");
    Serial.print(flagCruzamento);
    Serial.print(" TIPO:  ");
    //    if (flagVerificador)
    Serial.println(primeiraPosicao);
    //    if (!flagVerificador)
    //      Serial.println("RETA");
  }
}

char leituraVetor(){
  char fila;
  verificaMarcacao();
  fila = VETOR_FILA[contador];
  
  return fila;
}

void parada_calibracao(unsigned long tempo)
{
  motorDireito(0);
  motorEsquerdo(0);
  delay(tempo);
}

void paradinha(unsigned long tempo) {
  int parada;
  bool estadoVerificador = false;
  unsigned long tempoInicial = 0;
  do{
    Serial.println("esperando v direito");
    parada = digitalRead(4);
    if(parada == 1 && millis() - tempoInicial > 200){
      tempoInicial = millis();
      estadoVerificador = true;
    }
  }while(!estadoVerificador);
  for (int i = 60; i > 0; i - 5) {
    Serial.println("parando");
    motorDireito(i);
    motorEsquerdo(i);
    delay(20);
  }
  Serial.println("parou");
  delay(tempo);
} 
void calibraManual() {
  estadoBotao = false;
  if (DEBUG)
    Serial.print("Calibrando...");
  do {
    sensores.calibrate(QTR_EMITTERS_ON);
    botao = digitalRead(4);
    Serial.println(estadoBotao);
    if (botao == 0){
      estadoBotao = true;
      break;
    }
  } while (!estadoBotao);
  delay(500);
  if(DEBUG)
    Serial.println(estadoBotao);
  estadoBotao = false;

  do {
    botao = digitalRead(4);
    if (botao == 0)
      estadoBotao = true;
  } while (!estadoBotao);
  delay(1000);
}

//Calibra girando um motor;
void calibraAutomatico() {
  int cont = 0;
  if (!DEBUG)
    motorDireito(VBASE_CALIB);
  do {
    sensores.calibrate(QTR_EMITTERS_ON);
    if (analogRead(A3) < 500 && analogRead(A2) < 500) {
      cont++;
      delay(500);
    }
  } while (cont < 3);
  parada_calibracao(300);
}

void seguirLinha(char fila) {
  if(fila == 'F'){
    controleReta();
  }
  else if(fila == 'I'){
    controleCurva();
  }
  else if(fila == 'X'){
    controleCurva();
  }
  else if(fila == 'T') {
    controleReta();
  }
  else if(fila == 'P') {
    paradaTempo();
  }
  
  /*verificaCruzamento();
  //verificaCurva();
  iniciaCorrecao();
  if (erro <= 1.5 || erro >= -1.5) {
    controleReta();
  }
  else if (erro > 1.5 || erro < -1.5) {
    controleCurva();
  }
  */
}

void controleCurva() {
  flag_curva = true;
  flag_reta = false;
  int base = VBASE_CURVA;
  correcao = iniciaCorrecao();
  if (correcao == 0) {
    velocidadeMotorEsquerdo = base;
    velocidadeMotorDireito = base;
    if (!DEBUG) {
      motorEsquerdo(velocidadeMotorEsquerdo);
      motorDireito(velocidadeMotorDireito);
    }
  }
  else if (correcao < 0) {
    velocidadeMotorEsquerdo = abs(correcao - base);
    velocidadeMotorDireito = -abs(correcao - base)/4;
    if (velocidadeMotorEsquerdo > VELOCIDADE_MAXIMA)
      velocidadeMotorEsquerdo = VELOCIDADE_MAXIMA;
    if (velocidadeMotorDireito > -VELOCIDADE_MINIMA)
      velocidadeMotorDireito = -VELOCIDADE_MINIMA;
    if (velocidadeMotorDireito < -VELOCIDADE_MAXIMA)
      velocidadeMotorDireito = -VELOCIDADE_MAXIMA;
    if (!DEBUG)
      motorEsquerdo(velocidadeMotorEsquerdo);
    motorDireito(velocidadeMotorDireito);
  }
  else if (correcao > 0) {
    velocidadeMotorDireito = abs(correcao + base);
    velocidadeMotorEsquerdo = -abs(correcao + base)/4;
    if (velocidadeMotorDireito > VELOCIDADE_MAXIMA)
      velocidadeMotorDireito = VELOCIDADE_MAXIMA;
    if (velocidadeMotorEsquerdo > -VELOCIDADE_MINIMA)
      velocidadeMotorEsquerdo = -VELOCIDADE_MINIMA;
    if (velocidadeMotorEsquerdo < -VELOCIDADE_MAXIMA)
      velocidadeMotorEsquerdo = -VELOCIDADE_MAXIMA;
    if (!DEBUG)
      motorDireito(velocidadeMotorDireito);
    motorEsquerdo(velocidadeMotorEsquerdo);
  }
}
void controleReta() {
  base = VBASE_RETA;
  flag_reta = true;
  flag_curva = false;
  correcao = iniciaCorrecao();
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
      if (velocidadeMotorDireito < VELOCIDADE_MINIMA)
        velocidadeMotorDireito = VELOCIDADE_MINIMA;
      if (!DEBUG) {
        motorEsquerdo(velocidadeMotorEsquerdo);
        motorDireito(velocidadeMotorDireito);
      }
    }
  }
  else if (correcao > 0) {
    for (int i = 0; i < correcao; i++) {
      velocidadeMotorDireito = abs(i + base);
      velocidadeMotorEsquerdo = abs((i / 2) - base);
      if (velocidadeMotorDireito > VELOCIDADE_MAXIMA)
        velocidadeMotorDireito = VELOCIDADE_MAXIMA;
      if (velocidadeMotorEsquerdo < VELOCIDADE_MINIMA)
        velocidadeMotorEsquerdo = VELOCIDADE_MINIMA;
      if (!DEBUG) {
        motorDireito(velocidadeMotorDireito);
        motorEsquerdo(velocidadeMotorEsquerdo);
      }
    }
  }
}
/*
void controleCurva() {
  base = VBASE_CURVA;
  flag_reta = false;
  flag_curva = true;
  correcao = iniciaCorrecao();
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
      velocidadeMotorEsquerdo = abs(i - base) + AUMENTA_VELOCIDADEF;
      velocidadeMotorDireito = base + i - AUMENTA_VELOCIDADET;
      if (velocidadeMotorEsquerdo > VELOCIDADE_MAXIMA)
        velocidadeMotorEsquerdo = VELOCIDADE_MAXIMA;
      if (velocidadeMotorDireito < VELOCIDADE_MINIMA)
        velocidadeMotorDireito = VELOCIDADE_MINIMA;
      if (!DEBUG) {
        motorEsquerdo(velocidadeMotorEsquerdo);
        motorDireito(velocidadeMotorDireito);
      }
    }
  }
  else if (correcao > 0) {
    for (int i = 0; i < correcao; i++) {
      velocidadeMotorDireito = abs(i + base) - AUMENTA_VELOCIDADEF;
      velocidadeMotorEsquerdo = base - i - AUMENTA_VELOCIDADET;
      if (velocidadeMotorDireito > VELOCIDADE_MAXIMA)
        velocidadeMotorDireito = VELOCIDADE_MAXIMA;
      if (velocidadeMotorEsquerdo < VELOCIDADE_MINIMA)
        velocidadeMotorEsquerdo = VELOCIDADE_MINIMA;
      if (!DEBUG) {
        motorDireito(velocidadeMotorDireito);
        motorEsquerdo(velocidadeMotorEsquerdo);
      }
    }
  }
}
*/
void verificaCruzamento() {
  if (contadorLinhas > 3) {
    flagCruzamento = true;
    motorDireito(50);
    motorEsquerdo(50);
    delay(400);
    flagCruzamento = false;
  }
}

  void verificaMarcacao() {
  verificadorValue = analogRead(VERIFICADOR_ESQUERDO);
  if (verificadorValue < 500 && millis() - t0 > 200) {
    t0 = millis();
    contador++;
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
      return 3.5;
    if (binSensors[1] == 1)
      return 1.5;
    else if (binSensors[2] == 1)
      return 0.5;
    else if (binSensors[3] == 1)
      return -0.5;
    else if (binSensors[4] == 1)
      return -1.5;
    else if (binSensors[5] == 1)
      return -3.5;
  }
  else if (contadorLinhas == 0) {
    return erroAnterior;
  }
}

float iniciaCorrecao() {
  erro = posicaoBin();
  erroAnterior = erro;
  //return  (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
  if (flag_reta) {
    kp = KP_RETA;
    return (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
  }
  else if (flag_curva) {
    kp = KP_CURVA;
    return  (kp * erro) + (kd * (erro - erroAnterior)) + (ki * somatorioDeErro);
  }
}\

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
  if ((millis() - tempoInicio) >= TEMPO_FINAL) {
    if (DEBUG)
      Serial.println("Fim do trajeto. Parando");
    else {
      for (int i = base; i > 0; i - 50) {
        motorDireito(i);
        motorEsquerdo(i);
      }
      delay(15000);
    }
  }
}


