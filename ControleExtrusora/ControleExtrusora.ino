#include <PID_v1.h>   //biblioteca para fazer o controle de aquecimento
#include "softwarePWM.h"

//parametros do termometro
#define amostras 20        //numero de leituras reais do termômetro para cada leitura retornada na função temp()
#define delayAmostras 10   //delay entre as leituras reais do termômetro

const double beta = 3950.0;
const double r0 = 100000.0;
const double t0 = 273.0 + 25.0;
const double rx = r0 * exp(-beta/t0);
const double vcc = 5.0;   //tensão em que o termômetro será ligado
const double R = 10000.0; //resistor que será usado como divisor de tensão

//parametros para o calculo do PID
double Kp = 0.01;  // Ganho Proporcional
double Ki = 0.1;   // Ganho Integral
double Kd = 0.1;   // Ganho Derivativo com 0.1 ficou bom


//parametros de controle zona 1 (pre aquecimento)
double setpointZona1 = 180;   //setpoint da zona 2
double temperaturaZona1 = 0;  //temperatura lida na zona 1
double potenciaZona1 = 0;     //potencia na resistencia da zona 1
int termometroZona1 = A0;     //Pino do termometro da zona1
int resistenciaZona1 = 5;     //Pino da resistencia da zona1

//parametros de controle zona 2 (bico)
double setpointZona2 = 240;   //setpoint da zona 2
double temperaturaZona2 = 0;  //temperatura lida na zona 2
double potenciaZona2 = 0;     //potencia na resistencia da zona 2 
int termometroZona2 = A1;     //Pino do termometro da zona2
int resistenciaZona2 = 6;     //Pino da resistencia da zona2


PID pidZona1(&temperaturaZona1, &potenciaZona1, &setpointZona1, Kp, Ki, Kd, DIRECT);
PID pidZona2(&temperaturaZona2, &potenciaZona2, &setpointZona2, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  pinMode(resistenciaZona1, OUTPUT);
  pinMode(resistenciaZona2, OUTPUT);
  digitalWrite(resistenciaZona1, LOW);
  digitalWrite(resistenciaZona2, LOW);
  delay(10);

  //configura PWM para controle de potencia das resistencias
  adicionaCanal(resistenciaZona1); 
  adicionaCanal(resistenciaZona2);

  // Configura PID
  pidZona1.SetMode(AUTOMATIC);
  pidZona1.SetSampleTime(2*amostras*delayAmostras);  // Intervalo aproximado de amostragem em milissegundos
  pidZona1.SetOutputLimits(0, 100); 

  pidZona2.SetMode(AUTOMATIC);
  pidZona2.SetSampleTime(amostras*delayAmostras);  // Intervalo aproximado de amostragem em milissegundos
  pidZona2.SetOutputLimits(0, 100); 

  Serial.println("Setup Completo");

}


double temp(int termometro){
  double acc = 0;
  //Lê o termometro vezes a quantidade de amostras definidas
  for(int i=0; i<amostras; i++){
    acc += analogRead(termometro);
    delay(delayAmostras);
  }
  //calcula a temperatura
  double v = (vcc*acc)/(amostras*1024.0);
  double rt = (vcc*R)/v-R;
  double t = beta/log(rt/rx);
  acc = (t-273.0);
  //retorna a temperatura em graus celcius 
  return acc;
}

void aqc(int resistencia, double potencia){
  digitalWrite(resistencia, HIGH);
  delay(potencia);
  digitalWrite(resistencia, LOW);
  delay(500 - potencia);  
}

void loop() {

  temperaturaZona1 = temp(termometroZona1);
  temperaturaZona2 = temp(termometroZona2);

  pidZona1.Compute();
  pidZona2.Compute();

  atualizaPotencia(resistenciaZona1, potenciaZona1);
  atualizaPotencia(resistenciaZona2, potenciaZona2);

  atualizaPWM();

  Serial.print("Setpoin Zona 1: ");
  Serial.print(setpointZona1);
  Serial.print(" Temperatura Zona 1: ");
  Serial.print(temperaturaZona1);
  Serial.print(" Potencia Zona 1: ");
  Serial.print(potenciaZona1);

  Serial.print(" || Setpoin Zona 2: ");
  Serial.print(setpointZona2);
  Serial.print(" Temperatura Zona 2: ");
  Serial.print(temperaturaZona2);
  Serial.print(" Potencia Zona 2: ");
  Serial.println(potenciaZona2);
  
}
