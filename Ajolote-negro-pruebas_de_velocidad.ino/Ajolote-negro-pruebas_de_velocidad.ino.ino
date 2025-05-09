//////turbina///////
//#include <Servo.h>
//Servo turbina; 
///////////////SENSORES////////////
#define s0      14 //PA0
#define s1      15 //PA1
#define s2      16 //PA2
#define s3      17 //PA3
#define AI      A4
/////////////////////////////////////
#define LED 13 
#define BOTON 8 
#define GO 10 
#define pwmi 3 //PWM LEFT MOTOR 
#define izq1 4 
#define izq2 5 
#define pwmd 11 //PWM RIGHT MOTOR 
#define der1 7 
#define der2 6 

int sensores[16];
int digital[16]; 
int lectura_fondo[16];
int lectura_linea[16]; 
int umbral[16]; 
long int sumap, suma, pos, postlast, position; 
long int posSuavizada = 0;
int barra = 0;
int LINEA = 1;  

// NUEVOS VALORES PID
float KP = 1.3;   // Antes: 1.8
float KD = 11.0;  // Antes: 8.5
float KI = 0.001; // Pequeño ajuste

int vel = 200;
int veladelante = 150;
int velatras = 140;
int veladel2 = 120;
int velatras2 = 120;

int error1 = 0, error2 = 0, error3 = 0, error4 = 0, error5 = 0, error6 = 0, error7 = 0, error8 = 0;
int proporcional = 0; 
int integral = 0; 
int derivativo = 0; 
int last_prop; 
int setpoint = 850; 

void setup() { 
  TCCR2B = TCCR2B & B11111000 | B00000110;
  pinMode(LED, OUTPUT); 
  pinMode(BOTON, INPUT);
  pinMode(izq1, OUTPUT); 
  pinMode(izq2, OUTPUT); 
  pinMode(der1, OUTPUT); 
  pinMode(der2, OUTPUT); 
  pinMode(pwmi, OUTPUT);
  pinMode(pwmd, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  digitalWrite(A5, HIGH);

  while(digitalRead(BOTON) == LOW);
  for(int i = 0; i < 50; i++) { 
    fondo();
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20); 
  }
  while(digitalRead(BOTON) == LOW);
  for(int i = 0; i < 50; i++) { 
    lineas(); 
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20);
  } 
  promedio(); 
  while(digitalRead(BOTON) == LOW); 
  digitalWrite(LED, LOW);
}

void loop() { 
  lectura(); 
  motores(0, 0); 
  int go = digitalRead(GO); 
  while(go == 1) {
    go = digitalRead(GO);
    frenos(); 
    lectura();
    PID(); 
    if(go == 0){
      motores(0, 0); 
      break;
    }
  }
}

void fondo() { 
  for(int i = 0; i < 16; i++) {
    digitalWrite(s0, i & 0x01);
    digitalWrite(s1, i & 0x02);
    digitalWrite(s2, i & 0x04);
    digitalWrite(s3, i & 0x08);
    lectura_fondo[i] = analogRead(AI);
  }
}

void lineas() { 
  for(int i = 0; i < 16; i++) {
    digitalWrite(s0, i & 0x01);
    digitalWrite(s1, i & 0x02);
    digitalWrite(s2, i & 0x04);
    digitalWrite(s3, i & 0x08);
    lectura_linea[i] = analogRead(AI);
  }
}

void promedio() {
  for(int i = 0; i < 16; i++) {
    umbral[i] = (lectura_fondo[i] + lectura_linea[i]) / 2; 
  }
}

int lectura(void) {
  for(int i = 0; i < 16; i++) {
    digitalWrite(s0, i & 0x01);
    digitalWrite(s1, i & 0x02);
    digitalWrite(s2, i & 0x04);
    digitalWrite(s3, i & 0x08);
    sensores[i] = analogRead(AI);
    digital[i] = (LINEA == 0) ? (sensores[i] > umbral[i]) : (sensores[i] <= umbral[i]);
  }

  if (barra == 0) {
    sumap = (1600*digital[0]+1500*digital[1]+1400*digital[2]+1300*digital[3]+1200*digital[4]+1100*digital[5]+1000*digital[6]+900*digital[7]+800*digital[8]+700*digital[9]+600*digital[10]+500*digital[11]+400*digital[12]+300*digital[13]+200*digital[14]+100*digital[15]);
  } else {
    sumap = (1600*digital[15]+1500*digital[14]+1400*digital[13]+1300*digital[12]+1200*digital[11]+1100*digital[10]+1000*digital[9]+900*digital[8]+800*digital[7]+700*digital[6]+600*digital[5]+500*digital[4]+400*digital[3]+300*digital[2]+200*digital[1]+100*digital[0]);
  }

  suma = 0;
  for(int i = 0; i < 16; i++) suma += digital[i];

  pos = (suma != 0) ? (sumap / suma) : postlast;
  if(postlast <= 100 && pos == -1) pos = 0;
  if(postlast >= 1500 && pos == -1) pos = 1600;
  postlast = pos;
  return pos;
}

void PID() {
  // Suavizado de posición
  posSuavizada = (posSuavizada * 3 + pos) / 4;

  proporcional = posSuavizada - setpoint;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6 + error7 + error8;
  last_prop = proporcional;

  // Desplazar errores
  error8 = error7;
  error7 = error6;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2; 
  error2 = error1;
  error1 = proporcional;

  int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
diferencial = constrain(diferencial, -vel / 1.5, vel / 1.5); // Antes era 1.2

  // Control proporcional de ambos motores
  int baseSpeed = vel;
  int leftSpeed = baseSpeed - diferencial;
  int rightSpeed = baseSpeed + diferencial;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motores(leftSpeed, rightSpeed);
}

void frenos() { 
  if(pos <= 100){
    motores(veladelante, -velatras); 
  } else if(pos >= 1500){
    motores(-velatras , veladelante);
  }
}

void dir(){ 
  if(pos <= 300){ 
    motores(-velatras2, veladel2); 
  } else if(pos >= 1100){
    motores(veladel2, -velatras2); 
  }
}

void motores(int izq, int der){
  if(izq >= 0){ 
    digitalWrite(izq1, HIGH); 
    digitalWrite(izq2, LOW); 
  } else { 
    digitalWrite(izq1, LOW); 
    digitalWrite(izq2, HIGH);
    izq = -izq; 
  }
  analogWrite(pwmi, izq); 

  if(der >= 0){ 
    digitalWrite(der1, HIGH); 
    digitalWrite(der2, LOW); 
  } else { 
    digitalWrite(der1, LOW); 
    digitalWrite(der2, HIGH); 
    der = -der; 
  }
  analogWrite(pwmd, der); 
}
