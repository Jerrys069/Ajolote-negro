///////////////SENSORES////////////
#define s0      14
#define s1      15
#define s2      16
#define s3      17
#define AI      A4
/////////////////////////////////////
#define LED 13 
#define BOTON 8 
#define GO 10 
#define pwmi 3
#define izq1 4 
#define izq2 5 
#define pwmd 11 
#define der1 7
#define der2 6

int sensores [16];
int digital [16]; 
int lectura_fondo_blanco[16];
int lectura_linea_blanco[16];
int lectura_fondo_negro[16];
int lectura_linea_negro[16];
int umbral [16];

bool en_mitad_negra = false;

float KP=0.6;
float KD=17.00;
float KI=0.00089;

int vel=200;
int setpoint=850;

int pos = 0;
int last_prop = 0;
int error1 = 0;
int error2 = 0;
int error3 = 0;

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000110;
  pinMode(LED, OUTPUT); 
  pinMode(BOTON, INPUT);
  pinMode(izq1,OUTPUT); 
  pinMode(izq2,OUTPUT); 
  pinMode(der1,OUTPUT); 
  pinMode(der2,OUTPUT); 
  pinMode(pwmi, OUTPUT);
  pinMode(pwmd, OUTPUT);
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  digitalWrite(A5,HIGH);
  while(digitalRead(BOTON)== LOW);
  calibrar_sensores();
  while(digitalRead(BOTON)== LOW); 
}

void loop() {
  lectura(); 
  motores(0,0); 
  int go=digitalRead(GO); 
  while(go == 1){
    go = digitalRead(GO);
    lectura();
    PID(); 
    if(go == 0){
      motores(0,0); 
      break;
    }
  }
}

void calibrar_sensores() {
  for(int i=0; i<50; i++) {
    fondo(lectura_fondo_blanco);
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20); 
  }
  while(digitalRead(BOTON)== LOW);
  for(int i=0; i<50; i++) {
    lineas(lectura_linea_blanco);
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20);
  }
  while(digitalRead(BOTON)== LOW);
  for(int i=0; i<50; i++) {
    fondo(lectura_fondo_negro);
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20); 
  }
  while(digitalRead(BOTON)== LOW);
  for(int i=0; i<50; i++) {
    lineas(lectura_linea_negro);
    digitalWrite(LED, HIGH); delay(20);
    digitalWrite(LED, LOW); delay(20);
  }
  promedio();
}

void fondo(int lectura_fondo[]) {
  for(int i=0; i<16; i++){
    digitalWrite(s0,i&0x01);
    digitalWrite(s1,i&0x02);
    digitalWrite(s2,i&0x04);
    digitalWrite(s3,i&0x08);
    lectura_fondo[i]=analogRead(AI);
  }
}

void lineas(int lectura_linea[]) {
  for(int i=0; i<16; i++){
    digitalWrite(s0,i&0x01);
    digitalWrite(s1,i&0x02);
    digitalWrite(s2,i&0x04);
    digitalWrite(s3,i&0x08);
    lectura_linea[i]=analogRead(AI);
  }
}

void promedio() {
  for(int i=0; i<16; i++){
    umbral[i]=(lectura_fondo_blanco[i]+lectura_linea_blanco[i]+lectura_fondo_negro[i]+lectura_linea_negro[i])/4;
  }
}

void lectura() {
  int sumap = 0;
  int suma = 0;
  for(int i=0; i<16; i++){
    digitalWrite(s0,i&0x01);
    digitalWrite(s1,i&0x02);
    digitalWrite(s2,i&0x04);
    digitalWrite(s3,i&0x08);
    sensores[i]=analogRead(AI);
    if(sensores[i] < umbral[i]) {
      digital[i] = 1;
    } else {
      digital[i] = 0;
    }
    sumap += digital[i] * (100 * (16 - i));
    suma += digital[i];
  }
  if (suma > 0) {
    pos = sumap / suma;
  }
}

void PID() {
  int proporcional = pos - setpoint;
  int derivativo = proporcional - last_prop;
  int integral = error1 + error2 + error3;
  last_prop = proporcional;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;
  int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
  if(diferencial > vel) diferencial = vel;
  else if(diferencial < -vel) diferencial = -vel; 
  (diferencial < 0) ? motores(vel, vel+diferencial) : motores(vel-diferencial, vel);
}

void motores(int izq, int der) {
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
