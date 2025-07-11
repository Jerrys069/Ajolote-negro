//////turbina///////
//#include <Servo.h>
//Servo turbina; 
///////////////SENSORES////////////
#define s0      14 //PA0 //14
#define s1      15 //PA1 //15
#define s2      16 //PA2 //16
#define s3      17 //PA3 //17
#define AI      A4
/////////////////////////////////////
#define LED 13 
#define BOTON 8 
#define GO 10 //#define GO11 
#define pwmi 3//PWM LEFT MOTOR 
#define izq1 4 //LEFT1 MOTOR 
#define izq2 5 //LEFT2 MOTOR 
#define pwmd 11 //PWM RIGHT MOTOR 
#define der1 7//RIGHT1 MOTOR 
#define der2 6 //RIGHT2 MOTOR /*#define STBY PA11 //9 

#define LED 13  
//////turbina estados///////
//int pot = 0;
//int pot2 = 70;
//int t = 0;
/////////////////////////////
int sensores [16]; //int umbral = 600; 
int digital [16]; 
int lectura_fondo[16];
int lectura_linea[16]; 
int umbral [16]; 
long int sumap, suma, pos,postlast, position; 
int barra = 0;
int LINEA = 1;  // linea negra es 1 y linea blanca es 0 barra china de qtr (cambia solamente esto)
//constantes //
float KP=0.60;//constante proporcional 1.05 lb .60
float KD=10;//constante derivativa 5
float KI=0.003;//constante integral.0025
//float KP=0.5;//constante proporcional 
//float KD=5;//constante derivativa 
//float KI=0.002;//constante integral
//Velocidades 
int vel=200;//VELOCIDAD MÁXIMA DEL ROBOT MÁXIMA 255//150 
int veladelante=160;//VELOCIDAD DEL FRENO DIRECCIÓN ADELANTE//100 //Revisar *VELATRAS* 
int velatras=130;//VELOCIDAD DEL FRENO DIRECCIÓN ATRÁS//REVISAR **VELADELATE 
int veladel2 = 110;//65 
int velatras2 =110;//65 
int error1=0; 
int error2=0; 
int error3=0; 
int error4=0; 
int error5=0; 
int error6=0; 
int error7=0; 
int error8=0; 
int proporcional=0; 
int integral=0; 
int derivativo=0; 
int diferencial=0;
int last_prop; 
int setpoint=850; 
void setup() { 
  // Serial.begin(9600); 
  //TCCR2B = TCCR2B & B11111000 | B00000001;
  //set timer 2 divisor to 1 for PWM frequency of 31372.55 Hz 
  //TCCR2B = TCCR2B & B11111000 | B00000010; //set timer 2 divisor to 8 for PWM frequency of 3921.16 Hz 
  //TCCR2B = TCCR2B & B11111000 | B00000011; //set timer 2 divisor to 32 for PWM frequency of 980.39 Hz 
  //TCCR2B = TCCR2B & B11111000 | B00000100; //set timer 2 divisor to 64 for PWM frequency of 490.20 Hz (The DEFAULT) 
  //TCCR2B = TCCR2B & B11111000 | B00000101; //set timer 2 divisor to 128 for PWM frequency of 245.10 Hz 
  TCCR2B = TCCR2B & B11111000 | B00000110; //set timer 2 divisor to 256 for PWM frequency of 122.55 Hz 
  //TCCR2B = TCCR2B & B11111000 | B00000111; //set timer 2 divisor to 1024 for PWM frequency of 30.64 Hz 
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
  // turbina.attach(9); 
  while(digitalRead(BOTON)== LOW);
  for(int i=0; i<50; i++) { 
    fondo();
  digitalWrite(LED, HIGH); delay(20);
  digitalWrite(LED, LOW); delay(20); 
  } while(digitalRead(BOTON)== LOW);
  for(int i=0; i<50; i++) { 
    lineas(); 
    digitalWrite(LED, HIGH);
    delay(20);
    digitalWrite(LED, LOW);
    delay(20);
    } 
    promedio(); 
    while(digitalRead(BOTON)== LOW); 
    digitalWrite(LED, LOW);
    }
    void loop(){ 
      //dir(); 
      lectura(); 
      motores(0,0); 
      int go=digitalRead(GO); 
      while(go == 1){
       
        int go = digitalRead(GO);
        
        frenos(); 
        lectura();
        PID(); 
        if(go == 0){
          motores(0,0); 
          break;
          
        }
          }
          } 
          void fondo() { 
  for(int i=0;i<16;i++){
    digitalWrite(s0,i&0x01);
    digitalWrite(s1,i&0x02);
    digitalWrite(s2,i&0x04);
    digitalWrite(s3,i&0x08);
    lectura_fondo[i]=analogRead(AI);
              //Serial.print( lectura_fondo [i]); 
              //Serial.print("\t"); 
              }
              //Serial.println(""); 
              } 
              void lineas() { 
                 for(int i=0;i<16;i++){
                    digitalWrite(s0,i&0x01);
                    digitalWrite(s1,i&0x02);
                    digitalWrite(s2,i&0x04);
                    digitalWrite(s3,i&0x08);
                    lectura_linea[i]=analogRead(AI);
                  //Serial.print( lectura_linea [i]); 
                  //Serial.print("\t");
                  }
                  //Serial.println(" "); 
                  } 
                  void promedio() {
                      for(int i=0;i<16;i++){
                      umbral[i]=(lectura_fondo[i]+lectura_linea[i])/2; 
                      //Serial.print(umbral [i]); 
                      //Serial.print("\t"); 
                      } 
                      //Serial.println (" "); 
                      } 
                      int lectura(void){ 
                        /*sensores[0]= analogRead(A0); sensores[1]= analogRead(A1); sensores[2]= analogRead(A2); sensores[3]= analogRead(A3); sensores[4]= analogRead(A4); sensores[5]= analogRead(A5); sensores[6]= analogRead(A6); sensores[7]= analogRead(A7); */ 
                           for(int i=0;i<16;i++){
                            digitalWrite(s0,i&0x01);
                            digitalWrite(s1,i&0x02);
                            digitalWrite(s2,i&0x04);
                            digitalWrite(s3,i&0x08);
                            sensores[i]=analogRead(AI);
                          if(LINEA ==0) {
                            if(sensores[i]<=umbral [i]){
                              digital[i]=0;}else{digital[i]=1;
                              }
                              }
                              if(LINEA ==1) {
                                if(sensores[i]<=umbral[i]){
                                  digital[i]=1;}else{digital[i]=0;
                                  }
                                  }
                                  //Serial.print(digital [i]);
                                  //Serial.print("\t");
                                  }
                                  //Serial.println(""); 
                                  if (barra == 0) {sumap=(1600*digital[0]+1500*digital[1]+1400*digital[2]+1300*digital[3]+1200*digital[4]+1100*digital[5]+1000*digital[6]+900*digital[7]+800*digital[8]+700*digital[9]+600*digital[10]+500*digital[11]+400*digital[12]+300*digital[13]+200*digital[14]+100*digital[15]);}
                                  if (barra == 1) {sumap=(1600*digital[15]+1500*digital[14]+1400*digital[13]+1300*digital[12]+1200*digital[11]+1100*digital[10]+1000*digital[9]+900*digital[8]+800*digital[7]+700*digital[6]+600*digital[5]+500*digital[4]+400*digital[3]+300*digital[2]+200*digital[1]+100*digital[0]) ;}
                                  suma = (digital[0]+digital[1]+digital[2]+digital[3]+digital[4]+digital[5]+digital[6]+digital[7]+digital[8]+digital[9]+digital[10]+digital[11]+digital[12]+digital[13]+digital[14]+digital[15]) ; 
                                  pos = (sumap / suma); 
                                  if(postlast <=100 && pos == -1) {
                                    pos = 0;
                                    } 
                                    if(postlast >=1500 && pos == -1) {
                                      pos = 1600; 
                                      } 
                                      postlast = pos; 
                                      return pos;
                                      }
                                      void PID(){
                                        proporcional=pos-setpoint;
                                        derivativo=proporcional-last_prop;
                                        integral=error1+error2+error3+error4+error5+error6+error7+error8;
                                        last_prop=proporcional; 
                                        error8=error7;
                                        error7=error6;
                                        error6=error5;
                                        error5=error4;
                                        error4=error3;
                                        error3=error2; 
                                        error2=error1;
                                        error1=proporcional; 
                                        int diferencial=(proporcional*KP) + (derivativo*KD) + (integral*KI);
                                        if(diferencial > vel) diferencial=vel;
                                        else if(diferencial < -vel) diferencial=-vel; 
                                        (diferencial < 0)? motores(vel, vel+diferencial) : motores(vel-diferencial, vel); 
                                        }
                                        void frenos(){ 
                                          if(pos<=100){
                                            motores( veladelante, -velatras); 
                                            } else if(pos>=1500){ motores(-velatras ,veladelante) ;
                                            }
                                            }
                                            void dir(){ 
                                              if(pos<=300){ 
                                                motores(-velatras2, veladel2); 
                                                } else if(pos>=1100){
                                                  motores(veladel2, -velatras2); 
                                                  }
                                                  }
                                                  void motores(int izq, int der){
                                                    //0 hasta 255 0 hasta -255 ////////////////motor LEFT "IZQUIERDO" //////////////////////// 
                                                    if(izq>=0){ 
                                                      digitalWrite(izq1,HIGH); 
                                                      digitalWrite(izq2,LOW); 
                                                      } else{ 
                                                        digitalWrite(izq1,LOW); 
                                                        digitalWrite(izq2,HIGH);
                                                        izq=izq*(-1); 
                                                        } 
                                                        analogWrite(pwmi,izq); 
                                                        ////////////////motor RIGHT "DERECHO" //////////////////////// 
                                                        if(der>=0){ 
                                                          digitalWrite(der1,HIGH); 
                                                          digitalWrite(der2,LOW); 
                                                          } else{ 
                                                            digitalWrite(der1,LOW); 
                                                            digitalWrite(der2,HIGH); 
                                                            der=der*(-1); 
                                                            } 
                                                            analogWrite(pwmd,der); 
                                                            }
