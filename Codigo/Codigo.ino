

//EXPANSOR I2C PCF8574
#define SDA_PORT PORTD
#define SDA_PIN 4
#define SCL_PORT PORTD
#define SCL_PIN 5
#include <SoftWire.h>
SoftWire EXP = SoftWire();
const int expAddress = 0x20; //0x20 para PCF8574P; 0x38 para PCF8574AP
unsigned char expOutputRegister = 0b11111111;
void set_exp_value(char pin, char value);
byte get_exp_value(char pin);
#define EXP_P0    0
#define EXP_P1    1
#define EXP_P2    2
#define EXP_P3    3
#define EXP_P4    4
#define EXP_P5    5
#define EXP_P6    6
#define EXP_P7    7
#define VEL 255


// SENSORES DE LÍNEA
#define LINEA_1       A2
#define LINEA_2       A3
#define LINEA_3       A4
#define LINEA_SEL_1   12
#define LINEA_SEL_2   13
const int sensores_linea = 6;
unsigned int ADC_linea[sensores_linea];
void leer_sensores_linea(unsigned int* value);



// MOTORES
#define PWM_IZQ             6
#define PWM_DER             3
#define DIR_IZQ_1           EXP_P2
#define SET_DIR_IZQ_1_HIGH  set_exp_value(DIR_IZQ_1, HIGH)
#define SET_DIR_IZQ_1_LOW   set_exp_value(DIR_IZQ_1, LOW)
#define DIR_IZQ_2           EXP_P3
#define SET_DIR_IZQ_2_HIGH  set_exp_value(DIR_IZQ_2, HIGH)
#define SET_DIR_IZQ_2_LOW   set_exp_value(DIR_IZQ_2, LOW)
#define DIR_DER_1           EXP_P5
#define SET_DIR_DER_1_HIGH  set_exp_value(DIR_DER_1, HIGH)
#define SET_DIR_DER_1_LOW   set_exp_value(DIR_DER_1, LOW)
#define DIR_DER_2           EXP_P4
#define SET_DIR_DER_2_HIGH  set_exp_value(DIR_DER_2, HIGH)
#define SET_DIR_DER_2_LOW   set_exp_value(DIR_DER_2, LOW)




void setup() {
  pinMode(LINEA_SEL_1, OUTPUT);
  pinMode(LINEA_SEL_2, OUTPUT);
  pinMode(PWM_IZQ, OUTPUT);
  pinMode(PWM_DER, OUTPUT);

  Serial.begin(9600);
  EXP.begin();

}

void loop() {
  palante();
  /*
  leer_sensores_linea(ADC_linea);
  if(ADC_linea[2] >150 && ADC_linea[3] > 150){
      palante();
    
  }

  if(ADC_linea[2] > 150 && ADC_linea[3] < 150) {  //Negro el 2 y el 3 blanco
    derecha();
  }

  if(ADC_linea[3] > 150 && ADC_linea[2] < 150) {  //Negro el 2 y el 3 blanco
    izquierda();
  }

  if(ADC_linea[2] <150 && ADC_linea[3] < 150){
      parar();
    
  }
*/
  

/*
#ifdef TEST_MOTORES
  static int velocidad = 0;

  SET_DIR_IZQ_1_HIGH;
  SET_DIR_IZQ_2_LOW;
  SET_DIR_DER_1_HIGH;
  SET_DIR_DER_2_LOW;
  for(velocidad = 0; velocidad <= 255; velocidad += 1)
  {
    analogWrite(PWM_IZQ,velocidad);
    analogWrite(PWM_DER,velocidad);
    delay(10);
  }
  for(velocidad = 255; velocidad >= 0; velocidad -= 1)
  {
    analogWrite(PWM_IZQ,velocidad);
    analogWrite(PWM_DER,velocidad);
    delay(10);
  }

  SET_DIR_IZQ_1_LOW;
  SET_DIR_IZQ_2_HIGH;
  SET_DIR_DER_1_LOW;
  SET_DIR_DER_2_HIGH;
  for(velocidad = 0; velocidad >= -255; velocidad -= 1)
  {
    analogWrite(PWM_IZQ,-velocidad);
    analogWrite(PWM_DER,-velocidad);
    delay(10);
  }
  for(velocidad = -255; velocidad <= 0; velocidad += 1)
  {
    analogWrite(PWM_IZQ,-velocidad);
    analogWrite(PWM_DER,-velocidad);
    delay(10);
  }

  delay(1000);
#endif

*/

}

void set_exp_value(char pin, char value) {
  if(value == HIGH)
    expOutputRegister = expOutputRegister & (~(1<<pin));
  else // value == LOW
    expOutputRegister = expOutputRegister | (1<<pin);

  EXP.beginTransmission(expAddress);
  EXP.write(expOutputRegister);
  EXP.endTransmission();
}

byte get_exp_value(char pin) {
  byte value = 0;

  EXP.requestFrom(expAddress, 1);
  if(EXP.available())
    value = EXP.read();
  EXP.endTransmission();

  return (value & 1<<pin)>>pin;
}

void leer_sensores_linea(unsigned int* value) {
  digitalWrite(LINEA_SEL_1, LOW);
  value[0] = analogRead(LINEA_1);
  value[2] = analogRead(LINEA_2);
  value[4] = analogRead(LINEA_3);
  digitalWrite(LINEA_SEL_1, HIGH);
  value[1] = analogRead(LINEA_1);
  value[3] = analogRead(LINEA_2);
  value[5] = analogRead(LINEA_3);
}


void izquierda(){
  
    SET_DIR_IZQ_1_HIGH;
    SET_DIR_IZQ_2_LOW;
    SET_DIR_DER_1_LOW;
    SET_DIR_DER_2_HIGH;
    analogWrite(PWM_IZQ,VEL);
    analogWrite(PWM_DER,VEL);
}


void derecha(){
  
    SET_DIR_IZQ_1_LOW;
    SET_DIR_IZQ_2_HIGH;
    SET_DIR_DER_1_HIGH;
    SET_DIR_DER_2_LOW;
    analogWrite(PWM_IZQ,VEL);
    analogWrite(PWM_DER,VEL);
}


void palante(){
  
    SET_DIR_IZQ_1_LOW;
    SET_DIR_IZQ_2_HIGH;
    SET_DIR_DER_1_LOW;
    SET_DIR_DER_2_HIGH;
    analogWrite(PWM_IZQ,VEL);
    analogWrite(PWM_DER,VEL);
}



void parar(){
  
    SET_DIR_IZQ_1_LOW;
    SET_DIR_IZQ_2_HIGH;
    SET_DIR_DER_1_LOW;
    SET_DIR_DER_2_HIGH;
    analogWrite(PWM_IZQ,0);
    analogWrite(PWM_DER,0);
}
