
#define LIGHT_SENSOR_PIN A0

#define PROX_SENSOR_L_PIN A1
#define PROX_SENSOR_R_PIN A2
#define PROX_SENSOR_FL_PIN A3
#define PROX_SENSOR_FR_PIN A4
#define PROX_SENSOR_RL_PIN A5
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN 6
#define PROX_SENSOR_DR_PIN 9

#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5



void hardware_setup() {
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

void setup() {
  Serial.begin(4800);

  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);

  // Set speed to max
  analogWrite(MOTOR_R_SPEED, 200);
  analogWrite(MOTOR_L_SPEED, 200);
  
  
}

int v = 100 ;
unsigned long Time=0;

void stop(){
    digitalWrite(MOTOR_RF_PIN, LOW);
    digitalWrite(MOTOR_RB_PIN, LOW);
    digitalWrite(MOTOR_LF_PIN, LOW);
    digitalWrite(MOTOR_LB_PIN, LOW);   
}

void front(){
    analogWrite(MOTOR_R_SPEED, v);
    analogWrite(MOTOR_L_SPEED, v);
    digitalWrite(MOTOR_RF_PIN, HIGH);
    digitalWrite(MOTOR_RB_PIN, LOW);
    digitalWrite(MOTOR_LF_PIN, HIGH);
    digitalWrite(MOTOR_LB_PIN, LOW);
    delay(50);
}


void rotate_left(){
  analogWrite(MOTOR_R_SPEED, v/3);
  analogWrite(MOTOR_L_SPEED, v/3);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);


}


void rotate_right(){
  analogWrite(MOTOR_R_SPEED, v/3);
  analogWrite(MOTOR_L_SPEED, v/3);
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);


}

void little_left(){
  analogWrite(MOTOR_R_SPEED, v/0.9);
  analogWrite(MOTOR_L_SPEED, v);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);


}
void little_right(){
  analogWrite(MOTOR_R_SPEED, v);
  analogWrite(MOTOR_L_SPEED, v/0.9);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
    

}

void very_little_left(){
  analogWrite(MOTOR_R_SPEED, v/0.95);
  analogWrite(MOTOR_L_SPEED, v);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);



}
void very_little_right(){
  analogWrite(MOTOR_R_SPEED, v);
  analogWrite(MOTOR_L_SPEED, v/0.95);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);


}

int FL(){return digitalRead(PROX_SENSOR_FL_PIN);} //Front-Left Sensor
int FR(){return digitalRead(PROX_SENSOR_FR_PIN);} //Front-Right Sensor
int L(){return digitalRead(PROX_SENSOR_L_PIN);} //90° Left Sensor
int R(){return digitalRead(PROX_SENSOR_R_PIN);} //90° Right Sensor
int BR(){return digitalRead(PROX_SENSOR_RL_PIN);} //135° Right Sensor
int BL(){return digitalRead(PROX_SENSOR_RR_PIN);} //135° Left Sensor
int FLL(){return digitalRead(PROX_SENSOR_DL_PIN);}  //45° Left Sensor 
int FRR(){return digitalRead(PROX_SENSOR_DR_PIN);}  //45° Right Sensor
int Light(){return digitalRead(LIGHT_SENSOR_PIN);} //Light sensor
int FL_anlg(){return analogRead(PROX_SENSOR_FL_PIN);} //analog read of Front-Left Sensor
int FR_anlg(){return analogRead(PROX_SENSOR_FR_PIN);} //analog read of Front-Right Sensor



bool AL=true;

void finish(){
  if (Light()==0 && AL){
      Time = millis();
      AL=false;
  }
  if (Light()==1){AL=true;}
  if(millis() > Time + 200 && !AL){
      Serial.println("delayed");
      stop();
      delay(500);
  }
}

void loop() {
  finish();
  if (FL()==1||FR()==1){
    if(FLL()==1){ 
        while (FL()==1||FR()==1){
          rotate_right();
          finish();
        }
      }
      else if(FRR()==1){ 
        while (FL()==1||FR()==1){
          rotate_left();
          finish();
        }  
      }
      else{
            if(L()==1&&R()==0){ 
              while (FL()==1||FR()==1){
                rotate_right();
                finish();
              }
            }
            else if(R()==1&&L()==0){ 
              while (FL()==1||FR()==1){
                rotate_left();
                finish();
              }
            }
            else{ 
              if (FL()==0){
                while (FR()==1){
                  rotate_left();
                  finish();
                }
              }
              else if (FR()==0){
                while (FL()==1){
                  rotate_right();
                  finish();
                }
              }
              else {
                if (FL_anlg()>FR_anlg()){
                  while (FL()==1||FR()==1){
                    rotate_left();
                    finish(); 
                  }                 
                }
                if (FR_anlg()>FL_anlg()){
                  while (FL()==1||FR()==1){
                    rotate_right();
                    finish();
                  }
                }
              }
            }
      }
      
  }

  else{ // on est dans le cas où aucun capteur avant n'est à 1
      
      if(FLL()==1){ //quand le capteur latéral avant gauche est à 1 (même si le capteur latéral droit l'est aussi) on tourne un peu à droite
        while (FLL()==1 &&!(FL()==1||FR()==1)){ //ce qui est après le && est un moyen d'éviter la boucle infini qui peut apparaître si on est dans cette condition après avoir tourné près d'un mur et s'y être bloqué
          little_right();
          finish();
        }
      }
      else if(FRR()==1){ //on tourne à gauche quand le capteur latéral avant droit est à 1 (et pas le capteur latéral avant gauche) on tourne un peu à gauche
        while (FRR()==1 &&!(FL()==1||FR()==1)){
          little_left();
          finish();
        }
      }
      else{
            if(L()==1&&R()==0){ //tend très légèrement vers l la droite quand le capteur latéral gauche est à 1 (et pas droit)
              while (L()==1&&R()==0 &&!(FL()==1||FR()==1)){
                very_little_right();
                finish();
              }
            }
            if(R()==1&&L()==0){ // pareil à gauche
              while (R()==1&&L()==0 &&!(FL()==1||FR()==1)){
                very_little_left();
                finish();
              }
            }
            else{ //il avance vers l'avant dans le reste des cas
              front();
              finish();
            }
      }
  }
  delay(50);

}
