/////////////////////////////////Initialisation des paramètres de base du robot

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

/////////////////////////////////Fin de l’initialisation des paramètres de base du robot


/*
De base on ne voulait pas faire un robot qui longe les murs, mais on a décidé de faire un programme en 2 parties.
La première la partie où il longe les murs et la deuxième on il va chercher les cases par lui même (si longer les murs n’était pas un succès)
On va passer à la seconde partie, si le robot se rend compte que la case rouge ou noire ne sont pas sur les mêmes continuités de mur. (cas où il est retourné à la case rouge sans être passé par la noire)
Sinon, l’autre moyen de passer à la partie 2 est si le robot a bouclé, on a donc mis une limite de temps de 100 secondes (si le robot est resté en partie plus de 100s, il passe en partie 2)
*/

/////////////////////////////////Initialisation de variables utilisées à de nombreuses reprises dans tout le programme

int v = 100 ; //notre facteur de vitesse de nos robots
unsigned long Time=0; //cette variable va servir à mesurer le temps passer sur la case noire ou rouge pour savoir s'il faut s'arrêter ou non
bool AL=true; // boolean qui gère le premier parcours (depuis la case rouge jusqu'à la noir)
bool case_noire_atteinte=false; //boolean qui gère le retour à la case départ

/////////////////////////////////Initialisation de fonctions basiques utilisées à de nombreuses reprises dans le programme

bool Rouge(){ //teste si le robot a capté du rouge
  return Light()<290 && Light()>260;
}

bool Noir(){ // teste si le robot a capté du noir
  return Light()<50;
}

bool Blanc(){ // teste si le robot a capté du blanc
  return Light()>800;
}

//Cette fonction arrête tous les moteurs du robot et permet son arrêt total
void stop(){ 
    digitalWrite(MOTOR_RF_PIN, LOW);
    digitalWrite(MOTOR_RB_PIN, LOW);
    digitalWrite(MOTOR_LF_PIN, LOW);
    digitalWrite(MOTOR_LB_PIN, LOW);   
}

//Cette fonction fait avancer le robot tout droit à vitesse v
void front(){ 
    analogWrite(MOTOR_R_SPEED, v);
    analogWrite(MOTOR_L_SPEED, v);
    digitalWrite(MOTOR_RF_PIN, HIGH);
    digitalWrite(MOTOR_RB_PIN, LOW);
    digitalWrite(MOTOR_LF_PIN, HIGH);
    digitalWrite(MOTOR_LB_PIN, LOW);
    delay(50);
}

//Ces fonctions font tourner sur place le robot 
void rotate_left(){ //Gauche
  analogWrite(MOTOR_R_SPEED, v/3);
  analogWrite(MOTOR_L_SPEED, v/3);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
}
void rotate_right(){ //Droite
  analogWrite(MOTOR_R_SPEED, v/3);
  analogWrite(MOTOR_L_SPEED, v/3);
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
}

// Cette fonction gère les rotations qui n'arrêtent pas le robot 
void advanced_turn(float RSpeedFactor, float LSpeedFactor){ //cette fonction sert à gérer la différence de vitesse entre les moteurs droit et gauche
  analogWrite(MOTOR_R_SPEED, v*RSpeedFactor);
  analogWrite(MOTOR_L_SPEED, v*LSpeedFactor);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
}

//Ces fonctions sont des rotation basiques utilisés à de nombreuses reprises dans le programme
void little_left(){ //le robot avance en tournant un peu sur la gauche 
  advanced_turn(1.1,1);
}
void little_right(){ //le robot avance en tournant un peu sur la droite
  advanced_turn(1,1.1);
}

void very_little_left(){ //le robot avance en tournant très peu sur la gauche
  advanced_turn(1.0526,1);
}
void very_little_right(){ //le robot avance en tournant très peu sur la droite
    advanced_turn(1,1.0526);
}


//Ces fonctions servent à faciliter l'accès aux capteurs ainsi que de leurs donner des noms plus appropriés
int FL(){return digitalRead(PROX_SENSOR_FL_PIN);} //Front-Left Sensor
int FR(){return digitalRead(PROX_SENSOR_FR_PIN);} //Front-Right Sensor
int L(){return digitalRead(PROX_SENSOR_L_PIN);} //90° Left Sensor
int R(){return digitalRead(PROX_SENSOR_R_PIN);} //90° Right Sensor
int BR(){return digitalRead(PROX_SENSOR_RL_PIN);} //135° Right Sensor
int BL(){return digitalRead(PROX_SENSOR_RR_PIN);} //135° Left Sensor
int FLL(){return digitalRead(PROX_SENSOR_DL_PIN);}  //45° Left Sensor 
int FRR(){return digitalRead(PROX_SENSOR_DR_PIN);}  //45° Right Sensor
int Light(){return analogRead(LIGHT_SENSOR_PIN);} //Light sensor
int FL_anlg(){return analogRead(PROX_SENSOR_FL_PIN);} //analog read of Front-Left Sensor
int FR_anlg(){return analogRead(PROX_SENSOR_FR_PIN);} //analog read of Front-Right Sensor


//Cette fonction teste si on a atteint la case rouge ou la noire, en évaluant le temps passé à capter de la couleur noire ou  rouge (pour faire la différence entre les cases et les lignes)
//Elle permet l’arrêt pendant 4 secondes sur la case noir puis l’arrêt total lorsque la case rouge est atteinte après être déjà passé sur la case noire
void finish(){  
  if (case_noire_atteinte){ //dans ce cas là, on va regarder si on a atteint la case rouge, si on a déjà atteint la case noire avant
    
    if (Rouge() && AL){
      Time = millis();
      AL=false;
    }
    if ( Blanc() || Noir() ){AL=true;}
    if(millis() > Time + 500 && !AL){
      Serial.println("delayed rouge");
      stop();
      delay(5000);
      Serial.println(Light());
    }
  }
  else { //ici on s’arrête 5 secondes sur la case noire, la première fois qu’on passe dessus
    if (Noir() && AL){
      Time = millis();
      AL=false;
    }
    if (Blanc()) {AL=true;}
    if(millis() > Time + 500 && !AL){
      Serial.println("delayed noir");
      stop();
      delay(5000);
      case_noire_atteinte=true;
      AL=true;
    }
  }
}

//////////////////////////////////////// Partie réservée à tout ce qui concerne le longeage de mur

//////////////////Initialisation des variables nécessaires pour cette partie 
bool lonjmur=true; //cette variable sert à savoir si le robot va longer le mur ou non
bool RougeAvantNoir=true; //cette variable dit si le robot à trouver la case rouge avant la noire lorsqu’il longeait les mur (attention, true veut dire que rouge n’est pas encore trouvé avant la noire)
bool AuMoinsUneCaseBlancheAtteinte=false; //dans le nom, variable utile lorsqu’on longe le mur

/////////////////Initialisation des fonctions nécessaires à cette partie

//Cette fonction teste si on a atteint la case rouge avant la noire,, en évaluant le temps passé à capter de la couleur rouge
void finish_lonjmur(){ 
  if (Rouge() && RougeAvantNoir){
    Time = millis();
    RougeAvantNoir=false;
  }
  if ( Blanc() || Noir() ){RougeAvantNoir=true;}
  if(millis() > Time + 500 && !RougeAvantNoir){
    lonjmur=false; //
  }
}

//fonction pour faire tourner le robot à droite si il est trop proche du mur de gauche ou si il détecte un mur en face
void right(){  
  advanced_turn(0.6,1.3);
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
}

//fonction pour faire tourner le robot à gauche si il s’éloigne trop du mur de gauche
void murleft(){ 
  advanced_turn(1.17,0.83); 
}


void Lonjmur() { //cette fonction gère la partie longeage de mur
  finish();
  if ((!case_noire_atteinte)&&AuMoinsUneCaseBlancheAtteinte) //on teste la fonction finish_mur à chaque boucle après avant au moins atteint une case blanche
    finish_lonjmur();
  if (millis()/1000 > 100)
    lonjmur=false; //passe à un déplacement aléatoire au bout de 100 secondes à longer les murs sans succès
  if (FLL()==1){
    while(FLL()==1){ //fait une rotation à droite quand le capteur avant gauche est à 1 (sans le capteur avant droit)
      right();
    }
  }
  else
    murleft();

  if (!AuMoinsUneCaseBlancheAtteinte){
    if (Noir())
      AuMoinsUneCaseBlancheAtteinte=true;
  }
}

////////////////////////////////////////Cette fonction gère toute la partie où le robot cherche la case (rouge ou noire) si le robot n’a pas pu fonctionner en longeant les murs
void random() { 
  finish();
  if (FL()==1||FR()==1){ // on est dans le cas où aucun capteur avant n'est à 1
    if(FLL()==1){  //si le capteur 45° gauche est activé on va faire tourner le robot sur la droite jusqu’à ce que l’avant soit libre
        while (FL()==1||FR()==1){
          rotate_right();
          finish();
        }
      }
      else if(FRR()==1){ //pareil mais dans l’autre sens
        while (FL()==1||FR()==1){
          rotate_left();
          finish();
        }  
      }
      else{ //ici on cherche à savoir s’il vaut mieux tourner à gauche ou à droite lorsque aucune capteur 45° avant n’est activé
            if(L()==1&&R()==0){ //ici on tourne sur la droite car on a le capteur 90° gauche activé
              while (FL()==1||FR()==1){
                rotate_right();
                finish();
              }
            }
            else if(R()==1&&L()==0){ //pareil dans l’autre sens
              while (FL()==1||FR()==1){
                rotate_left();
                finish();
              }
            }
            else{ //ici on va tourner à gauche si seulement le capteur avant-droite est activé et inversement sur la droite
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
              else { //cette fois-ci on va se décider le sens de rotation par rapport à la distance des deux capteurs
                if (FL_anlg()>=FR_anlg()){ //le capteur de droite étant plus proche, le robot tourne sur la gauche
                  while (FL()==1||FR()==1){
                    rotate_left();
                    finish(); 
                  }                 
                }
                if (FR_anlg()>FL_anlg()){ //pareil dans l’autre sens
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
}

/////////////////////////////////////////Boucle de fonctionnement

void loop(){
  if (lonjmur) //si on est dans la configuration de longeage de mur, on appelle la fonction qui longe les murs
    Lonjmur();
  else //sinon on appelle la fonction qui cherche les cases sans longer
    random();
}

