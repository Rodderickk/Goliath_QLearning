//Q - Learning Algorith for a Hexapod robot
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"
#include <NewPing.h>
#define trigPin 10
#define echoPin 6
#define maxDistance 300

NewPing ultrasonic(trigPin, echoPin, maxDistance);

//Asignar la dirección hexadecimal de cada driver y su respectivo lado en el hexapod
//Driver Izuierdo | Dirección 0x41
Adafruit_PWMServoDriver leftDriver = Adafruit_PWMServoDriver();
//Driver derecho | Dirección 0x40
Adafruit_PWMServoDriver rightDriver = Adafruit_PWMServoDriver(0x41);

//Fórmula para cuentas de servomotor.
//nCuentas=Pulso_ms*(frecuencia_Hz/1000)*4096
//0º - 0.5 ms - 123 cuentas
//180º - 2.4 ms - 590 cuentas
//char var, data;
unsigned int pos0 = 123;      //Ancho de pulso en cuentas pos 0º
unsigned int pos180 = 590;    //Ancho de puslo en cuentas pos 180º

//Variables de almacenamiento de instrucciones
//Variables de almacenamiento de instrucciones
//////////////////////////////////////////////////////////////////

//Parametros computacionales
float gamma = 0.75;     //look-ahead weight
 //'Forgetfulness weight'. The closer this is to 1 the more wgt is given to recent samples
float alpha = 0.1;  
float AnguloAnteriorL1;
float AnguloAnteriorL2;
float AnguloAnteriorD1;
float AnguloAnteriorD2;
float AnguloAnteriorL7;
float AnguloAnteriorL8;
float AnguloAnteriorD7;
float AnguloAnteriorD8;

//--------------------------------------------
//Función para convertir las cuentas en ángulos lado derecho
void setServoR(uint8_t n_servo, float angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180); 
  rightDriver.setPWM(n_servo, 0, duty);
}

//Función para convertir las cuentas en ángulos lado izquierdo
void setServoL(uint8_t n_servo, float angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180); 
  leftDriver.setPWM(n_servo, 0, duty);
}

//--------------------------------------------------------

//parametres for getAction()
//epsilon is the probability of choosing an action randomly.  
//1-epsilon is the probability of choosing the optimal action
float epsilon;

//***********************************PARAMETROS DE SERVOS*********************************
//Para el caso de este proyecto, las extremidades 3 y 4 quedarán inmovilizadas, o bien se retirarán
//Extremidades 1, 2, 5 y 6. El servo de la tibia quedará inmovilizado
//Generando así una matriz para cada extremidad usando solo femur y coxa.
//Tibia se queda en condiciones iniciales (140) para dar soporte

//Extremidad 1
//Servo 1 - Femur
const int numLTheta1States = 6;
float Ltheta1InitialAngle = 110.0;
float Ltheta1Max = 140.0;
float Ltheta1Min = 100.0;
float deltaLTheta1 = (Ltheta1Max - Ltheta1Min)/(float(numLTheta1States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ls1 = int((Ltheta1InitialAngle - Ltheta1Min)/deltaLTheta1);
float delayTimeL1 = 4.5*deltaLTheta1;

//Extremidad 1
//Servo 2 - Coxa
const int numLTheta2States = 6;
float Ltheta2InitialAngle = 110.0;
float Ltheta2Max = 130.0;
float Ltheta2Min = 50.0;
float deltaLTheta2 = (Ltheta2Max - Ltheta2Min)/(float(numLTheta2States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ls2 = int((Ltheta2InitialAngle - Ltheta2Min)/deltaLTheta2);
float delayTimeL2 = 4.5*deltaLTheta2;

//------------------------------------------------------------------
//Extremidad 2

//Extremidad 2
//Servo 1 - Femur
const int numDTheta1States = 6;
float Dtheta1InitialAngle = 110.0;
float Dtheta1Max = 140.0;
float Dtheta1Min = 100.0;
float deltaDTheta1 = (Dtheta1Max - Dtheta1Min)/(float(numDTheta1States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ds1 = int((Dtheta1InitialAngle - Dtheta1Min)/deltaDTheta1);
float delayTimeD1 = 4.5*deltaDTheta1;

//Extremidad 2
//Servo 2 - Coxa
const int numDTheta2States = 6;
float Dtheta2InitialAngle = 50.0;
float Dtheta2Max = 100.0;
float Dtheta2Min = 30.0;
float deltaDTheta2 = (Dtheta2Max - Dtheta2Min)/(float(numDTheta2States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ds2 = int((Dtheta2InitialAngle - Dtheta2Min)/deltaDTheta2);
float delayTimeD2 = 4.5*deltaDTheta2;


//----------------------------------------------------------------

//Extremidad 5
//Servo 1 - Femur
const int numLTheta7States = 6;
float Ltheta7InitialAngle = 110.0;
float Ltheta7Max = 140.0;
float Ltheta7Min = 100.0;
float deltaLTheta7 = (Ltheta7Max - Ltheta7Min)/(float(numLTheta7States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ls7 = int((Ltheta7InitialAngle - Ltheta7Min)/deltaLTheta7);
float delayTimeL7 = 4.5*deltaLTheta7;

//Extremidad 5
//Servo 2 - Coxa
const int numLTheta8States = 6;
float Ltheta8InitialAngle = 70.0;
float Ltheta8Max = 130.0;
float Ltheta8Min = 40.0;
float deltaLTheta8 = (Ltheta8Max - Ltheta8Min)/(float(numLTheta8States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ls8 = int((Ltheta8InitialAngle - Ltheta8Min)/deltaLTheta8);
float delayTimeL8 = 4.5*deltaLTheta8;

//--------------------------------------------------------------------

//Extremidad 6
//Servo 1 - Femur
const int numDTheta7States = 6;
float Dtheta7InitialAngle = 105.0;
float Dtheta7Max = 130.0;
float Dtheta7Min = 70.0;
float deltaDTheta7 = (Dtheta7Max - Dtheta7Min)/(float(numDTheta7States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ds7 = int((Dtheta7InitialAngle - Dtheta7Min)/deltaDTheta7);
float delayTimeD7 = 4.5*deltaDTheta7;

//Extremidad 6
//Servo 2 - Coxa
const int numDTheta8States = 6;
float Dtheta8InitialAngle = 100.0;
float Dtheta8Max = 130.0;
float Dtheta8Min = 60.0;
float deltaDTheta8 = (Dtheta8Max - Dtheta8Min)/(float(numDTheta8States)-1.0);
//This is an integer between zero and numTheta1States-1 used 
//to index the state number of servo1
int Ds8 = int((Dtheta8InitialAngle - Dtheta8Min)/deltaDTheta8);
float delayTimeD8 = 4.5*deltaDTheta8;

//---------------------------------------------------------------

//Initialize Q to zeros for the ext 1
const int numStates1 = numLTheta1States*numLTheta2States;
const int numActions1 = 4;
float Q1[numStates1][numActions1];

//Initialize Q to zeros for ext 2
const int numStates2 = numDTheta1States*numDTheta2States;
const int numActions2 = 4;
float Q2[numStates2][numActions2];

//Initialize Q to zeros for the ext 5
const int numStates5 = numLTheta7States*numLTheta8States;
const int numActions5 = 4;
float Q5[numStates5][numActions5];

//Initialize Q to zeros for ext 6
const int numStates6 = numDTheta7States*numDTheta8States;
const int numActions6 = 4;
float Q6[numStates6][numActions6];

//---------------------------------------------------------------
//Initialize the state number. The state number is calculated using the theta1 state number and 
//the theta2 state number.  This is the row index of the state in the matrix Q. Starts indexing at 0.
int s1 = int(Ls1*numLTheta2States + Ls2);
int s1Prime = s1;

int s2 = int(Ds1*numDTheta2States + Ds2);
int s2Prime = s2;

int s5 = int(Ls7*numLTheta8States + Ls8);
int s5Prime = s5;

int s6 = int(Ds7*numDTheta8States + Ds8);
int s6Prime = s6;

//---------------------------------------------------------------------
//Initialize vars for getDeltaDistanceRolled()
//Esta variable es compartida, no cambia para cada extremidad
float distanceNew = 0.0;
float distanceOld = 0.0;
float deltaDistance = 0.0;


//-------Verificar si se cambia la variable//////////////////////////*********************
//These get used in the main loop
float r = 0.0;
float lookAheadValueUno = 0.0;
float lookAheadValueDos = 0.0;
float lookAheadValueCinco = 0.0;
float lookAheadValueSeis = 0.0;
float sampleUno = 0.0;
float sampleDos = 0.0;
float sampleCinco = 0.0;
float sampleSeis = 0.0;
int a = 0;
int b = 0;
int c = 0;
int d = 0;
//---------------------------------------***************************

void setup(){
  Serial.begin(9600);
  leftDriver.begin();
  rightDriver.begin();
  leftDriver.setPWMFreq(60);
  rightDriver.setPWMFreq(60);

  //Extremidades en condiciones iniciales
  //Se guarda la variable en AnguloAnterior##
  
  //Extremidad 1 - Tibia 0 | Femur 1 | Coxa 2
  setServoL(0,140);
  setServoL(1,Ltheta1InitialAngle);
  AnguloAnteriorL1 = Ltheta1InitialAngle;
  setServoL(2,Ltheta2InitialAngle);
  AnguloAnteriorL2 = Ltheta2InitialAngle;
  
  //Extremidad 2 - Tibia 0 | Femur 1 | Coxa 2
  setServoR(0,140);
  setServoR(1,Dtheta1InitialAngle);
  AnguloAnteriorD1 = Dtheta1InitialAngle;
  setServoR(2,Dtheta2InitialAngle);
  AnguloAnteriorD2 = Dtheta2InitialAngle;
  
  //Extremidad 5 - Tibia 6 | Femur 7 | Coxa 8
  setServoL(6,140);
  setServoL(7,Ltheta7InitialAngle);
  AnguloAnteriorL7 = Ltheta7InitialAngle;
  setServoL(8,Ltheta8InitialAngle);
  AnguloAnteriorL8 = Ltheta8InitialAngle;
  
  //Extremidad 6 - Tibia 6 | Femur 7 | Coxa 8
  setServoR(6,140);
  setServoR(7,Dtheta7InitialAngle);
  AnguloAnteriorD7 = Dtheta7InitialAngle;
  setServoR(8,Dtheta8InitialAngle);
  AnguloAnteriorD8 = Dtheta8InitialAngle;
  delay(4000);
}

//********************************************************************************
//Returns an action 0, 1, 2 or 3
int getActionUno(){
  //Extremidad 1
  int actionUno;
  float valMaxUno = -10000000.0;
  float valUno;
  int aMaxUno;
  float randValUno;
  //-1 if action of the index takes you outside the state space.  +1 otherwise
  int allowedActions1[4] = {-1, -1, -1, -1}; 
  boolean randomActionFound1 = false;
//-----------------------------------Extremidad 1-----------------------------------
  if((Ls1 + 1) != numLTheta1States){
    allowedActions1[0] = 1;
    valUno = Q1[s1][0];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
      aMaxUno = 0;
    }
  }
    if(Ls1 != 0){
    allowedActions1[1] = 1;
    valUno = Q1[s1][1];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
      aMaxUno = 1;
    }
  }
  if((Ls2 + 1) != numLTheta2States){
    allowedActions1[2] = 1;
    valUno = Q1[s1][2];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
      aMaxUno = 2;
    }
  }
    if(Ls2 != 0){
    allowedActions1[3] = 1;
    valUno = Q1[s1][3];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
      aMaxUno = 3;
    }
  }

//------------------------------Extremidad 1---------------------------------------------  
  //implement epsilon greedy
  randValUno = float(random(0,101));
  if(randValUno < (1.0-epsilon)*100.0){    //choose the optimal action with probability 1-epsilon
    actionUno = aMaxUno;
  }else{
    while(!randomActionFound1){
      actionUno = int(random(0,4));        //otherwise pick an action between 0 and 3 randomly (inclusive), but don't use actions that take you outside the state-space
      if(allowedActions1[actionUno] == 1){
        randomActionFound1 = true;
      }
    }
  }    
  return(actionUno);
//---------------------------------------------------------------------------------------
}

int getActionDos(){
  //---------------------------------------Extremidad 2----------------------------------
  int actionDos;
  float valMaxDos = -10000000.0;
  float valDos;
  int aMaxDos;
  float randValDos;
  //-1 if action of the index takes you outside the state space.  +1 otherwise
  int allowedActions2[4] = {-1, -1, -1, -1}; 
  boolean randomActionFound2 = false;
  //--------------------------------------Extremidad 2----------------------------------------------
  if((Ds1 + 1) != numDTheta1States){
    allowedActions2[0] = 1;
    valDos = Q2[s2][0];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
      aMaxDos = 0;
    }
  }
    if(Ds1 != 0){
    allowedActions2[1] = 1;
    valDos = Q2[s2][1];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
      aMaxDos = 1;
    }
  }
  if((Ds2 + 1) != numDTheta2States){
    allowedActions2[2] = 1;
    valDos = Q2[s2][2];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
      aMaxDos = 2;
    }
  }
    if(Ds2 != 0){
    allowedActions2[3] = 1;
    valDos = Q2[s2][3];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
      aMaxDos = 3;
    }
  }
  //implement epsilon greedy
  randValDos = float(random(0,101));
  if(randValDos < (1.0-epsilon)*100.0){    //choose the optimal action with probability 1-epsilon
    actionDos = aMaxDos;
  }else{
    while(!randomActionFound2){
      actionDos = int(random(0,4));        //otherwise pick an action between 0 and 3 randomly (inclusive), but don't use actions that take you outside the state-space
      if(allowedActions2[actionDos] == 1){
        randomActionFound2 = true;
      }
    }
  }    
  return(actionDos);
}

int getActionCinco(){
  //------------------------------------Extremidad 5-----------------------------------
  int actionCinco;
  float valMaxCinco = -10000000.0;
  float valCinco;
  int aMaxCinco;
  float randValCinco;
  //-1 if action of the index takes you outside the state space.  +1 otherwise
  int allowedActions5[4] = {-1, -1, -1, -1}; 
  boolean randomActionFound5 = false;
  //----------------------------------Extremidad 5---------------------------------------
  if((Ls7 + 1) != numLTheta7States){
    allowedActions5[0] = 1;
    valCinco = Q5[s5][0];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
      aMaxCinco = 0;
    }
  }
    if(Ls7 != 0){
    allowedActions5[1] = 1;
    valCinco = Q5[s5][1];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
      aMaxCinco = 1;
    }
  }
  if((Ls8 + 1) != numLTheta8States){
    allowedActions5[2] = 1;
    valCinco = Q5[s5][2];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
      aMaxCinco = 2;
    }
  }
    if(Ls8 != 0){
    allowedActions5[3] = 1;
    valCinco = Q5[s5][3];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
      aMaxCinco = 3;
    }
  }
    //implement epsilon greedy
  randValCinco = float(random(0,101));
  if(randValCinco < (1.0-epsilon)*100.0){    //choose the optimal action with probability 1-epsilon
    actionCinco = aMaxCinco;
  }else{
    while(!randomActionFound5){
      actionCinco = int(random(0,4));        //otherwise pick an action between 0 and 3 randomly (inclusive), but don't use actions that take you outside the state-space
      if(allowedActions5[actionCinco] == 1){
        randomActionFound5 = true;
      }
    }
  }    
  return(actionCinco);
}

int getActionSeis(){
  //------------------------------------Extremidad 6------------------------------------
  int actionSeis;
  float valMaxSeis = -10000000.0;
  float valSeis;
  int aMaxSeis;
  float randValSeis;
  //-1 if action of the index takes you outside the state space.  +1 otherwise
  int allowedActions6[4] = {-1, -1, -1, -1}; 
  boolean randomActionFound6 = false;

  //find the optimal action.  Exclude actions that take you outside the allowed-state space.
//-----------------------------Extremidad 6------------------------------------------------
  if((Ds7 + 1) != numDTheta7States){
    allowedActions6[0] = 1;
    valSeis = Q6[s6][0];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
      aMaxSeis = 0;
    }
  }
    if(Ds7 != 0){
    allowedActions6[1] = 1;
    valSeis = Q6[s6][1];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
      aMaxSeis = 1;
    }
  }
  if((Ds8 + 1) != numDTheta8States){
    allowedActions6[2] = 1;
    valSeis = Q6[s6][2];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
      aMaxSeis = 2;
    }
  }
    if(Ds8 != 0){
    allowedActions6[3] = 1;
    valSeis = Q6[s6][3];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
      aMaxSeis = 3;
    }
  }
    //implement epsilon greedy
  randValSeis = float(random(0,101));
  if(randValSeis < (1.0-epsilon)*100.0){    //choose the optimal action with probability 1-epsilon
    actionSeis = aMaxSeis;
  }else{
    while(!randomActionFound6){
      actionSeis = int(random(0,4));        //otherwise pick an action between 0 and 3 randomly (inclusive), but don't use actions that take you outside the state-space
      if(allowedActions6[actionSeis] == 1){
        randomActionFound6 = true;
      }
    }
  }    
  return(actionSeis);
}

//Extremidad 1
//Given a and the global(s) find the next state.  Also keep track of the individual joint indexes s1 and s2.
void setSPrimeUno(int actionUno){  
  if (actionUno == 0){
    //joint1++
    s1Prime = s1 + numLTheta2States;
    Ls1++;
  }else if (actionUno == 1){
    //joint1--
    s1Prime = s1 - numLTheta2States;
    Ls1--;
  }else if (actionUno == 2){
    //joint2++
    s1Prime = s1 + 1;
    Ls2++;
  }else{
    //joint2--
    s1Prime = s1 - 1;
    Ls2--;
  }
}

//Extremidad 2
//Given a and the global(s) find the next state.  Also keep track of the individual joint indexes s1 and s2.
void setSPrimeDos(int actionDos){  
  if (actionDos == 0){
    //joint1++
    s2Prime = s2 + numDTheta2States;
    Ds1++;
  }else if (actionDos == 1){
    //joint1--
    s2Prime = s2 - numDTheta2States;
    Ds1--;
  }else if (actionDos == 2){
    //joint2++
    s2Prime = s2 + 1;
    Ds2++;
  }else{
    //joint2--
    s2Prime = s2 - 1;
    Ds2--;
  }
}

//Extremidad 5
//Given a and the global(s) find the next state.  Also keep track of the individual joint indexes s1 and s2.
void setSPrimeCinco(int actionCinco){  
  if (actionCinco == 0){
    //joint1++
    s5Prime = s5 + numLTheta8States;
    Ls7++;
  }else if (actionCinco == 1){
    //joint1--
    s5Prime = s5 - numLTheta8States;
    Ls7--;
  }else if (actionCinco == 2){
    //joint2++
    s5Prime = s5 + 1;
    Ls8++;
  }else{
    //joint2--
    s5Prime = s5 - 1;
    Ls8--;
  }
}

//Extremidad 6
//Given a and the global(s) find the next state.  Also keep track of the individual joint indexes s1 and s2.
void setSPrimeSeis(int actionSeis){  
  if (actionSeis == 0){
    //joint1++
    s6Prime = s6 + numDTheta8States;
    Ds7++;
  }else if (actionSeis == 1){
    //joint1--
    s6Prime = s6 - numDTheta8States;
    Ds7--;
  }else if (actionSeis == 2){
    //joint2++
    s6Prime = s6 + 1;
    Ds8++;
  }else{
    //joint2--
    s6Prime = s6 - 1;
    Ds8--;
  }
}

//-----------------------Extremidad 1---------------------------------------------------
//Update the position of the servos (this is the physical state transition command)
void setPhysicalStateUno(int actionUno){
  float currentAngleUno;
  float finalAngleUno;
  if (actionUno == 0){
    //currentAngleUno = servo1.read();
    currentAngleUno = AnguloAnteriorL1;
    finalAngleUno = currentAngleUno + deltaLTheta1;
    AnguloAnteriorL1 = finalAngleUno;
    //servo1.write(finalAngle);
    setServoL(1,finalAngleUno);
    delay(delayTimeL1);
  }else if (actionUno == 1){
    //currentAngle = servo1.read();
    currentAngleUno = AnguloAnteriorL1;
    finalAngleUno = currentAngleUno - deltaLTheta1;
    AnguloAnteriorL1 = finalAngleUno;
    //servo1.write(finalAngle);
    setServoL(1,finalAngleUno);
    delay(delayTimeL1);
  }else if (actionUno == 2){
    //currentAngle = servo2.read();
    currentAngleUno = AnguloAnteriorL2;
    finalAngleUno = currentAngleUno + deltaLTheta2;
    AnguloAnteriorL2 = finalAngleUno;
    //servo2.write(finalAngle);
    setServoL(2,finalAngleUno);
    delay(delayTimeL2);
  }else{
    //currentAngle = servo2.read();
    currentAngleUno = AnguloAnteriorL2;
    finalAngleUno = currentAngleUno - deltaLTheta2;
    AnguloAnteriorL2 = finalAngleUno;
    //servo2.write(finalAngle);
    setServoL(2,finalAngleUno);
    delay(delayTimeL2);
  }
}

//----------------------------Extremidad 2------------------------------------------
//Update the position of the servos (this is the physical state transition command)
void setPhysicalStateDos(int actionDos){
  float currentAngleDos;
  float finalAngleDos;
  if (actionDos == 0){
    //currentAngleUno = servo1.read();
    currentAngleDos = AnguloAnteriorD1;
    finalAngleDos = currentAngleDos + deltaDTheta1;
    AnguloAnteriorD1 = finalAngleDos;
    //servo1.write(finalAngle);
    setServoR(1,finalAngleDos);
    delay(delayTimeD1);
  }else if (actionDos == 1){
    //currentAngle = servo1.read();
    currentAngleDos = AnguloAnteriorD1;
    finalAngleDos = currentAngleDos - deltaDTheta1;
    AnguloAnteriorD1 = finalAngleDos;
    //servo1.write(finalAngle);
    setServoR(1,finalAngleDos);
    delay(delayTimeD1);
  }else if (actionDos == 2){
    //currentAngle = servo2.read();
    currentAngleDos = AnguloAnteriorD2;
    finalAngleDos = currentAngleDos + deltaDTheta2;
    AnguloAnteriorD2 = finalAngleDos;
    //servo2.write(finalAngle);
    setServoR(2,finalAngleDos);
    delay(delayTimeD2);
  }else{
    //currentAngle = servo2.read();
    currentAngleDos = AnguloAnteriorD2;
    finalAngleDos = currentAngleDos - deltaDTheta2;
    AnguloAnteriorD2 = finalAngleDos;
    //servo2.write(finalAngle);
    setServoR(2,finalAngleDos);
    delay(delayTimeD2);
  }
}

//--------------------------------Extremidad 5-----------------------------------------
//Update the position of the servos (this is the physical state transition command)
void setPhysicalStateCinco(int actionCinco){
  float currentAngleCinco;
  float finalAngleCinco;
  if (actionCinco == 0){
    //currentAngleUno = servo1.read();
    currentAngleCinco = AnguloAnteriorL7;
    finalAngleCinco = currentAngleCinco + deltaLTheta7;
    AnguloAnteriorL7 = finalAngleCinco;
    //servo1.write(finalAngle);
    setServoL(7,finalAngleCinco);
    delay(delayTimeL7);
  }else if (actionCinco == 1){
    //currentAngle = servo1.read();
    currentAngleCinco = AnguloAnteriorL7;
    finalAngleCinco = currentAngleCinco - deltaLTheta7;
    AnguloAnteriorL7 = finalAngleCinco;
    //servo1.write(finalAngle);
    setServoL(7,finalAngleCinco);
    delay(delayTimeL7);
  }else if (actionCinco == 2){
    //currentAngle = servo2.read();
    currentAngleCinco = AnguloAnteriorL8;
    finalAngleCinco = currentAngleCinco + deltaLTheta8;
    AnguloAnteriorL8 = finalAngleCinco;
    //servo2.write(finalAngle);
    setServoL(8,finalAngleCinco);
    delay(delayTimeL8);
  }else{
    //currentAngle = servo2.read();
    currentAngleCinco = AnguloAnteriorL8;
    finalAngleCinco = currentAngleCinco - deltaLTheta8;
    AnguloAnteriorL8 = finalAngleCinco;
    //servo2.write(finalAngle);
    setServoL(8,finalAngleCinco);
    delay(delayTimeL8);
  }
}
//--------------------------------Extremidad 6------------------------------------------
//Update the position of the servos (this is the physical state transition command)
void setPhysicalStateSeis(int actionSeis){
  float currentAngleSeis;
  float finalAngleSeis;
  if (actionSeis == 0){
    //currentAngleUno = servo1.read();
    currentAngleSeis = AnguloAnteriorD7;
    finalAngleSeis = currentAngleSeis + deltaDTheta7;
    AnguloAnteriorD7 = finalAngleSeis;
    //servo1.write(finalAngle);
    setServoR(7,finalAngleSeis);
    delay(delayTimeD7);
  }else if (actionSeis == 1){
    //currentAngle = servo1.read();
    currentAngleSeis = AnguloAnteriorD7;
    finalAngleSeis = currentAngleSeis - deltaDTheta7;
    AnguloAnteriorD7 = finalAngleSeis;
    //servo1.write(finalAngle);
    setServoR(7,finalAngleSeis);
    delay(delayTimeD7);
  }else if (actionSeis == 2){
    //currentAngle = servo2.read();
    currentAngleSeis = AnguloAnteriorD8;
    finalAngleSeis = currentAngleSeis + deltaDTheta8;
    AnguloAnteriorD8 = finalAngleSeis;
    //servo2.write(finalAngle);
    setServoR(8,finalAngleSeis);
    delay(delayTimeD8);
  }else{
    //currentAngle = servo2.read();
    currentAngleSeis = AnguloAnteriorD8;
    finalAngleSeis = currentAngleSeis - deltaDTheta8;
    AnguloAnteriorD8 = finalAngleSeis;
    //servo2.write(finalAngle);
    setServoR(8,finalAngleSeis);
    delay(delayTimeD8);
  }
}
//---------------------------------------------------------------------

//Get the reward using the distance the agent has moved since the last call
float getDeltaDistanceRolled(){
  //get current distance
  distanceNew = float(ultrasonic.ping());   
  deltaDistance = distanceNew - distanceOld;
  if (abs(deltaDistance) < 57.0 || abs(deltaDistance) > 230.0){         //don't count noise
    deltaDistance = 0.0;
  }
  distanceOld = distanceNew;
  return deltaDistance;
}

//--------------------------------------------------------------------

//-------------------------------Extremidad 1--------------------------------------------
//Get max over a' of Q(s',a'), but be careful not to look at actions which take the agent outside of the allowed state space
float getLookAheadUno(){
  float valMaxUno = -100000.0;
  float valUno;
  if((Ls1 + 1) != numLTheta1States){
    valUno = Q1[s1Prime][0];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
    }
  }
  if(Ls1 != 0){
    valUno = Q1[s1Prime][1];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
    }
  }
  if((Ls2 + 1) != numLTheta2States){
    valUno = Q1[s1Prime][2];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
    }
  }
  if(Ls2 != 0){
    valUno = Q1[s1Prime][3];
    if(valUno > valMaxUno){
      valMaxUno = valUno;
    }
  }
  return valMaxUno;
}

//--------------------------------Extremidad 2-------------------------------------------
//Get max over a' of Q(s',a'), but be careful not to look at actions which take the agent outside of the allowed state space
float getLookAheadDos(){
  float valMaxDos = -100000.0;
  float valDos;
  if((Ds1 + 1) != numDTheta1States){
    valDos = Q2[s2Prime][0];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
    }
  }
  if(Ds1 != 0){
    valDos = Q2[s2Prime][1];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
    }
  }
  if((Ds2 + 1) != numDTheta2States){
    valDos = Q2[s2Prime][2];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
    }
  }
  if(Ds2 != 0){
    valDos = Q2[s2Prime][3];
    if(valDos > valMaxDos){
      valMaxDos = valDos;
    }
  }
  return valMaxDos;
}

//--------------------------------Extremidad 5------------------------------------------
//Get max over a' of Q(s',a'), but be careful not to look at actions which take the agent outside of the allowed state space
float getLookAheadCinco(){
  float valMaxCinco = -100000.0;
  float valCinco;
  if((Ls7 + 1) != numLTheta7States){
    valCinco = Q5[s5Prime][0];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
    }
  }
  if(Ls7 != 0){
    valCinco = Q5[s5Prime][1];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
    }
  }
  if((Ls8 + 1) != numLTheta8States){
    valCinco = Q5[s5Prime][2];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
    }
  }
  if(Ls8 != 0){
    valCinco = Q5[s5Prime][3];
    if(valCinco > valMaxCinco){
      valMaxCinco = valCinco;
    }
  }
  return valMaxCinco;
}

//--------------------------------Extremidad 6-------------------------------------------
//Get max over a' of Q(s',a'), but be careful not to look at actions which take the agent outside of the allowed state space
float getLookAheadSeis(){
  float valMaxSeis = -100000.0;
  float valSeis;
  if((Ds7 + 1) != numDTheta7States){
    valSeis = Q6[s6Prime][0];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
    }
  }
  if(Ds7 != 0){
    valSeis = Q6[s6Prime][1];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
    }
  }
  if((Ds8 + 1) != numDTheta8States){
    valSeis = Q6[s6Prime][2];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
    }
  }
  if(Ds8 != 0){
    valSeis = Q6[s6Prime][3];
    if(valSeis > valMaxSeis){
      valMaxSeis = valSeis;
    }
  }
  return valMaxSeis;
}
//----------------------------------------------------------------------
void printQUno(){
  for(int iUno=0; iUno<numStates1; iUno++){
    for(int jUno=0; jUno<numActions1; jUno++){
      Serial.print(Q1[iUno][jUno]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  Serial.println(" ");
}


//-------------------------------------------------------------------------------
//Iniciar Q de extremidad 1
void initializeQ1(){
  for(int iUno=0; iUno<numStates1; iUno++){
    for(int jUno=0; jUno<numActions1; jUno++){
      Q1[iUno][jUno] = 10.0;               //Initialize to a positive number to represent optimism over all state-actions
    }
  }
}

//Iniciar Q de extremidad 2
void initializeQ2(){
  for(int iDos=0; iDos<numStates2; iDos++){
    for(int jDos=0; jDos<numActions2; jDos++){
      Q2[iDos][jDos] = 10.0;               //Initialize to a positive number to represent optimism over all state-actions
    }
  }
}

//Iniciar Q de extremidad 5
void initializeQ5(){
  for(int iCinco=0; iCinco<numStates5; iCinco++){
    for(int jCinco=0; jCinco<numActions5; jCinco++){
      Q5[iCinco][jCinco] = 10.0;               //Initialize to a positive number to represent optimism over all state-actions
    }
  }
}

//Iniciar Q de extremidad 6
void initializeQ6(){
  for(int iSeis=0; iSeis<numStates6; iSeis++){
    for(int jSeis=0; jSeis<numActions6; jSeis++){
      Q6[iSeis][jSeis] = 10.0;               //Initialize to a positive number to represent optimism over all state-actions
    }
  }
}

/////////////////////////////////////////////////////////////////////////////
//****Remains the same
const int rollDelay = 200;                   //allow time for the agent to roll after it sets its physical state
const float explorationMinutes = 1.0;        //the desired exploration time in minutes 
const float explorationConst = (explorationMinutes*60.0)/((float(rollDelay))/1000.0);  //this is the approximate exploration time in units of number of times through the loop

int t = 0;
void loop(){
  t++;
  epsilon = exp(-float(t)/explorationConst);
  //Serial.println(distanceNew);
  //Serial.println(" ");
  Serial.println(deltaDistance);
  //printQUno();
  //**Remains the same
  //Changes for each leg
  a = getActionUno();           //a is beween 0 and 3
  b = getActionDos();
  c = getActionCinco();
  d = getActionSeis();
  
  setSPrimeUno(a);              //this also updates s1 and s2.
  setSPrimeDos(b);
  setSPrimeCinco(c);
  setSPrimeSeis(d);
  
  setPhysicalStateUno(a);
  setPhysicalStateDos(b);
  setPhysicalStateCinco(c);
  setPhysicalStateSeis(d);
  //put a delay after the physical action occurs so the agent has time to move/roll 
  //before measuring the new position (before calling getDeltaDistanceRolled
  delay(rollDelay);                     
  
  r = getDeltaDistanceRolled();//-----
  
  lookAheadValueUno = getLookAheadUno();
  lookAheadValueDos = getLookAheadDos();
  lookAheadValueCinco = getLookAheadCinco();
  lookAheadValueSeis = getLookAheadSeis();
  
  sampleUno = r + gamma*lookAheadValueUno;
  sampleDos = r + gamma*lookAheadValueDos;
  sampleCinco = r + gamma*lookAheadValueCinco;
  sampleSeis = r + gamma*lookAheadValueSeis;
  
  Q1[s1][a] = Q1[s1][a] + alpha*(sampleUno - Q1[s1][a]);
  Q2[s2][b] = Q2[s2][b] + alpha*(sampleDos - Q2[s2][b]);
  Q5[s5][c] = Q5[s5][c] + alpha*(sampleCinco - Q5[s5][c]);
  Q6[s6][d] = Q6[s6][d] + alpha*(sampleSeis - Q6[s6][d]);
  
  s1 = s1Prime;
  s2 = s2Prime;
  s5 = s5Prime;
  s6 = s6Prime;
  
  if(t == 2){                //need to reset Q at the beginning since a spurious value arises at the first initialization (something from the rangefinder..)
    initializeQ1();
    initializeQ2();
    initializeQ5();
    initializeQ6();
  }
  
  //if(t == int(explorationConst)){
  //  printQ();
  //}
}
