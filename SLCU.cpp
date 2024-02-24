/*File: SLCU.cpp
	Brief: Classe Arduino du SLCU/PDS
	Date: 28 janvier 2024
	Authors: Simon Jourdenais, David Mihai Kibos
*/
#include "SLCU.h"
#include "SLCU_Constants.h"
#include <Servo.h>

Servo servo_A;

SLCU::SLCU(){}

void SLCU::init(int baudRate) //FUNCTION DECLARED TWICE
{
  Serial.begin(baudRate);


  pinMode(GATE_A,OUTPUT);// drop // renmplacer les gates par les servo (aller voir la librarie servo.h) les servo prennent un pwm
  pinMode(GATE_B,OUTPUT);// drop // mux 32 1 2 la sortie des cerveau vont être 9 11 l'entrée
  pinMode(GATE_C,OUTPUT);// drop 
  pinMode(rotation_pulses, INPUT);
  pinMode(SERVOPin, INPUT);

  initialized = true;
  Launch_Status = NOGO; 
  Op_Mode = NONE; 
  Staging = STANDBY;
}


/*
* Nom: checkSerial 
* Brief: Verifie si une commande serie est recue et si le json est bon, l'enregistre dans trame_Pi
* Params: 
* Return: 0 si tout c'est bien passé, si une erreur, retourne -1
*/
uint8_t SLCU::checkSerial() 
{
  if(Serial.available()>0)
  {
    DeserializationError error = deserializeJson(trame_Pi, Serial.readStringUntil("\n"));

      if (error)
      {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return -1;
      }
        
  }
  return 0;    
}

/*
* Nom: processCommand 
* Brief: Decode la commande envoyée et execute la commande associée
* Params: 
* Return: retourne la valeur de Commande equivalente a la commande recue ; permet utilisation de switch cases..
*/
Commands SLCU::processCommand() 
{

    if (!strcmp(trame_Pi["CMD"], "LSC"))
    {
        return LSC;
    }
    else if (!strcmp(trame_Pi["CMD"], "OVRD"))
    {
        return OVRD;
    }
    else if (!strcmp(trame_Pi["CMD"], "GCSO"))
    {
        return GCSO;
    }
    else if (!strcmp(trame_Pi["CMD"], "CANCEL"))
    {
        return CANCEL;
    }
    else if (!strcmp(trame_Pi["CMD"], "DBG"))
    {
        return DBG;
    }
    else if (!strcmp(trame_Pi["CMD"], "DROP"))
    {
        return DROP;
    }
    else
    {
      Serial.println("Commande invalide");
      return -1;
    }
}


/*
* Nom: execute_LSC
* Brief:
* Params:
* Return: 
*/
void SLCU::execute_LSC()
{
  if(Op_Mode == NONE)
  {
    sendToGCSO("Executing LSC");
    if(!strcmp(trame_Pi["PARAMS"][0], "TESTING"))
    {
      //Verify systems under tests are GO and set mode to Testing if positive
      Op_Mode = TESTING;
    }
    else if(!strcmp(trame_Pi["PARAMS"][0], "FLIGHT"))
    {
      //Verify that All systems are GO and if yes, Set OP Mode to Flight
      Op_Mode = FLIGHT;
    }
  }
  else
  {
    sendToGCSO("Invalid: Command Ignored.");
  }

};

/*
* Nom: override
* Brief:
* Params:
* Return: 
*/
void SLCU::override()
{
  if(trame_Pi["PARAMS"][0]=="MOD_ELEV" && trame_Pi["PARAMS"][0] >= 0.5)
  {
    setTerminalAlt();
    Serial.println("Override Terminal Payload Altitude - Jettisonning Payload at "+String(terminal_Altitude)+" meters");
  }
  else
    sendToGCSO("Altitude value too low - Risks of impact with target !");
};


/*
* Nom: sendToGCSO
* Brief:
* Params:
* Return: 
*/
void SLCU::sendToGCSO(String message)
{
  Serial.println("Send To GCSO the following message");
};

/*
* Nom: cancelDelivery
* Brief:
* Params:
* Return: 
*/
void SLCU::cancelDelivery()
{
  sendToGCSO("Cancel Delivery");
}

/*
* Nom: debugging
* Brief:
* Params:
* Return: 
*/
void SLCU::debugging()
{
  Serial.println("Debugging");
}




float integral = 0; // Needs to be initially set to 0=> declared outside the function 
float previous_error = 0; 
// PWM calculation to slow down the bottle
float calculatePID(float setpoint, float current_value) {
    float error = setpoint - current_value;
    integral = integral + error;
    float derivative = error - previous_error;
    float output = KP * error + KI * integral + KD * derivative;
    previous_error = error;
    return output;
}

// Modify the delivery sequence function
void SLCU::delivery_Sequence() {
  // Variables are initialized once
  static float startTime;
  static float endTime; 
  static float current_distance = 0;
  static float final_altitude = 1;

  //PID Variables
 
  float current_speed;
  bool risingEdgeDetected = false;

  int previousInputState;
  // Extract trame params 
  const char* bottleIdentifier = trame_Pi["PARAMS"][0]; 
  char bottleChar = bottleIdentifier[0];
  float height = trame_Pi["PARAMS"][1];

  switch (bottleChar) {
    case 'A':
        digitalWrite(GATE_A, HIGH);
        break;
    case 'B':
        digitalWrite(GATE_B, HIGH);
        break;
    case 'C':
        digitalWrite(GATE_C, HIGH);
        break;
  }

  Serial.println("Gate " + String(bottleIdentifier) + " opened; Initial Altitude: " + String(height));

  int inputState = digitalRead(rotation_pulses);
  
  servo_A.attach(SERVOPin); // Attach servo to the pin
  
  while (height - current_distance > final_altitude) {
    // Read the state of the input pin
    previousInputState = inputState;
    inputState = digitalRead(rotation_pulses); // Removed int declaration here

    if (inputState == HIGH && previousInputState == LOW && !risingEdgeDetected) // For the first rotational pulse detected
    {
        startTime = millis();
        risingEdgeDetected = true;
    }

    if (inputState == HIGH && previousInputState == LOW && risingEdgeDetected) 
    {
        endTime = millis();
        unsigned long timeBetweenEdges = endTime - startTime;

        current_speed = (M_PI * (RAYON / 2)) / (timeBetweenEdges / 1000.0); 
        current_distance += (M_PI/2) * RAYON; 

        // Calculate PID control output to adjust speed
        float pid_output = calculatePID(final_altitude, height - current_distance);
        int servo_speed = map(pid_output, -255, 255, 0, 180); // Map PID output to servo position

       /* // Ensure servo_speed is within valid range for servo
        if (servo_speed < 0) {
            servo_speed = 0;
        } else if (servo_speed > 180) {
            servo_speed = 180;
        }*/

        servo_A.write(servo_speed); // Set servo speed
        // Print the calculated values
        Serial.print("Speed=");
        Serial.println(current_speed); 
        Serial.print("Payload altitude = ");
        Serial.println(String(height - current_distance));
        startTime = endTime;
    }
  }
}



/*void SLCU::delivery_Sequence() // version original pour backup
{
  // Variables are initilized once
  static float startTime;
  static float endTime; 
  static float current_distance = 0;
  
  float current_speed;
  bool risingEdgeDetected = false;
  // Extract trame params 
  const char* bottleIdentifier = trame_Pi["PARAMS"][0]; 
  char bottleChar = bottleIdentifier[0];
  float height = trame_Pi["PARAMS"][1];

  switch (bottleChar) {
    case 'A':
        digitalWrite(GATE_A, HIGH);
        break;
    case 'B':
        digitalWrite(GATE_B, HIGH);
        break;
    case 'C':
        digitalWrite(GATE_C, HIGH);
        break;
  }

  Serial.println("Gate " + String(bottleIdentifier) + " opened; Initial Altitude: " + String(height));

  int inputState = digitalRead(rotation_pulses);
  
  while (current_distance < height)

  {
    // Read the state of the input pin
    int previousInputState = inputState;
    inputState = digitalRead(rotation_pulses); // Removed int declaration here

    if (inputState == HIGH && previousInputState == LOW) 
    {
        startTime = millis();
        risingEdgeDetected = true;
    }

    if (inputState == HIGH && previousInputState == LOW && risingEdgeDetected) 
    {
        endTime = millis();
        unsigned long timeBetweenEdges = endTime - startTime;

        current_speed = (M_PI * (RAYON / 2)) / (timeBetweenEdges / 1000.0); 
        current_distance += (M_PI/2) * RAYON; 

        // Print the calculated values
        Serial.print("Speed=");
        Serial.println(current_speed * 100); 
        Serial.print("Distance Traveled = ");
        Serial.println(current_distance);

        risingEdgeDetected = false; // à verifier pour faire un calcul à chaque pulse 
    }
  }
}*/


 