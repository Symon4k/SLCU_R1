/*File: SLCU.cpp
	Brief: Classe Arduino du SLCU/PDS
	Date: 28 janvier 2024
	Authors: Simon Jourdenais, David Mihai Kibos
   {"CMD":"DROP","PARAMS":["A", 3]}
*/
#include "SLCU.h"
#include "SLCU_Constants.h"
#include <Servo.h>

Servo servo_A ;
Servo servomoteur;

SLCU::SLCU(){}

void SLCU::init(int baudRate) //FUNCTION DECLARED TWICE
{
  Serial.begin(baudRate);



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
      bool system_check = true;
      //Verify systems under tests are GO and set mode to Testing if positive
      int system_pins[] = {demux1, demux2, wire_A, wire_B, wire_C};
      int num_pins = sizeof(system_pins) / sizeof(system_pins[0]); // Calculate the number of pins

      int expected_states[] = {LOW,LOW,LOW,LOW,LOW,LOW}; //input the default state for each pin 
     
      for (int i = 0; i < num_pins; i++) 
      {
          int actual_state = digitalRead(system_pins[i]); // Read the actual state of the pin
          if (actual_state == expected_states[i]) 
          {
              Serial.print("Pin ");
              Serial.print(system_pins[i]);
              Serial.println(" is in the correct state.");
          } 
          else 
          {
              Serial.print("Pin ");
              Serial.print(system_pins[i]);
              Serial.println(" is NOT in the correct state.");
              system_check = false;
          }
      }
      Op_Mode = TESTING;

      if (system_check == true)
      {
        Serial.println("System check ok");
      }
      else if (system_check == false)
      {
        Serial.println("System no ready");
      }
    
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
   else if(trame_Pi["PARAMS"][0]=="IGNORE" )
  {
     digitalWrite(sound_alarm, LOW);
     Serial.println("Alarm desactivated");
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

  if(trame_Pi["PARAMS"][0]=="STATUS")
  {
    /*if(g_SLCU.getLaunchStatus()=="Go")
    {
      //return the lauch status SEQUENCE
    }*/
  }
  if(trame_Pi["PARAMS"][0]=="ALL")
  {
   
    
  }

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
  const char* bottleIdentifier = trame_Pi["PARAMS"][0]; 
  char bottleChar = bottleIdentifier[0];
  float height = trame_Pi["PARAMS"][1];


  if (bottleIdentifier == 'A') 
  {
    digitalWrite(wire_A, HIGH);
  } 
  else if (bottleIdentifier == 'B') 
  {
    digitalWrite(wire_B, HIGH);
  } 
  else if (bottleIdentifier == 'C') 
  {
    digitalWrite(wire_C, HIGH);
  }
  //Cutting wire
  Serial.println("Wire " + String(bottleIdentifier) + " cut ");

  //Need to check if wire is cutted

  //Closing the gate
  digitalWrite(demux1, LOW);
  digitalWrite(demux2,HIGH);
  Serial.println("Gates closed");




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
    int it=0;
    String param = trame_Pi["PARAMS"][0];
    float param_num = trame_Pi["PARAMS"][1];

  int system_pins[] = {demux1, demux2, wire_A, wire_B, wire_C}; 
    int num_pins = sizeof(system_pins) / sizeof(system_pins[0]); // Calculate the number of pins

    for (int i = 0; i < num_pins; i++) {
      String pinName = String(system_pins[i]);  // Convert pin number to string
      Serial.print(pinName);
      Serial.print(": ");
      Serial.println(digitalRead(system_pins[i]));

      
    }
    if (param == "LED")
  {
  switch (int(param_num))
  {
      case 0:
        digitalWrite(ledPin, HIGH);

          break;
      case 1:
          while(it<1000)
          {
            digitalWrite(ledPin, HIGH); // Turn the LED on
            delay(500); // Wait for 500 milliseconds (0.5 seconds)
            digitalWrite(ledPin, LOW); // Turn the LED off
            delay(500); // Wait for 500 milliseconds (0.5 seconds)
            it=it+1;
          }
          break;
      case 2:
          while(it<1000)
          {
            digitalWrite(ledPin, HIGH); // Turn the LED on
            it=it+1;
          }
          break;
    
    }
  }
  
}




void SLCU::delivery_Sequence() {
  Serial.println("To the moon!!!!");
  // Variables are initialized once
  static float startTime;
  static float endTime; 
  static float current_distance = 0;
  static float final_altitude = 1;
  // Declare PID variables outside the function
  static float integral = 0; // Needs to be initially set to 0
  static float previous_error = 0;

  // Extract trame params 
  const char* bottleIdentifier = trame_Pi["PARAMS"][0]; 
  char bottleChar = bottleIdentifier[0];
  float height = trame_Pi["PARAMS"][1];

  switch (bottleChar) {
    case 'A':
        digitalWrite(demux1, LOW);
        digitalWrite(demux2,HIGH);
        break;
    case 'B':
        digitalWrite(demux1, HIGH);
        digitalWrite(demux2,LOW);
        break;
    case 'C':
        digitalWrite(demux1, HIGH);
        digitalWrite(demux2,HIGH);
        break;
  }

  Serial.println("Gate " + String(bottleIdentifier) + " opened; Initial Altitude: " + String(height));

  int inputState = digitalRead(rotation_pulses);
  
  servo_A.attach(SERVOPin); // Attach servo to the pin
  
  while (height - current_distance > final_altitude) {
    // Read the state of the input pin
    int previousInputState = inputState;
    inputState = digitalRead(rotation_pulses);

    if (inputState == HIGH && previousInputState == LOW) {
        startTime = millis();
    }

    if (inputState == HIGH && previousInputState == LOW) {
        endTime = millis();
        unsigned long timeBetweenEdges = endTime - startTime;

        float current_speed = (M_PI * (RAYON / 2)) / (timeBetweenEdges / 1000.0); 
        current_distance += (M_PI/2) * RAYON; 

        // Calculate PID control output to adjust speed


        float error = final_altitude - (height - current_distance);
        integral = integral + error;
        float derivative = error - previous_error;
        float output = KP * error + KI * integral + KD * derivative;
        previous_error = error;

        int servo_speed = map(output, -255, 255, 0, 180); // Map PID output to servo position

        // Ensure servo_speed is within valid range for servo
        if (servo_speed < 0) {
            servo_speed = 0;
        } else if (servo_speed > 180) {
            servo_speed = 180;
        }

        servo_A.write(servo_speed); // Set servo speed
        // Print the calculated values
        Serial.print("Speed=");
        Serial.println(current_speed); 
        Serial.print("Payload altitude = ");
        Serial.println(String(height - current_distance));
        startTime = endTime;

        switch (*bottleIdentifier) {
    case 'A':
        digitalWrite(wire_A, HIGH);
        break;
    case 'B':
        digitalWrite(wire_B, HIGH);
        break;
    case 'C':
        digitalWrite(wire_C, HIGH);
        break;
  }


    }
  }
}

