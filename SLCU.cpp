/*File: SLCU.cpp
	Brief: Classe Arduino du SLCU/PDS
	Date: 28 janvier 2024
	Authors: Simon Jourdenais, David Mihai Kibos
*/
#include "SLCU.h"
#include "SLCU_Constants.h"

SLCU::SLCU(){}

void SLCU::init(int baudRate)
{
  Serial.begin(baudRate);
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
    return 0;
  }
};


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
};