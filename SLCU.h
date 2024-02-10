/*File: SLCU.h
	Brief: Header file de la classe Arduino SLCU
	Date: 28 janvier 2024
	Authors: Simon Jourdenais, David Mihai Kibos
*/

#ifndef SLCU_H_
#define SLCU_H_

#include <Arduino.h>
#include "ArduinoJson.h"
#include "SLCU_Constants.h"

class SLCU
{
  private: //Prototype of private functions and vars
    States Launch_Status; //Launch Status : GO or NOGO or OVERRIDDEN
    Mode Op_Mode; //Operation mode : Flight or Test or NONE (Intialized only)
    Sequence Staging; //Delivery Staging :Standby, Engaging, Monitoring, Confirmed, FAULT, SIM
    StaticJsonDocument<200> trame_Pi; 
    double terminal_Altitude = 1.50;


  public: //Prototypes of public functions and variables

    SLCU(); //Prototype of Constructor
    uint8_t checkSerial(); 
    const StaticJsonDocument<200>& getTramePi() const {
        return trame_Pi;
    }
    void execute_LSC();
    void override();
    void decode_CMD();
    void sendToGCSO(String message);
    void cancelDelivery();
    void debugging();
    void drop(const char* Iden_Bottle, float height);
    bool initialized = false;
    Commands processCommand(); 
    void init(int baudRate);

    double getTerminalAlt() { return terminal_Altitude; };
    void setTerminalAlt() { terminal_Altitude = trame_Pi["PARAMS"][1]; };    

    States getLaunchStatus() { return Launch_Status; };
    void setLaunchStatus(States newStatus) { Launch_Status = newStatus; };    

    Sequence getStagingStatus() { return Staging; };
    void setStatingStatus(Sequence newStaging) { Staging = newStaging; };   

    Mode getOperationMode() { return Op_Mode; };
    void setOperationMode(Mode newMode) { Op_Mode = newMode; };    
};

#endif /*SLCU_H_*/