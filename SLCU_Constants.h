/*File: SLCU_Constants.h
	Brief: Constantes du SLCU/PDS
	Date: 27 janvier 2024
	Author: Simon Jourdenais
*/

#ifndef SLCU_CONSTANTS_H_
#define SLCU_CONSTANTS_H_
//########################
#define RAYON 0.15
//########################


#define DELAY_DEBOUNCE 250
#define COMM_DELAY_PERIOD 100 //Delays in ms before checking if communication arrived again
#define UPDATE_PERIOD 100 //Delays in ms before checking if communication arrived again
#define BAUD_RATE 115200

#define GATE_A 2 
#define GATE_B 3
#define GATE_C 4
#define rotation_pulses 5 /*Input for drop sequence*/

// Define PID constants
#define KP 0.1
#define KI 0.01
#define KD 0.001
#define SERVOPin  14

enum States
{
  GO, // Mode a été intialisé (TESTING ou FLIGHT) et le Launch Systems Check est Clear
  NOGO,
  OVERRIDDEN //Si le systeme dit no go pour quelconque raison et qu'on doit forcer la mise en fonction du reste du système ( En NOGO, pas de drop disponible et si on est dans un mode autre que NONE, alarme crie)
};

enum Sequence
{
  STANDBY=0, //En attente de commande "DROP" avec A B ou C et la hauteur fonction: lit le serial port en continue
  ENGAGE, //A partir de quand on envoie signal drop jusqua confirmation porte ouverte 
  MONITOR, // Debut Porte ouverte et commence monitoring speed and height and accel (Limit Force pulling down when breaking or impact speed...)
  CONFIRM, // Quand les signaux concordent avec le fait que la charge est relachée
  FAULT, // Obvious
  SIM //pour Simulation -- Developpement
};

enum Mode
{
  NONE=-1,
  TESTING=0,
  FLIGHT
};
enum Commands
{
  LSC,
  DROP,
  OVRD,
  GCSO,
  CANCEL,
  DBG
};
#endif /*SLCU_CONSTANTS_H_*/