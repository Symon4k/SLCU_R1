/*File: Algo_SLCU.ino
	Brief: Programme principal du Système de Livraison de la Charge Utile (SLCU) du UAV du VAMUdeS conçu pour lacompétition américaine SUAS 2024.
  (Eng) Main program for the Payload Distribution System (PDS) for the VAMUdeS UAV designed for the SUAS 2024 US competition.
	Date: 28 janvier 2024
	Authors: Simon Jourdenais, David Mihai Kibos were here
  Revision: 1.2
*/
#include "SLCU.h"
#include "SLCU_Constants.h"
 
SLCU g_SLCU;

void setup()
{
  g_SLCU.init(BAUD_RATE);
};
 

void loop()
{
  static long int mainTimer = millis();
  Commands CMD;

  while(g_SLCU.initialized==true)
  {
    if( (millis()-mainTimer) < UPDATE_PERIOD)
    {
      delayMicroseconds(5); //Do Nothing while Not busy
    }
    else if(!g_SLCU.checkSerial())
    {

      //MEF Generale (Gere status de launch et demarrage des systemes, alertes s'il y a lieu et les commandes forcées)
      switch(g_SLCU.getStagingStatus())
      {
          case STANDBY:
          switch(g_SLCU.processCommand())
          {
            case LSC:
              g_SLCU.execute_LSC(); 
              break;
            case OVRD:
              //Override pour passer par dessus la raison du NOGO
              //if parametre Override == skip LSC en erreur -> Skip le LSC et set le mode TESTING si autre param = 0 et Flight si 1 ( usually 1 when using override command)
              break;
            case GCSO:
              //GSCO pourrait demander status ou all pour savoir wtf is going (Why is system in NOGO)
              break;
            case DBG:
              //DBG peux faire allumer la LED.. permet de debugger (Premier param == "LED" et 2eme param = freq en HZ (0 == on, -1 off).
              //Sinon, fait printer le texte envoyer en params + le 2eme param a la fin du texte
              break;
            case CANCEL:
              g_SLCU.sendToGCSO("System is in NO-GO ! Command ignored");
              break;
            case DROP:

              
              const StaticJsonDocument<200>& trame_Pi=g_SLCU.getTramePi();
              g_SLCU.sendToGCSO("Drop procedure started");
              g_SLCU.delivery_Sequence(trame_Pi);
              break;
            default:
              g_SLCU.sendToGCSO("Command not recognized");
              break;
          }
          break;
        case ENGAGE:
          //Active Servo X pour laisser la porte ouvrir
          //relache brake (PID avec commande echelon)
          //attends un total de 3 secondes pour confirmation de poids/porte ouverte et switch a etat MONITOR
          // Si 3 secondes et pas signaux de distribution en cours (Poids/Porte) -> etat FAULT
          break;
        case MONITOR:
          //Track vitesse et altitude
          //quand setpoint arrive proche (t-2metre disons pour commencer), ralentir jusqua vitesse 0
          //couper le thether (couper le fil)
          //attends un total de ~10 secondes (a tester/iterer) pour confirmation de poids/porte fermé et switch a etat CONFIRM
          //Si signaux contradictoire sendToGSCO message pour qu'il fasse override ( OVRD avec CUT comme param pour ressayer de couper le fil )  ou cancel pour annuler l'alerte et continuer comme si c'est ok
          break;
        case CONFIRM:
          //Envoie signal au GCSO et a l'ordinateur de bord que la séquence de livraison est terminée
          break;
        case FAULT:
          //Envoie au GCSO qu'il y a une faute, l'alarme sonore est activée
          //Le GCSO doit alors envoyer OVRD ou CANCEL dependemment de l'erreur
          break;
        case SIM:
          //Developpement purposes
          break;
        default:
          break;
      }
    };
  }
};
 



 