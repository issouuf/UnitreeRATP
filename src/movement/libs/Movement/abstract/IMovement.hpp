#pragma once
#include <string>

// @brief Interface purement abstraite pour la création et le contrôle d'un drone.
// @note Cette interface définit les méthodes de base pour la connexion, la déconnexion et le mouvement des drones.
// @note Elle est conçue pour être implémentée par des classes spécifiques à chaque type de drone.
// @note Voir Unitree/dogs/UnitreeGo1 pour une implémentation concrète.
class IMovement 
{
    public :
    virtual ~IMovement();

    //@brief Définit l'état de la connexion au robot.
    //@note true si le robot est connecté, false sinon.
    //@note Ne doit pas être modifié via une instance de la classe, mais uniquement en interne par les classes qui implémentent cette interface.
    bool IsConnected = false;

    //@brief Définit l'autorisation de mouvement du robot.
    //@note true si le robot est autorisé à se déplacer, false sinon.
    //@note Ne doit pas être modifié via une instance de la classe, mais uniquement en interne par les classes qui implémentent cette interface.
    bool IsAllowedToMove = false;

    //@brief Définit l'état de mouvement du robot.
    //@note true si le robot est en mouvement, false sinon.
    //@note Ne doit pas être modifié via une instance de la classe, mais uniquement en interne par les classes qui implémentent cette interface.
    bool IsMoving = false;

    //@brief Définit l'état d'arrêt du robot.
    //@note true si le robot est arrêté, false sinon.
    //@note Ne doit pas être modifié via une instance de la classe, mais uniquement en interne par les classes qui implémentent cette interface.
    bool IsStopped = true;

    //@brief Définit l'état de demande de déconnexion du robot.
    //@note true si une déconnexion a été demandée, false sinon.
    //@note Ne doit pas être modifié via une instance de la classe, mais uniquement en interne par les classes qui implémentent cette interface.
    bool IsDisconnectRequested = false;

    //@brief Définit si le robot a atteint un marqueur.
    //@note true si le robot a atteint le marqueur, false sinon.
    //@note Peut être modifié par une instance de la classe qui implémente cette interface, par exemple pour indiquer que le robot a atteint une position cible.
    bool IsMarkerReached = false;


    //@brief Le modèle du robot.
    std::string Model = "Unknown Model";

    //@brief Le constructeur du robot.
    std::string Manufacturer = "Unknown Manufacturer";

    //@brief Lance la connexion au robot.
    //@return true si la connexion a réussi, false sinon.
    virtual bool Connect() = 0;

    //@brief Lance la deconnexion au robot.
    //@return true si la deconnexion a réussi, false sinon.
    virtual bool Disconnect() = 0;

    //@brief Déplace le robot dans l'espace en utilisant les coordonnées X, Y et une valeur angle.
    //@param xMillimeters Déplacement horizontal en millimètres, une valeur négative équivaut à un déplacement vers la gauche, une valeur positive vers la droite.
    //@param yMillimeters Déplacement vertical en millimètres, une valeur négative équivaut à un déplacement vers l'arrière, une valeur positive vers l'avant.
    //@param angle Rotation du robot en degrès, une valeur négative équivaut à une rotation vers la gauche, une valeur positive vers la droite.
    virtual void MoveXYZ(float xMillimeters, float yMillimeters, float angle) = 0;
    
    // @brief Déplace le robot dans l'espace en utilisant la coordonnée X.
    // @param millimeters Déplacement en millimètres sur l'axe X.
    // @note Une valeur négative équivaut à un déplacement vers la gauche, une valeur positive vers la droite.
    virtual void MoveX(float millimeters) = 0;

    // @brief Déplace le robot dans l'espace en utilisant la coordonnée Y.
    // @param millimeters Déplacement en millimètres sur l'axe Y.
    // @note Une valeur négative équivaut à un déplacement vers l'arrière, une valeur positive vers l'avant.
    virtual void MoveY(float millimeters) = 0;

    // @brief Déplace le robot dans l'espace en utilisant la coordonnée Z.
    // @param millimeters Déplacement en millimètres sur l'axe Z.
    // @note Une valeur négative équivaut à une rotation vers la gauche, une valeur positive vers la droite.
    virtual void MoveZ(float millimeters) = 0;

    virtual void MoveLeftBy(float millimeters) = 0;
    virtual void MoveRightBy(float millimeters) = 0;
    virtual void MoveForwardBy(float millimeters) = 0;
    virtual void MoveBackwardsBy(float millimeters) = 0;

    virtual void Start() = 0;
    virtual void Stop() = 0;

    virtual float GetX() = 0;
    virtual float GetY() = 0;
    virtual float GetZ() = 0;
};