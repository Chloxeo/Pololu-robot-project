#ifndef MACRO_H
#define MACRO_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // pour les interruptions
#include <stdbool.h>

#include "print.h"  // affiche sur l'ecran ou le terminal


// I) MACROS

// positionner un bit à la valeur 1 
#define set_bit(register_name, bit_position)((register_name) |= (1<<(bit_position)))

// positionner un bit à la valeur 0
#define clear_bit(register_name,bit_position)((register_name) &= ~(1<<(bit_position)))

// inverser la valeur d'un bit
#define toggle_bit(register_name,bit_position)((register_name) ^= (1<<(bit_position)))

// tester si un bit est positionné à la valeur 1 
#define test_bit(register_name,bit_position)((register_name) & (1<<(bit_position)))


// II) Structures

/**
 * @brief Structure stockant les valeurs minimales et maximales 
 *        des 5 capteurs de réflexion mesurées lors de la calibration
 */
typedef struct {
    int min_C2, max_C2;
    int min_C4, max_C4;
    int min_C1, max_C1;
    int min_C3, max_C3;
    int min_C5, max_C5;
} MinMax;

/**
 * @brief Structure contenant les coordonnées X et Y de la position du robot
 */
typedef struct {
    int X, Y;
} Position;

/**
 * @brief Structure contenant les informations de retour au point de départ
 *        CAP : angle de retour en degrés
 *        DISTANCE : distance à parcourir pour revenir au point de départ
 */
typedef struct {
    int CAP, DISTANCE;
} CheminRetour;


// II) FONCTIONS

/**
 * @brief Initialise les pins pour controler les moteurs en PWM
 */
void init_pins_PWM_motor();

/**
 * @brief Initialise les pins pour les interruptions externes
 */
void init_pins_interrupts();

/**
 * @brief Lit les mesures d'un capteur de réflexion
 * @param DDRx Pointeur vers le registre de direction du port
 * @param DDxn Numéro du bit dans le registre de direction
 * @param PORTx Pointeur vers le registre de sortie du port
 * @param PORTxn Numéro du bit dans le registre de sortie
 * @param PINx Pointeur vers le registre de lecture du port
 * @param PINxn Numéro du bit dans le registre de lecture
 * @return Un entier représentant le temps de charge du condensateur
 */
int read_reflectance_sensor(volatile uint8_t* DDRx, uint8_t DDxn,
                            volatile uint8_t* PORTx, uint8_t PORTxn,
                            volatile uint8_t* PINx, uint8_t PINxn);

/**
 * @brief Calibre les 5 capteurs de réflexion en mesurant les 
 *        valeurs minimales et maximales sur une période de 4 secondes
 * @param mm Pointeur vers la structure MinMax pour stocker les valeurs min/max
 */
void calibrate_sensors(MinMax* mm);

/**
 * @brief Normalise une valeur de capteur entre 0 et 1000
 * @param val La valeur mesurée du capteur
 * @param min La valeur minimale mesurée lors de la calibration
 * @param max La valeur maximale mesurée lors de la calibration
 * @return La valeur normalisée entre 0 et 1000
 */
int normalize(int val, int min, int max);

/**
 * @brief Lit les valeurs normalisées de tous les capteurs de réflexion
 * @param mm Pointeur vers la structure MinMax contenant les valeurs min/max
 * @param values Tableau pour stocker les valeurs normalisées des capteurs
 */
void read_all_sensors_normalized(MinMax* mm, int* values);

/**
 * @brief Tourne le robot à droite de 90 degrés 
 */
void tourner_a_droite_90();

/**
 * @brief Tourne le robot à gauche de 90 degrés
 */
void tourner_a_gauche_90();

/**
 * @brief Effectue un demi-tour du robot
 */
void Uturn();

/**
 * @brief permet au robot d'avancer en ligne droite pendant 200 ms
 *        (utile avant de tourner ou de faire un demi-tour)
 */
void avancer();

/**
 * @brief prend en paramètre un délai variable exprimé en millisecondes,
 *        et construira le délai par pas de 1000 µs.
 */
void delay_en_ms(int temps_ms);

/**
 * @brief permet au robot de tourner pendant un temps donné
 * @param temps Le temps en millisecondes pendant lequel le robot doit tourner
 */
void tourner_temps(int temps);

/**
 * @brief Avance le robot pendant un temps donné
 * @param temps Le temps en millisecondes pendant lequel le robot avance
 */
void avancer_temps(int temps);

/**
 * @brief Donne les instructions au robot pour qu'il retourne à son point de départ
 * @param XTOT La distance X totale parcourue depuis le point de départ
 * @param YTOT La distance Y totale parcourue depuis le point de départ
 */
void return_to_start(int XTOT, int YTOT);


#endif