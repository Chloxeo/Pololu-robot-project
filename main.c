// N'hésitez pas à regarder le Readme.md pour plus de détails !

#include "macro.h"  // importer les fonction, structures et macros


// CONSTANTES POUR LE PI
#define KP 0.01  // gain proportionnel
#define KI 0.08  // gain intégral

#define SPEED_REFERENCE 20  // vitesse de référence pour les moteurs


// VARIABLES GLOBALES
volatile int old_A_R = 0;  // A right
volatile int old_B_R = 0;  // B right

volatile int count_retour = 0; // compteur pour le moteur gauche (lien entre tour de roue et position)
volatile int old_A_L = 0;  // A left
volatile int old_B_L = 0;  // B left

volatile Position pos = {0, 0}; // position du robot (X, Y)
volatile int orientation = 0; // 0 = X, 90 = Y, 180 = -X, 270 = -Y


// INTERRUPTIONS

ISR(INT6_vect) { 
    // interruption déclenchée par la rotation du moteur droit

    int current_xor = test_bit(PINE,PINE6);  // PE6
    int current_B = test_bit(PINF,PINF0);  // PF0
    int current_A = current_xor ^ current_B;

    if (current_B == old_A_R) {  // cas 1 : on avance

        // incrémenter la position selon l'orientation
        if (orientation == 0 || orientation == 360) pos.X++;
        else if (orientation == 90) pos.Y++;
        else if (orientation == 180) pos.X--;
        else if (orientation == 270) pos.Y--;

    }

    if (current_A == old_B_R) {  // cas 2 : on recule

        // décrémenter la position selon l'orientation
        if (orientation == 0) pos.X--;
        else if (orientation == 90) pos.Y--;
        else if (orientation == 180) pos.X++;
        else if (orientation == 270) pos.Y++;

    }
    
    // Mise à jour des anciennes valeurs de A et B
    old_B_R = current_B;
    old_A_R = current_A;
}

ISR(PCINT0_vect) {
    // interruption déclenchée par la rotation du moteur gauche

    int current_xor_L = test_bit(PINB,PINB4);
    int current_B_L = test_bit(PINE,PINE2);
    int current_A_L = current_xor_L ^ current_B_L;

    // mise à jour du compteur "count_retour" mesurant la distance parcourue
    if (current_A_L == old_B_L){count_retour--;}
    if (current_B_L == old_A_L){count_retour++;}

    old_B_L = current_B_L;
    old_A_L = current_A_L;

}

int main(void) {

    // Calibration des capteurs
    MinMax mm;
    calibrate_sensors(&mm);

    // Initialisation des ports pour les moteurs
    init_pins_PWM_motor();

    // Initialisation des interruptions pour les moteurs
    init_pins_interrupts();

    // Initialisation des variables pour le PI
    float error = 0; // erreur actuelle pour le PI
    float previous_error = 0; // erreur précédente pour le PI

    float command = 0; // commande pour les moteurs
    float previous_command = 0; // commande précédente pour les moteurs

    int int_command = 0; // arrondi de la commande pour les moteurs

    uint8_t previous_clock = TCNT1; // horloge pour le calcul du temps

    int capteurs[5]; // tableau pour stocker les valeurs normalisées des capteurs
    int count_time = 0; // initialisation compteur de temps pour la détection de fin de ligne

    while (1) {

        // Lecture des 5 capteurs
        read_all_sensors_normalized(&mm, capteurs);

        // DETECTION VIRAGE À DROITE
        if (capteurs[4] > 500) {

            avancer();

            // maj de l'orientation
            tourner_a_droite_90();
            orientation = (orientation + 90) % 360;
            
        }

        // DETECTION VIRAGE À GAUCHE
        if (capteurs[0] > 500) {

            avancer();

            // maj de l'orientation
            orientation = (orientation + 270) % 360;

        }
        
        // DETECTION FIN DE LIGNE
        if (capteurs[0] < 100 && capteurs[1] < 100 && capteurs[2] < 100 && capteurs[3] < 100 && capteurs[4] < 100) {
            count_time++;
            if (count_time == 10) {
                
                // retour à l'orientation initiale
                while (orientation != 0) { 
                    tourner_a_droite_90();
                    orientation = (orientation + 90) % 360;
                }

                // retour à la position de départ
                return_to_start(pos.X, pos.Y)

                count_time = 0; // réinitialiser le compteur de temps
                
            }
        } else {
            count_time = 0; // réinitialiser si on n'est plus en fin de ligne
        }

        // valeurs des capteurs 2 et 4 normalisées
        int capteur2 = capteurs[1];
        int capteur4 = capteurs[3];

        error = capteur2 - capteur4;  // Calcul de l'erreur

        uint8_t current_clock = TCNT1;

        command = KP * (error - previous_error) + KI * previous_error * (current_clock - previous_clock) * 62.5e-6 + previous_command;
        command_int = (int)command;

        // Calcul des vitesses moteur
        int speed_right = SPEED_REFERENCE + command_int;
        int speed_left = SPEED_REFERENCE - command_int;

        // Écrêtage des vitesses pour éviter les valeurs négatives ou trop élevées
        if (speed_right > 20) speed_right = 20;
        if (speed_left > 20) speed_left = 20;
        if (speed_right < 0) speed_right = 0;
        if (speed_left < 0) speed_left = 0;

        // Application des puissances aux moteurs
        OCR1A = speed_right; // moteur droit
        OCR1B = speed_left;  // moteur gauche

        // Mémorisation pour la prochaine boucle
        previous_clock = current_clock;
        previous_error = error;
        previous_command = command;

        // Affichage
        clear();
        printf("count1 = %d\ncount2 = %d\ncount3 = %d\ncount4 = %d\ncount5 = %d\n",capteurs[0],capteurs[1],capteurs[2],capteurs[3],capteurs[4]);
        printf("X, Y = (%d, %d)\nOrientation = %d",pos.X, pos.Y,orientation);


        _delay_ms(10); // petite pause de 10 ms pour éviter une boucle trop rapide
    }
}
