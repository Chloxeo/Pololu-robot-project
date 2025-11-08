
#include "macro.h"


// initialiser les pins pour controler les moteurs en PWM
void init_pins_PWM_motor() {
    
    TCCR1A = 0b10100001; // mode Fast PWM, OC1A et OC1B en mode non inversé
    TCCR1B = 0b00001011; // prescaler clk/64, mode Fast PWM

    // controle de la puissance
    set_bit(DDRB, DDB5); // OC1A en sortie
    set_bit(DDRB, DDB6); // OC1B en sortie

    // direction des moteurs
    set_bit(DDRB, DDB1); // PB1 en sortie
    set_bit(DDRB, DDB2); // PB2 en sortie

    clear_bit(PORTB, PB1); // moteur 1 dans le sens horaire
    clear_bit(PORTB, PB2); // moteur 2 dans le sens horaire

    // initialisation de la puissance
    OCR1A = 0; // puissance moteur droit
    OCR1B = 0; // puissance moteur gauche

}


// initialiser les registres de configuration des interruptions externes
void init_pins_interrupts() {

    // INT6 se déclenche sur front montant et front descendant
    clear_bit(EICRB,ISC61);
    set_bit(EICRB,ISC60);
    set_bit(EIMSK,INT6);

    set_bit(PCICR,PCIE0);
    set_bit(PCMSK0,PCINT4);

    sei();

}


// lire les mesures d'un capteur de réflexion
int read_reflectance_sensor(volatile uint8_t* DDRx, uint8_t DDxn,
                            volatile uint8_t* PORTx, uint8_t PORTxn,
                            volatile uint8_t* PINx, uint8_t PINxn) {

    int count = 0;

    // décharger condensateurs
    set_bit(*DDRx,DDxn);       // config pin xn en sortie
    set_bit(*PORTx,PORTxn);    // Port xn à l'état haut
    _delay_ms(10);             // attendre la décharge

    // étape 2 : broches en entrée sans pull up
    clear_bit(*PORTx,PORTxn);  // port xn à l'état bas
    clear_bit(*DDRx,DDxn);     // pin xn en entrée

    // activer EMIT
    set_bit(DDRB,DDB7);        // pin B7 en sortie
    set_bit(PORTB,PORTB7);     // Port B7 à l'état haut

    while ( test_bit(*PINx,PINxn) ){ // True si DOWN == 1
        _delay_us(1);
        count++;
    }

    clear_bit(PORTB,PORTB7);    // Port B7 à l'état bas

    return count;

}


// Fonction de calibration des capteurs (durée 6 secondes)
void calibrate_sensors(MinMax* mm) {

    clear();
    printf("Calibration en cours...\n");

    // Initialisation des min/max
    mm->min_C2 = mm->min_C4 = mm->min_C1 = mm->min_C3 = mm->min_C5 = 10000;
    mm->max_C2 = mm->max_C4 = mm->max_C1 = mm->max_C3 = mm->max_C5 = 0;

    for (int i = 0; i < 200; i++) {
        int val2 = read_reflectance_sensor(&DDRF, DDF7, &PORTF, PORTF7, &PINF, PINF7);
        int val4 = read_reflectance_sensor(&DDRF, DDF4, &PORTF, PORTF4, &PINF, PINF4);
        int val1 = read_reflectance_sensor(&DDRD, DDD6, &PORTD, PORTD6, &PIND, PIND6); 
        int val3 = read_reflectance_sensor(&DDRF, DDF5, &PORTF, PORTF5, &PINF, PINF5); 
        int val5 = read_reflectance_sensor(&DDRF, DDF1, &PORTF, PORTF1, &PINF, PINF1); 

        if (val2 < mm->min_C2) mm->min_C2 = val2;
        if (val2 > mm->max_C2) mm->max_C2 = val2;

        if (val4 < mm->min_C4) mm->min_C4 = val4;
        if (val4 > mm->max_C4) mm->max_C4 = val4;

        if (val1 < mm->min_C1) mm->min_C1 = val1;
        if (val1 > mm->max_C1) mm->max_C1 = val1;

        if (val3 < mm->min_C3) mm->min_C3 = val3;
        if (val3 > mm->max_C3) mm->max_C3 = val3;

        if (val5 < mm->min_C5) mm->min_C5 = val5;
        if (val5 > mm->max_C5) mm->max_C5 = val5;

        _delay_ms(20);
    }

    clear();
    printf("Calibration terminee !\n");
    _delay_ms(2000);
}

// Fonction pour normaliser une valeur de capteur
int normalize(int val, int min, int max) {
    
    if (val < min) return 0; // Si la valeur est en dessous du minimum, renvoyer 0
    if (val > max) return 1000; // Si la valeur est au-dessus du maximum, renvoyer 1000
    else return ((val - min) * (long)(1000)) / (max - min) ;

}


void read_all_sensors_normalized(MinMax* mm, int* values) {

    values[0] = normalize(read_reflectance_sensor(&DDRD, DDD6, &PORTD, PORTD6, &PIND, PIND6), mm->min_C1, mm->max_C1);
    values[1] = normalize(read_reflectance_sensor(&DDRF, DDF7, &PORTF, PORTF7, &PINF, PINF7), mm->min_C2, mm->max_C2);
    values[2] = normalize(read_reflectance_sensor(&DDRF, DDF5, &PORTF, PORTF5, &PINF, PINF5), mm->min_C3, mm->max_C3);
    values[3] = normalize(read_reflectance_sensor(&DDRF, DDF4, &PORTF, PORTF4, &PINF, PINF4), mm->min_C4, mm->max_C4);
    values[4] = normalize(read_reflectance_sensor(&DDRF, DDF1, &PORTF, PORTF1, &PINF, PINF1), mm->min_C5, mm->max_C5);

}


void tourner_a_droite_90() {
    
    OCR1A = 0;   // moteur droit à l'arrêt
    OCR1B = 0;   // moteur gauche à l'arrêt

    set_bit(PORTB, PB1);    // sens antihoraire
    clear_bit(PORTB, PB2);  // sens horaire

    OCR1A = 20;  // moteur droit recule
    OCR1B = 20;  // moteur gauche avance

    _delay_ms(780); 

    OCR1A = 0;  // moteur droit à l'arrêt
    OCR1B = 0;  // moteur gauche à l'arrêt

    clear_bit(PORTB, PB1);  // sens horaire

}


void tourner_a_gauche_90() {

    OCR1A = 0;  // moteur droit à l'arrêt
    OCR1B = 0;  // moteur gauche à l'arrêt

    clear_bit(PORTB, PB1);  // sens horaire
    set_bit(PORTB, PB2);    // sens antihoraire

    OCR1A = 20;  // moteur droit avance
    OCR1B = 20;  // moteur gauche recule

    _delay_ms(780); 

    OCR1A = 0;  // moteur droit à l'arrêt
    OCR1B = 0;  // moteur gauche à l'arrêt

    clear_bit(PORTB, PB2);  // sens horaire

}


void Uturn() {

    OCR1A = 0;  // arret moteur droit
    OCR1B = 0;  // arret moteur gauche

    clear_bit(PORTB, PB1);  // sens horaire
    set_bit(PORTB, PB2);    // sens antihoraire

    // puissance moteur droit et gauche
    OCR1A = 20;   
    OCR1B = 20;     

    _delay_ms(1600);  // tourner pendant 1.6 secondes

    OCR1A = 0;  // arret moteur droit  
    OCR1B = 0;  // arret moteur gauche 

    clear_bit(PORTB, PB2);  // sens horaire

}


void avancer() {
    OCR1A = 20;
    OCR1B = 20;

    clear_bit(PORTB, PB1);
    clear_bit(PORTB, PB2);

    _delay_ms(200); 
}

// Fonction pour créer un délai (variable) en millisecondes
void delay_en_ms(int temps_ms) {
    for (int i = 0; i < temps_ms; i++) {
        _delay_us(1000); // 1 ms = 1000 µs
    }
}

void tourner_temps(int temps) {

    OCR1A = 0;     // moteur droit à l'arrêt
    OCR1B = 0;     // moteur gauche à l'arrêt

    set_bit(PORTB, PB1);    // sens antihoraire
    clear_bit(PORTB, PB2);  // sens horaire

    OCR1A = 20;  // moteur droit recule
    OCR1B = 20;  // moteur gauche avance

    delay_en_ms(temps); 

    OCR1A = 0;    // moteur droit à l'arrêt
    OCR1B = 0;    // moteur gauche à l'arrêt

    clear_bit(PORTB, PB1);  // sens horaire

}


void avancer_temps(int temps) {
    
    clear_bit(PORTB, PB1);  // sens horaire
    clear_bit(PORTB, PB2);  // sens horaire

    OCR1A = 20;
    OCR1B = 20;

    delay_en_ms(temps);

    OCR1A = 0;  // moteur droit à l'arrêt
    OCR1B = 0;  // moteur gauche à l'arrêt
}


void return_to_start(int XTOT, int YTOT) {

    float distance = sqrt(XTOT * XTOT + YTOT * YTOT);  // hypothénuse
    float cap = atan2(YTOT, XTOT);                     // angle CAP en degrés

    int temps_retour = distance * 10;   
    int temps_rotation = (cap / 90) * 780; // temps pour tourner à droite ou à gauche

    tourner_temps(temps_rotation); // tourner pour s'orienter vers le point de départ
    _delay_ms(100);  
    avancer_temps(temps_retour); // avancer vers le point de départ

}