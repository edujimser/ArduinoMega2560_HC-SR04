#include "hc-sr04/hc-sr04.h"


HCSR04::HCSR04(PinInfo PinTrigger, PinInfo PinEcho, unsigned int max_cm_distance) {
    HCSR04::maxDistanceCm = maxDistance(max_cm_distance);
    HCSR04::maxEchotimeUs = maxEchotime(maxDistanceCm);

    //MASK bits for direct port manipulation
    MASKBIT_PinTrigger = digitalPinToBitMask(PinTrigger.number);
    MASKBIT_PinEcho = digitalPinToBitMask(PinEcho.number);

    //PORT -  PORTx registers for direct port manipulation, HIGH/LOW
    PORT_PinTrigger = portOutputRegister(digitalPinToPort(PinTrigger.number));
    PORT_PinEcho = portInputRegister(digitalPinToPort(PinEcho.number));

    //MODE - DDRx registers for direct port manipulation, IN/OUT
    MODE_PinTrigger = (uint8_t *) portModeRegister(*PORT_PinTrigger);
    MODE_PinEcho = (uint8_t *) portModeRegister(digitalPinToPort(PinEcho.number));

    // Configure pins
    *MODE_PinTrigger |= MASKBIT_PinTrigger;    // Set trigger pin to output.
    *PORT_PinTrigger &= ~MASKBIT_PinTrigger;   // Trigger pin should already be low

    *MODE_PinEcho &= ~MASKBIT_PinEcho;      // Set echo pin to input.
    *PORT_PinEcho &= ~MASKBIT_PinEcho;     // Just in case, set echo pin low.
}

int HCSR04::maxDistance(int newMaxDistanceCm) {
    if (newMaxDistanceCm > HC_SR04_DEFAULT_MAX_DISTANCE_CM) {
        return  HC_SR04_DEFAULT_MAX_DISTANCE_CM;
    }
    return newMaxDistanceCm;
};

int HCSR04::maxEchotime(int newMaxDistanceCm) {
    return (newMaxDistanceCm * HC_SR04_CONVERSION_US_CM) * 2;
}


unsigned long HCSR04::ping_cm() {
    unsigned long duration_us = ping();
    return duration_us / HC_SR04_CONVERSION_US_CM;
}

unsigned long HCSR04::ping() {
    pingTrigger();
    return (flagEchoTime);
}

boolean HCSR04::pingTrigger(){
    // Lanzamos el pulso del trigger
    *PORT_PinTrigger |= MASKBIT_PinTrigger;  // Disparar el trigger
    delayMicroseconds(HC_SR04_DELAY_TRIGGER_MS); // Mantener el trigger durante 12us                   
    *PORT_PinTrigger &= ~MASKBIT_PinTrigger;  // Apagar el trigger



    // Esperamos la respuesta del ECHO por el flanco de subida
    unsigned long start = micros() - HC_SR04_DELAY_TRIGGER_MS;

    while (!(*PORT_PinEcho & MASKBIT_PinEcho)){ // Esperando flanco de subida
       if ((micros() - start) >= maxEchotimeUs){ // Timeout
            cycleStateEcho++; // Incrementar contador de ciclos fallidos
            if (cycleStateEcho > HC_SR04_FAIL_CYCLES_LIMIT) stateEcho = static_cast<char>(stateEcho::ECHO_NOOK);// Si hay más de 3 ciclos fallidos, marcar estado NOOK
            else stateEcho = static_cast<char>(stateEcho::ECHO_TIMEOUT_UP);// Marcar estado TIMEOUT
            return false;// Salir de la función
       } 
    }

    // Marcar el tiempo de inicio del eco
    flagEchoStart = micros();


    // Esperamos el flanco de bajada del ECHO
    while (*PORT_PinEcho & MASKBIT_PinEcho){ // Esperando flanco de bajada
       if ((micros() - flagEchoStart) >= maxEchotimeUs){// Timeout
            cycleStateEcho++;// Incrementar contador de ciclos fallidos
            if (cycleStateEcho > HC_SR04_FAIL_CYCLES_LIMIT) stateEcho = static_cast<char>(stateEcho::ECHO_NOOK);// Si hay más de 3 ciclos fallidos, marcar estado NOOK
            else stateEcho = static_cast<char>(stateEcho::ECHO_TIMEOUT_DOWN);// Marcar estado TIMEOUT
            return false;
       } 
    }

    // Marcar el tiempo de fin del eco
    flagEchoEnd = micros();


  
    

    cycleStateEcho = 0; // Resetear contador de ciclos correctos
    flagEchoTime = flagEchoEnd - flagEchoStart;// Calcular tiempo de eco
    stateEcho = static_cast<char>(stateEcho::ECHO_OK);// Marcar estado OK
    return true;
}

void HCSR04::printStateEcho(unsigned long distance){
    switch (stateEcho){
        case static_cast<char>(stateEcho::ECHO_OK):
            Serial.print("ECHO_OK: ");
            Serial.print(distance);
            Serial.println(" CM");
            break;
        case static_cast<char>(stateEcho::ECHO_TIMEOUT_UP):
            Serial.println("ECHO_TIMEOUT_UP");
            break;
        case static_cast<char>(stateEcho::ECHO_TIMEOUT_DOWN):
            Serial.println("ECHO_TIMEOUT_DOWN");
            break;
        case static_cast<char>(stateEcho::ECHO_NOOK):
            Serial.println("ECHO_NOOK");
            break;
        default:
            Serial.println("Unknown stateEcho");
            break;
    };
}