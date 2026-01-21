#ifndef HC_SR04_h
#define HC_SR04_h


// ---------------------------------------------------------------------------
// HC-SR04 Ultrasonic Sensor Library
// ---------------------------------------------------------------------------

// Library to control HC-SR04 ultrasonic distance sensors using Arduino.
#if defined(ARDUINO) && ARDUINO >= 100
		#include <Arduino.h>
#else
        #include <WProgram.h>
        #include <pins_arduino.h>
#endif

//Define
#define HC_SR04_DEFAULT_MAX_DISTANCE_CM 50
#define HC_SR04_CONVERSION_US_CM  58
#define HC_SR04_DELAY_ECHO_TIMEOUT 58 * 2
#define HC_SR04_DELAY_ECHO 58000 
#define HC_SR04_DELAY_TRIGGER_MS 12
#define HC_SR04_FAIL_CYCLES_LIMIT 20

// Default Library Settings
#include "pinout.h"
// ---------------------------------------------------------------------------
// DEFINE
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// CLASS DEFINITION
// ---------------------------------------------------------------------------
class HCSR04 {
  public:
    // Constructor
    HCSR04(PinInfo PinTrigger, PinInfo PinEcho, unsigned int maxDistanceCm = HC_SR04_DEFAULT_MAX_DISTANCE_CM);
  public:
  // Enumerations
   enum class stateEcho: unsigned char {
        ECHO_OK = 0,
        ECHO_TIMEOUT_UP = 1,
        ECHO_TIMEOUT_DOWN = 2,
        ECHO_OK_OUT_LIMIT_CM = 3,
        ECHO_NOOK = 4,
        
        ENUM_END_STATEECHO
    }; char stateEcho;

  protected:
    //Variables
    volatile int maxDistanceCm;
    volatile int maxEchotimeUs;
    volatile unsigned long flagEchoStart;
    volatile unsigned long flagEchoEnd;
    volatile unsigned long flagEchoTime;
    volatile char cycleStateEcho;

    //MASK bits
    volatile uint8_t MASKBIT_PinTrigger;
    volatile uint8_t MASKBIT_PinEcho;
    //PORT
    volatile uint8_t *PORT_PinTrigger;
    volatile uint8_t *PORT_PinEcho;
    //MODE
    volatile uint8_t *MODE_PinTrigger;
    volatile uint8_t *MODE_PinEcho;
    
  public:
    // Fuctions
    unsigned long ping_cm();
    unsigned long  ping();
    boolean pingTrigger();
    // Fuctions complements
    int maxDistance(int newMaxDistanceCm); 
    int maxEchotime(int newMaxDistanceCm);
    void printStateEcho(unsigned long duration);
};
#endif // HC_SR04_h


