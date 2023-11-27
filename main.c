/*
    Alternative firmware for STC15W104 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include <delay.h>
#include <uart_software.h>

// just do not have much more space for HAL with 1KB flash code size
//#include <eeprom-hal.h>
//#include <gpio-hal.h>
//#include <power-hal.h>
//#include <timer-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER

// uncomment only one protocol timing
//#define PROTOCOL_STOCK
#define PROTOCOL_ONE
//#define PROTOCOL_TWO

//---------------------------------------------------------------------------------------------
// we need to obey rules for alarm transmission: https://www.law.cornell.edu/cfr/text/47/15.231
//  human actuated or alarm conditions are allowed to transmit (emergency if door opened)
//  battery condition would not be an emergency and so we piggy when sending human actuated codes
// very little time is allowed for any further periodic transmission, as an exercise given our timings
//   for 2 secs / hour allowed periodic transmission
//   number of events is about 2000000/(20*(470+14060+24*(470+1360))) ~ 2 events
//   (3600 seconds / 2 events) / (16 sec wakeup timer) ~ 113 wake ups
//---------------------------------------------------------------------------------------------


// check for tamper switch pressed at startup, if so enter bootloader mode
// this is helpful during firmware development or reflashing
// however, if inserting the battery, make sure not to hold down the tamper switch
// FIXME: concerned that brownout reset could falsely trigger this while in housing
#if 1
    #define CHECK_FOR_TAMPER_AT_STARTUP true
#else
    #define CHECK_FOR_TAMPER_AT_STARTUP false
#endif


// hardware pin definitions
//   circuit is voltage divider and high side transistor with processor controlling divider
//   pin 3.0 is also connected with receive pin on header
// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
//   pin 3.1 is also connected with transmit pin on header
// ASK modulation to RF chip is pin 3.4
// battery sensed on my board when variable power supply (stand in for battery)
//   reaches just above one volt from starting point of 1.5 volts
//   pin 3.5 signal line goes from high (3.3V) to low (0V)
#define RADIO_VDD      P3_0
#define LED_PIN        P3_1
#define REED_SWITCH    P3_2
#define TAMPER_SWITCH  P3_3
#define RADIO_ASK      P3_4
#define BATTERY_DETECT P3_5


// radio protocol apparently requires repeated transmissions so it is accepted at receiver
// RF-Bridge-EFM8BB1 alternative firmware for receiver chip on sonoff bridge seems to require only repeating twice
// stock EFM8BB1 receiver on sonoff bridge r1 seems to require four retransmissions
// @mmotley999 reported up to eight retransmissions are required
// @bismosa reports stock firmware repeats transmission twenty times!
// sonoff bridge r2 seems to to similarly require many repeats
//#define REPEAT_TRANSMISSIONS 1
//#define REPEAT_TRANSMISSIONS 4
//#define REPEAT_TRANSMISSIONS 8
#define REPEAT_TRANSMISSIONS 20

// longest protocol in bits in rcswitch project is Alecto WS-1200 weather station so 9*8 = 72 bits
#define MAX_PACKET_BYTES 9

// sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
// oscillator startup time crystal HC49S (300 microseconds)
// standby ask delay time (min, 30), typical(75), max(120) milliseconds
// however fig. 6 shows only about a two millisecond ramp up time for VDD
// sec. 14.2 of SYN115-ASK-Transmitter-Datasheet.pdf describes two more modes for on-off control that save battery power
// may want to look into that more in the future
// milliseconds
#define RADIO_STARTUP_TIME 10

// some receivers require a delay so that multiple packets are received (e.g., sonoff bridge r2v2.2)
// milliseconds
#define RADIO_GUARD_TIME 30

// tens of microseconds
#define DEBOUNCE_TIME_10US 20

// maximum is 32768 as per sec. 7.8 power down wake-up special timer
// highest bit of register will be set to enable wake up timer
// so maximum count is 0x7FFF and then highest bit set

// pg. 551, sec. 7.8 power down wake up special timer
// this equates to about 16 seconds according to examples
// note: weird, why does setting to 0xFFFF wake up too quickly, about every two seconds?
#define SLEEP_TIME_0 0xfffe

// milliseconds
#define CONTROLLER_STARTUP_TIME 200

// array size
#define EVENT_HISTORY_SIZE 16


// unique ID is stored in ram in the same locations on mcu 101/104 parts
// see makefile - we are limiting ram size so that it is not initialized/written at these addresses
// it is possible to pull id from flash space, but it is at different locations on different sized parts
// note: the values stored in ram at these locations should match the last four characters shown by Target UID when flashing
#define ID0_ADDR_RAM 0x76
#define ID1_ADDR_RAM 0x77

//  1K MCU (e.g. STC15W101)
// #define ID_ADDR_ROM 0x03f9
//  4K MCU (eg. STC15W104)
//#define ID_ADDR_ROM 0x0ff9

// point at ram locations to derive uniqe id for inclusion in radio messages as original firmware did
// see sec. 1.12 global unique identification number STC15-English.pdf
static const __idata unsigned char *guid0 = (__idata unsigned char*) ID0_ADDR_RAM;
static const __idata unsigned char *guid1 = (__idata unsigned char*) ID1_ADDR_RAM;


// codes used in original firmware are (0x06, 0x07, 0x0a, 0x0e), others were added
// original codes reported by: https://community.home-assistant.io/t/budget-priced-wife-friendly-wireless-wall-switch-using-433mhz-and-mqtt/88281
// if sensor is being powered by 3.3V on header pins, this will also show battery low apparently
static const unsigned char gBatteryLow  = 0x06;
static const unsigned char gTamperOpen  = 0x07;
static const unsigned char gTamperClose = 0x08;
static const unsigned char gBatteryOk   = 0x09;
static const unsigned char gReedOpen    = 0x0a;
static const unsigned char gReedClose   = 0x0e;



// stock protocol was read from unaltered sensor by user @bismosa using logic analyzer
// "protocol 1" is the first rc-switch protocol which is also supported by RF-Bridge-EFM8BB1
// (https://github.com/Portisch/RF-Bridge-EFM8BB1/blob/master/inc/RF_Protocols.h)
// rc-switch project timings (https://github.com/sui77/rc-switch)
// changed pulse lengths from microseconds in rc-switch library to 10 microseconds units
//   because hardware abstraction layer provides delay10us() for STC15 MCU
// avoid performing mathematical multiplies on processor by computing these as constants
#if defined(PROTOCOL_STOCK)
    // stock sensor timings (see door_sensor_reverse_notes_fv1.txt)
    // (tested working on Sonoff Bridge R2 V1.0)
    // (does not work on stock Sonoff Bridge R2 V2.2)
    const uint16_t gPulseHigh =   47;
    const uint16_t gPulseLow  = 1406;
    const uint16_t gZeroHigh  =   47;
    const uint16_t gZeroLow   =  136;
    const uint16_t gOneHigh   =  136;
    const uint16_t gOneLow    =   47;
        
    // it saves code space to just specify polarity for conditional compilation
    #define PROTOCOL_INVERTED false

#elif defined(PROTOCOL_ONE)
    const uint16_t gPulseHigh =   35;
    const uint16_t gPulseLow  = 1085;
    const uint16_t gZeroHigh  =   35;
    const uint16_t gZeroLow   =  105;
    const uint16_t gOneHigh   =  105;
    const uint16_t gOneLow    =   35;
        
    #define PROTOCOL_INVERTED false
    
#elif defined(PROTOCOL_TWO)
    const uint16_t gPulseHigh =   65;
    const uint16_t gPulseLow  =  650;
    const uint16_t gZeroHigh  =   65;
    const uint16_t gZeroLow   =  130;
    const uint16_t gOneHigh   =  130;
    const uint16_t gOneLow    =   65;
        
    #define PROTOCOL_INVERTED false

#endif // specify protocol


// FIXME: add better comment
// these are usually, though not always, divisible by eight
const uint8_t  gProtocolBits = 24;
//const uint8_t  gProtocolBits = 40;
//const uint8_t  gProtocolBits = 72;


// storage for bytes send over radio
unsigned char packet[MAX_PACKET_BYTES];


// set flags in interrupts, then save and send event history in main loop
struct Flags {
    volatile bool reedInterrupted;
    volatile bool tamperInterrupted;
    volatile bool batteryLowInterrupted;
    volatile bool tamperTripped;
    volatile bool reedTripped;
    volatile bool batteryLowTripped;
    // save switches or battery states for sending
    volatile unsigned char eventHistory[EVENT_HISTORY_SIZE];
    volatile unsigned char eventCount;
    // track packets sent since startup
    volatile unsigned char packetCount;
    // track times woken up so we can track longer time periods
    unsigned char tamperWakeupCount;
    unsigned char reedWakeupCount;
    unsigned char batteryWakeupCount;
};

// arrays are not initialized, but okay because we only read when count is incremented
struct Flags flag = {
    .reedInterrupted       = false, 
    .tamperInterrupted     = false,
    .tamperTripped         = false,
    .reedTripped           = false,
    .batteryLowTripped     = false,
    .eventCount         = 0,
    .packetCount        = 0,
    .tamperWakeupCount  = 0,
    .reedWakeupCount    = 0,
    .batteryWakeupCount = 0
};



// only enable power to radio when we are going to modulate ASK pin (i.e., send data)
// low pin setting enables transistor which supplies power to radio chip
inline void enable_radio_vdd(void)
{
    RADIO_VDD = 0;
}

// pin setting functions are more readable than direct pin setting
// and avoid making errors (e.g., "enabling" something is actually setting pin logic level to zero)
inline void disable_radio_vdd(void)
{
    RADIO_VDD = 1;
}

// specify inline to save some flash code space
inline void radio_ask_first_logic_level(void)
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 1;
    #else
        RADIO_ASK = 0;
    #endif
}

inline void radio_ask_second_logic_level(void)
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 0;
    #else
        RADIO_ASK = 1;
    #endif
}

// led is controlled by transistor which essentially inverts pin output
// (so low level enables transistor and then LED is on)
inline void led_on(void)
{
    LED_PIN = 0;
}

inline void led_off(void)
{
    LED_PIN = 1;
}

// enable external interrupt 0 (reed switch connected)
// enable external interrupt 1 (tamper switch connected)
// also of course need to enable global interrupts (EA = 1)
inline void enable_interrupts(void)
{
    // called the interrupt enable register in datasheet (IE)
    // note: default for ext. interrupts 0 and 1 is to interrupt on both falling and rising edges
    // (IT0 = 0, IT1 = 0)
    // EA = 1, EX1 = 1, EX0 = 1
    IE1 = 0x85;
    // EX3 = 1 for battery status
    //INT_CLKO = 0x20;
}


// allows integer delays with 10 microsecond function at the expense of accuracy
void delay10us_wrapper(unsigned int microseconds)
{
    const unsigned char step = 0xff;
    
    while (microseconds > step)
    {
        delay10us(step);
        microseconds -= step;
    }
    
    delay10us(microseconds);
}

/*! \brief 
 *
 */        
inline bool isTamperOpen(void)
{
    return TAMPER_SWITCH;
}

inline bool isReedOpen(void)
{    
    return REED_SWITCH;
}

// read resistive divider state
inline bool isBatteryLow(void)
{    
    return !BATTERY_DETECT;
}


/*! \brief Description
 *         Tips [http://ww1.microchip.com/downloads/en/AppNotes/Atmel-9164-Manchester-Coding-Basics_Application-Note.pdf]
 *
 */  
void send_radio_packet(unsigned char* packetPtr, const unsigned char bitsInPacket)
{
    //unsigned char index;
    unsigned char bitIndex;
    unsigned char currentBit;
    
    
    //enable_radio_vdd();
    //delay1ms(RADIO_STARTUP_TIME);
    
    // many receivers require repeatedly sending identical transmissions to accept data
    //for (index = 0; index < REPEAT_TRANSMISSIONS; index++)
    //{
        // rf sync pulse
        radio_ask_second_logic_level();
        delay10us(gPulseHigh);
        
        // this should be the only really long delay required
        radio_ask_first_logic_level();
        delay10us_wrapper(gPulseLow);

        // this is similar to the atmel manchester encoding approach
        currentBit = 0;
        for (bitIndex = 0; bitIndex < bitsInPacket; bitIndex++)
        {
            if (currentBit == 8)
            {
                // FIXME:
                packetPtr++;
                
                // FIXME: why is this done, looking for wrap around?
                //if(packetPtr == &packet[0]){bitIndex = bitsInPacket + 1;}
                
                currentBit = 0;
            }
            
            // mask out all but left most bit value, and if byte is not equal to zero (i.e. left most bit must be one) then send one level
            if((*packetPtr & 0x80) == 0x80)
            {
                radio_ask_second_logic_level();
                delay10us_wrapper(gOneHigh);
                
                radio_ask_first_logic_level();
                delay10us_wrapper(gOneLow);
            }
            else
            {
                radio_ask_second_logic_level();
                delay10us_wrapper(gZeroHigh);
                
                radio_ask_first_logic_level();
                delay10us_wrapper(gZeroLow);
            }
            
            //
            *packetPtr = *packetPtr << 1;
            
            //
            currentBit++;
        }
    //}
    
}


//-----------------------------------------
//FIXME: handle reentrancy?
// interrupt and wake up on reed pin change (default is rising and falling edge)
void external_isr0(void) __interrupt (0)
{
    flag.reedInterrupted = true;
}

//-----------------------------------------
// interrupt and wake up on tamper switch pin change
void external_isr1(void) __interrupt (2)
{
    flag.tamperInterrupted = true;
}


// sec. 4.1 All port pins default to quasi-bidirectional after reset. Each one has a Schmitt-triggered input for improved input noise rejection.
//batteryMonitor GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN5, GPIO_HIGH_IMPEDANCE_MODE)
//radioASK       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE)
//tamperSwitch   GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_BIDIRECTIONAL_MODE)
//reedSwitch     GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_BIDIRECTIONAL_MODE)
//ledPin         GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE)
//radioVDD       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE)
inline void configure_pin_modes(void)
{
    // avoid bit operations to save on flash code space
    // just explicitly set gpio mode
    P3M1 = 0x23;
    P3M0 = 0x03;
}

// open drain
//   turn led off
//   disable radio power
// sec. 4.9.1 quasi-bidirectional i/o
//  enables weak pullups for reed and tamper switches assuming we are in bidirectional mode (pins high)
// note: do not use radio_ask_X() to disable level/power on ask pin, because it might be inverted, so would actually enable power!
inline void startup_pins_state(void)
{
    P3 = 0x0f;
}

void main(void)
{
    // as per HAL instructions
    INIT_EXTENDED_SFR();
    
    // makes the code easier to read below
    bool state;
    unsigned char index;
    unsigned char repeatIndex;
    
    // FIXME: add comment
    unsigned char bitsInPacket = gProtocolBits;

    // setup gpio
    configure_pin_modes();
    startup_pins_state();
    
    // software uart
    // FIXME: this will have problems because pin is shared with radio power enable?
    // FIXME: would need to set pin as default high in any case for software uart
    //init_software_uart();
    //enable_timer0();
    
    // pulse LED at startup because we delay next anyway
    led_on();
    
    // give the microcontroller time to stabilize
    delay1ms(CONTROLLER_STARTUP_TIME);
    
    led_off();


#if CHECK_FOR_TAMPER_AT_STARTUP

    // if we are holding down tamper switch at power up, enter iap mode
    // this is intended to make entering flash mode easier on subsequent flashes
    // if the board has no tamper switch, then this will just never do anything
    // if a regular user is inserting a battery, it is unlikely that they are also pressing tamper switch
    //  so this should just be skipped over in that case
    if (!isTamperOpen())
    {
        // set ISP boot bit and reset processor
        IAP_CONTR = 0x60;
    }
    
#endif
    
    
    // enable everything in one call to save bytes
    enable_interrupts();
    

    // main loop
    while (true)
    {       

        // check if there are unserviced interrupt(s) prior to sleeping
        if (!(flag.reedInterrupted | flag.tamperInterrupted))
        {
            {
                // go to sleep
                // program will next either resume due to wake up timer (if enabled) or due to interrupt(s)
                PCON |= M_PD;
                
                // sec. 2.3.3.1 demo program in datasheet and example HAL show providing nops() after power down
                NOP();
                NOP();
                
                // DEBUG: test software uart
                //putc(0x4A);
            }
        }
        
        
        // if we do too much in the interrupts themselves it produces too many stack pushes/pops
        // among other problems that are caused, so perform most logic here
        if (flag.reedInterrupted)
        {
            // slope on reed switch from high to low is about 200 microseconds as measured on oscilloscope
            // FIXME: this may be a terrible debounce
            delay10us(DEBOUNCE_TIME_10US);

            // clear flag after debounce so we do not pick up any bounces
            flag.reedInterrupted = false;         
            
            // more readable for array indexing
            index = flag.eventCount;
            
            // just read switch once here, but use in two logic locations below
            state = isReedOpen();
            
            // save any switch (or later battery) changes in order of occurence
            // note: we really do not have code space for checking array overflows
            if (state)
            {
                flag.eventHistory[index] = gReedOpen;
            } else {
                flag.eventHistory[index] = gReedClose;
            }
            
            // track count because we might have multiple events in quick succession
            flag.eventCount++;
            
            
            // if reed is open, consider it tripped until closed
            // note: if trip is not enabled, this flag should not do anything
            if (state)
            {
                // flag as tripped for alarm sending
                flag.reedTripped = true;
                flag.reedWakeupCount = 0;
            }
        }

        
        // we do not check if reed occurred prior to tamper or vice versa
        // so if both codes are sent reed code will always arrive first at receiver
        if (flag.tamperInterrupted)
        {            
            // FIXME: another terrible debounce
            delay10us(DEBOUNCE_TIME_10US);
            flag.tamperInterrupted = false;

            
            index = flag.eventCount;
            
            state = isTamperOpen();
            
            if (state)
            {
                flag.eventHistory[index] = gTamperOpen;
            } else {
                flag.eventHistory[index] = gTamperClose;
            }
            
            flag.eventCount++;
            

            // same trip behavior
            if (state)
            {
                // flag as tripped for sending elsewhere
                flag.tamperTripped = true;
                flag.tamperWakeupCount = 0;
            }
        }
        
        
        // send any stored event(s) over radio if count is incremented
        // we need to start at index zero every time (the oldest event)
        index = 0;
        while (flag.eventCount > 0)
        {
            // this is smarter than using using blocking delays to pulse led
            led_on();
            
            enable_radio_vdd();
            delay1ms(RADIO_STARTUP_TIME);
            
            // many receivers require repeatedly sending identical transmissions to accept data
            for (repeatIndex = 0; repeatIndex < REPEAT_TRANSMISSIONS; repeatIndex++)
            {
                packet[0] = *guid0;
                packet[1] = *guid1;
                packet[2] = flag.eventHistory[index];

                // DEBUG: for checking arbitrary data lengths at receiver
                packet[3] = 0x4A;
                packet[4] = 0x55;
                
                
                // send radio packet consisting of start pulse and 24-bits of data
                send_radio_packet(&packet[0], bitsInPacket);
            }
            
            
            // FIXME: we need to force ask low/high just in case correct?
            disable_radio_vdd();
    
            
            // delay before sending next packet (if available) so as not to overwhelm receiver
            delay1ms(RADIO_GUARD_TIME);
            
            // effectively just pulsed led due to inherent delays of radio sending
            led_off();
            
            // count up from zero (i.e., send oldest to newest event in that order)
            index++;
            flag.eventCount--;
            
            // track this for debugging purposes
            flag.packetCount++;
        }
        

    } // main while loop
}
