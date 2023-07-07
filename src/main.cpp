#include <Arduino.h>
#include "Mouse.h"

// put function declarations here:

#include <util/atomic.h>
//rotary encoder sketch for arduino leonardo

 static volatile int16_t position, prevposition;
static volatile uint8_t preva, prevb;

#define ca 0 //current a
#define cb 1 //current b
#define pa 2 //past a
#define pb 3 //past b

void update () {  
  uint8_t a, b;
  b = (PIND & (1<<PIND0)) >> PIND0;
  a = (PIND & (1<<PIND1)) >> PIND1;
  //noise causes multiple interrupts for same value
  if((a == preva) && (b == prevb)) return;  
  uint8_t r = 0;
  if(a) r|= (1<<ca);
  if(b) r|= (1<<cb);
  if(preva) r |= (1<<pa);
  if(prevb) r |= (1<<pb);
  if((r == 2) || (r == 11) || (r == 13) || (r == 4)) position++;   
  else if((r==14) || (r==7) || (r ==1) || (r == 8)) position--;
  else return;
  preva = a;
  prevb = b;  
 };


// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
//RotaryEncoder *encoder = nullptr;


// void checkPosition()
// {
//   encoder->tick(); // just call tick() to check the state.
// }

 void InitEncoder() {
     // set pin a and b to be input
  pinMode(0, INPUT_PULLUP); 
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);      
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
};

volatile uint8_t ledpin = 0;

void mouseclick() {
  Mouse.click();
}
 

void setup() {
 // Serial.begin(9600);
 // Serial.println("Initialize Serial Monitor");

  InitEncoder();
  attachInterrupt(digitalPinToInterrupt(3), update, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(2), update, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(0), mouseclick, CHANGE) ;
 // Mouse.begin();

  //interruptenable registers, falling edge
//  EICRA |= (1<<ISC20) | (1<<ISC21) | (1<<ISC11) | (1<<ISC01);
  
  //11.1.3 External Interrupt Mask Register – EIMSK
  /*When an INT[6;3:0] bit is written to one and the I-bit in the Status Register (SREG) is set (one), the
  corresponding external pin interrupt is enabled */
//  EIMSK |= (1<<INT2) | (1<<INT1) | (1<<INT0);
  
  //11.1.7 Pin Change Mask Register 0 – PCMSK
  //Each PCINT7..0 bit selects whether pin change interrupt is enabled on the corresponding I/O pin
//  PCMSK0 |= (1<<PCINT2) | (1<<PCINT1) | (1<<PCINT0);
  
  //11.1.5 Pin Change Interrupt Control Register - PCICR
  /*When the PCIE0 bit is set (one) and the I-bit in the Status Register (SREG) is set (one), pin change interrupt 0 is
  enabled. */
//  PCICR |= (1<<PCIE0); 
  
  //power down registers
  //SMCR |= (1<<SM2) | (1<<SM1) | (1<<SM0);
  //PRR0 |= (1<<PRTWI) | (1<<PRSPI) | (1<<PRADC);  
}

void loop() {
  //int newPos = encoder->getPosition();
  // put your main code here, to run repeatedly:  
  if(position != prevposition) {        
    Mouse.move((position - prevposition)*10,0,0);
    //Serial.println(position);
    prevposition = position;         
  }   
}
