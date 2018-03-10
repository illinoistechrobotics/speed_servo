#include "globals.h"
#include <util/atomic.h>

#define WATCHDOG_
#ifdef WATCHDOG_
#include <avr/wdt.h>
#endif

void setup(){
  speed = 0;
  controller_output = 0;
  #ifdef WATCHDOG_
    wdt_reset();        //Watchdpg Timer Reset
    wdt_enable(WDTO_250MS);     //Set to expire in 250 MS
  #endif
  
  //Load saved tunings from EEPROM, initalize the controller
  v_pterm = load_controller_tuning(PTERM);
  v_iterm = load_controller_tuning(ITERM);
  v_dterm = load_controller_tuning(DTERM);
  controller.SetTunings(pterm,iterm,dterm);
  controller.SetMode(MANUAL);
  state = STOP;
  controller.SetOutputLimits(-1*PWM_WIDTH,PWM_WIDTH);
  controller.SetSampleTime(20);

  Serial.begin(9600);
  Serial.println("RESET");
  //A and B channels of the Quad encoder
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  attachInterrupt(0,spin,RISING);

  pinMode(VICTOR_PIN, OUTPUT);
  victor.attach(VICTOR_PIN);
  victor.writeMicroseconds(PWM_CENTER); 

  /*
     The Arduino has a ground pin, then pin 13, then pin 12
     We set 13 as the input, and set 12 high so we can jumper to 
     either GND or 12 (HIGH) to set the i2c address 
   */
  pinMode(ADDRESS_PIN, INPUT);
  pinMode(12,OUTPUT);
  digitalWrite(12,HIGH); 
  int address = (digitalRead(ADDRESS_PIN) == HIGH)?2:1;
  Serial.print("Address: ");
  Serial.println(address);
  Wire.begin(address);
  Wire.onReceive(receive);//This requires a protocol to understand which parameter is being changed
  Wire.onRequest(writeTicks);//This is always the speed
}

void loop(){
  /*
     We only set the external tuning variables in the i2c
     receive routine. We load the controller with new values periodically
     this assumes that you are setting the values infrequently, so the lag period 
     doesn't have a large impact.
   */
  if(millis() - last_tuning_update > 1000){
    controller.SetTunings(pterm,iterm,dterm);
    last_tuning_update = millis();
  }
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    speed = v_speed;
    pterm = v_pterm;
    iterm = v_iterm;
    dterm = v_dterm;
    ticks = v_ticks;
    if(state == RUN && controller.GetMode() == MANUAL){
      ticks = 0;
      v_ticks = 0;
      last_ticks = 0;
      controller.SetMode(AUTOMATIC);
    } else if(state == STOP){
      controller.SetMode(MANUAL);
    }
  }
  if(controller.Compute() || controller.GetMode() == MANUAL){
    if(controller.GetMode() != MANUAL){
      int output = (int)(controller_output) + PWM_CENTER;
      victor.writeMicroseconds(output);
     } else {
      //The controller is only in MANUAL when it has been estopped
      //Therefore, write the Victor's zero value
      victor.writeMicroseconds(PWM_CENTER);
    }
     ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
      v_ticks = 0;
  }
    if(pc == 0)
      printDebug();
    pc+=4;
    last_ticks = ticks;
   
  
  }
}

//Interupt handler volatile vars only!
void receive(int incoming){
  #ifdef WATCHDOG_
    wdt_reset();        //Watchdog Timer Reset
  #endif
  
  switch((enum command)Wire.read()){
  case SPEED:
    state = RUN;
    setParam(incoming, &v_speed);
    break;
  case PTERM:
    setParam(incoming, &v_pterm);
    save_controller_tuning(PTERM,v_pterm);
    break;
  case ITERM:
    setParam(incoming, &v_iterm);
    save_controller_tuning(ITERM,v_iterm);
    break;
  case DTERM:
    setParam(incoming, &v_dterm);
    save_controller_tuning(DTERM,v_dterm);
    break;
  case ESTOP:
    state = STOP;
    v_speed = 0;
   // controller_output = 0;
  }
}

void writeTicks(){
  Wire.write((char *) &last_ticks, sizeof(int));
}

void printDebug(){
    //All serial printing is for debugging only.
    //Control information 
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" ticks: ");
    Serial.print(ticks);
    Serial.print(" p: ");
    Serial.print(pterm);
    Serial.print(" i: ");
    Serial.print(iterm);
    Serial.print(" d: ");
    Serial.print(dterm);
    Serial.print(" OUTPUT: ");
    Serial.println(controller_output);
    if(controller.GetMode() == MANUAL && state == STOP){
      Serial.println("****ESTOP****");
    }
}
void setParam(int incoming, volatile int * dest){
  if(incoming != sizeof(int) +1){
    return;
  }
  *dest = Wire.read() + (Wire.read() << 8);
}
void setParam(int incoming, volatile double * dest){
  if(incoming != sizeof(double)+1){
    return;
  }
  byte buff[sizeof(double)];
  for(int i = 0; i < sizeof(double); ++i){
    buff[i] = Wire.read();
  }
  *dest = *((double *)buff);
}

void spin(){
  if(digitalRead(PIN_A) == HIGH && digitalRead(PIN_B) == LOW){
    v_ticks++;
  } else {
    v_ticks--;
  }
}
double load_controller_tuning(int tuning){
  double out;
  byte * reassembled = (byte *)&out;
  for(int i = 0; i < sizeof(double); ++i){
    *reassembled = EEPROM.read(sizeof(double)*tuning + i);
    reassembled++;
  }
  return out;
}
void save_controller_tuning(int tuning, volatile double value){
  byte * disassembled = (byte *)&value;
  for(int i = 0; i < sizeof(double); ++i){
    EEPROM.write(sizeof(double)*tuning + i, *disassembled);
    disassembled++;
  }
}
