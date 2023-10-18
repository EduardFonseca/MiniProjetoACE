// LONG PRESS E IMPORTANTE TER UMA MAQUINA DE ESTADOS SO PRA ISSO
// 

#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 6

#define Sgo_pin 2
#define Sup_pin 3
#define Sdown_pin 4

typedef struct {
  int state, new_state, prev_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

// Input variables
uint8_t Sgo, prevSgo;
uint8_t Sup, Sdown;
uint8_t prevSup, prevSdown;

// Output variables
// uint8_t LED_1 = 1, LED_2 = 1;

// Our finite state machines
fsm_t fsm1, fsm2, fsm3;

unsigned long blink_period = 500;
int blink_var = 0; // se variavel for 0 maquina 2 deve estar em standby se for 1 ela entra em funcionameto

unsigned long interval, last_cycle;
unsigned long loop_micros;

int n_led = 5;
unsigned long time_elapsed=0;

int led_on = n_led-1;
int led_added = 0;

unsigned long press_time = 3000;
int config_mode = 0;
int config_led = 0;

uint16_t count_time = 10; //seconds
uint16_t count_ms = count_time * 1000; //miliseconds
uint16_t t_led = count_ms/n_led; //miliseconds

// inicializacao da strip
NeoPixelConnect strip(LED_PIN, MAXIMUM_NUM_NEOPIXELS, pio0, 0);

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.prev_state = fsm.state;
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void setup() 
{
  pinMode(LED_BUILTIN,OUTPUT);

  pinMode(Sgo_pin, INPUT_PULLUP);
  pinMode(Sup_pin, INPUT_PULLUP);
  pinMode(Sdown_pin, INPUT_PULLUP); 

  interval = 40;
  last_cycle = millis();

  // Start the serial port with 115200 baudrate
  Serial.begin(115200);
  set_state(fsm1, 0);
  set_state(fsm2, 0);    
  set_state(fsm3, 0);    
}

void loop() 
{
    
    uint8_t b;
    if (Serial.available()) {  // Only do this if there is serial data to be read
      b = Serial.read();       
    }  
    // To measure the time between loop() calls
    //unsigned long last_loop_micros = loop_micros; 
    
    // Do this only every "interval" miliseconds 
    // It helps to clear the switches bounce effect
    unsigned long now = millis();
    if (now - last_cycle > interval) {
      loop_micros = micros();
      last_cycle = now;


      // Read the switches
      prevSgo = Sgo;
      prevSup = Sup;
      prevSdown = Sdown;
      Sgo = !digitalRead(Sgo_pin);
      Sup = !digitalRead(Sup_pin);
      Sdown = !digitalRead(Sdown_pin);
      // FSM processing

      // Update tis for all state machines
      unsigned long cur_time = millis();   // Just one call to millis()
      fsm1.tis = cur_time - fsm1.tes;
      fsm2.tis = cur_time - fsm2.tes; 
      fsm3.tis = cur_time - fsm3.tes; 

      // TODO: ARRUMAR O ESTADO DE CONFIGURACAO
      // Calculate state transitions for the first state machine
      // state0 = initial state
      // state1 = countdown state
      // state2 = finished countdown state
      // state3 = paused state
      if(config_mode == 0){
        if(Sgo && !prevSgo && fsm1.state == 0) {
          // first transition
          // START COUNTDOWN
          fsm1.new_state = 1;
        }else if(fsm1.state == 1 && 1+led_on == 0){
          // second transition
          // END COUNTDOWN
          time_elapsed += fsm1.tis;
          fsm1.new_state = 2;
        }else if(Sgo && fsm1.state == 2){
          // third transition
          // reset when finished
          fsm1.new_state = 0;
        }else if(Sdown && !prevSdown && fsm1.state == 1){
          // fourth transition
          // PAUSE COUNTDOWN
          fsm1.new_state = 3;
        }else if(Sdown && !prevSdown && fsm1.state == 3){
          // fifth transition
          // RESUME COUNTDOWN
          fsm1.new_state = 1;
        }else if(Sgo && fsm1.state == 3){
          // sixth transition
          // Reset when paused
          fsm1.new_state = 0;
        }
      }
      
      
      // Calculate state transitions for the second state machine
      // state0 = standby
      // state1 = led_on
      // state2 = led_off
      if(fsm2.state==0 && (blink_var == 1 || config_mode == 1)){
        fsm2.new_state = 1;
      }else if(blink_var == 0 && config_mode == 0){
        fsm2.new_state = 0;
      }else if(fsm2.state == 1 && fsm2.tis >= blink_period){
        fsm2.new_state = 2;
      }else if(fsm2.state == 2 && fsm2.tis >= blink_period){
        fsm2.new_state = 1;
      }
      
      // Calculate state transitions for the third state machine
      // state0 = standby
      // state1 = button pressed
      // state2 = config mode
      if(fsm1.state==0){
        if(Sup && !prevSup && fsm3.state != 1){
          config_led += 1;
          if(config_led > n_led-1){
            config_led = 0;
          }
          fsm3.new_state = 1;
          fsm3.new_state = 1;
        }else if(!Sup && fsm3.state == 1){
          fsm3.new_state = fsm3.prev_state;
        }else if(fsm3.state == 1 && fsm3.tis >= press_time && fsm3.prev_state == 0){
          fsm3.new_state = 2;
        }else if(fsm3.state == 1 && fsm3.tis >= press_time && fsm3.prev_state != 0){
          fsm3.new_state = 0;
        }
      }

      // Calculate next state for the first state machine
      if(config_mode == 0){
        if(fsm1.state == 0){
        //reset initial state
        led_on = n_led-1; // reset all pixels to one
        time_elapsed = 0; // reset time elapsed
        blink_var = 0;    // reset blink var to off
        led_added = 0;    // reset led added to 0

        // turn off all pixels green
        for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
          strip.neoPixelSetValue(i, 0, 25, 0);  
        }
        }else if(fsm1.state == 1){
      //countdown
        blink_var = 0; //reset blink var to off
        
        // Add LED IF Sup is pressed
        if(Sup && !prevSup && led_on < n_led-1){
          Serial.print("Sup pressed");
          led_on+=1;
          led_added+=1;
        }

        // if time elapsed and time in stae is grater time for the led to be on
        if(time_elapsed+fsm1.tis-(led_added*t_led) >= (5-led_on)*t_led){
          //Turn off the led
          led_on-=1;
        }
        
        if(fsm1.new_state==3){
          // if paused update time elapsed
          time_elapsed += fsm1.tis;
        }

        // turn on the right pixels  
        for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
          if(i<=led_on){
            strip.neoPixelSetValue(i, 25, 25, 0);  
          }else{
            strip.neoPixelSetValue(i, 0, 0, 0);  
          }     
        }

        }else if(fsm1.state == 2){
        //finished countdown
        blink_var = 1; // turn on blinking mode

        // turn off all pixels red
        for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
          strip.neoPixelSetValue(i, 25, 0, 0);  
        }

        }else if(fsm1.state == 3){
        //paused
        blink_var = 1; // turn on blinking mode

        //turn on the right pixels
        for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
        if(i<=led_on){
          strip.neoPixelSetValue(i, 25, 25, 0);  
        }else{
          strip.neoPixelSetValue(i, 0, 0, 0);  
        }
        }     
      }
      }
      
      // Calculate next state for the second state machine
      if(fsm2.state == 0){
        //Show LED 
        strip.neoPixelShow();
      }else if(fsm2.state == 1){
        //Show LED 
        strip.neoPixelShow();
      }else if(fsm2.state == 2){
        //Show LED 
        strip.neoPixelClear();
      }

      // Calculate next state for the third state machine
      if(fsm3.state==0){
        // wait for button press
        config_led = 0;
        config_mode = 0;
      }else if(fsm3.state==1){
        // wait for time
        config_mode = 1;
      }else if(fsm3.state==2){
        config_mode = 1;
        // turn on the right pixels
        for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
          if(i == config_led){
            strip.neoPixelSetValue(i, 25, 25, 0);  
          } else{
            strip.neoPixelSetValue(i, 0, 0, 0);
          }
        }
      } 


      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);


      // Debug using the serial port
      Serial.print("Sgo: ");
      Serial.print(Sgo);
      Serial.print(" Sup: ");
      Serial.print(Sup);
      Serial.print(" Sdown: ");
      Serial.print(Sdown);

      Serial.print(" fsm1.state: ");
      Serial.print(fsm1.state);
      
      Serial.print(" fsm1.tis: ");
      Serial.print(fsm1.tis);

      Serial.print(" fsm2.state: ");
      Serial.print(fsm2.state);
      
      Serial.print(" fsm2.tis: ");
      Serial.print(fsm2.tis);

      Serial.print(" fsm3.state: ");
      Serial.print(fsm3.state);

      Serial.print(" fsm3.tis: ");
      Serial.print(fsm3.tis);

      Serial.print(" blink_var: ");
      Serial.print(blink_var);

      Serial.println();

    }
    
}

