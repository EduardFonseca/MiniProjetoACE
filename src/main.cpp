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
fsm_t fsm1, // countdown state machine
      fsm2, // blink pause and stop
      fsm3, // config mode state machine
      fsm4, // second half blink
      fsm5, // control config example led
      fsm6, // auxiliar state machine
      fsm7; // idle state state machine

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

uint16_t t_led = 2000; //miliseconds

int possible_times[4] = {1000, 2000, 5000, 10000};

// violet, blue, cyan, green, yellow, orange or white
int led_color = 0;
int colors[7][3] = {{127, 0, 255}, {0,0,255},{0, 255, 255},{0,255,0},{255,255,0},{255,165,0},{255,255,255}};
int color[3] = {colors[led_color][0],colors[led_color][1],colors[led_color][2]};

char index_time = 1;
float percent;
int fsm4_flag = 0;
int fsm6_flag = 0;
int led_mode = 0;
unsigned long time_buffer = 0;

// idele variables
uint8_t hue = 0;
uint8_t step = 10;
int cycle = 0;

int idle_mode = 0;
unsigned long idle_time = 30000;

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

void led_effect(int mode, unsigned long time){
  if(mode == 0){
    // turn on the right pixels   
    for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
      if(i<=led_on){
        strip.neoPixelSetValue(i, color[0],color[1], color[2]);  
      }else{
        strip.neoPixelSetValue(i, 0, 0, 0);  
      }     
    }
  }else if (mode == 1){
    // for the second half of its interval the LED must blink
    if(((5-led_on)*t_led- time) <= t_led/2){
      // state machine 4
      if(fsm4.state == 0){
        fsm4_flag = 1;
      }
    }else{
        fsm4_flag = 0;
    }
  }else if(mode == 2){
    // fades out from 100% to 0% over time
    percent = ((5-led_on)*(float)t_led - (float)time)/(float)t_led;
    for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
      if(i<led_on){
        strip.neoPixelSetValue(i, color[0],color[1], color[2]);  
      }else if(i==led_on){
        strip.neoPixelSetValue(i, percent*color[0],percent*color[1], percent*color[2]);  
      }else{
        strip.neoPixelSetValue(i, 0, 0, 0);  
      }     
    }
  }
}

void RGB_wave(int step, uint8_t hue, int cycle){
  
  uint8_t hue_g;
  uint8_t hue_b;
  uint8_t hue_r;
  uint8_t r,g,b;
  if(cycle == 0){
  // mantain blue hue in 0 and incrise green while decreasing red
    hue_r = 215 - hue;
    hue_g = hue;
    hue_b = 0;
  }else if(cycle == 1){
  // mantain red hue in 0 and incrise blue while decreasing green
    hue_r = 0;
    hue_g = 214 - hue;   
    hue_b = hue;
  }else if(cycle == 2){
  // mantain green hue in 0 and incrise red while decreasing blue
    hue_r = hue;
    hue_g = 0;   
    hue_b = 215 - hue;
  }
  for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
    r = hue_r + i*step;
    g = hue_g + i*step;  
    b = hue_b + i*step;
    Serial.print(r);
    Serial.print(" , ");
    Serial.print(g);
    Serial.print(" , ");
    Serial.println(b);

      strip.neoPixelSetValue(i, r, g, b,true);
  }
}

// APAGAR
void config_leds(int led){
    for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
    if(i == led){
      strip.neoPixelSetValue(i, 25, 25, 0);  
    } else{
      strip.neoPixelSetValue(i, 0, 0, 0);
    }
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
  set_state(fsm4, 0);    
  set_state(fsm5, 0);    
  set_state(fsm6, 0);    
  set_state(fsm7, 0);
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
      fsm4.tis = cur_time - fsm4.tes; 
      fsm5.tis = cur_time - fsm5.tes;
      fsm6.tis = cur_time - fsm6.tes;
      fsm7.tis = cur_time - fsm7.tes;

      // TODO: ARRUMAR O ESTADO DE CONFIGURACAO
      // Calculate state transitions for the first state machine
      // state0 = initial state
      // state1 = countdown state
      // state2 = finished countdown state
      // state3 = paused state
      if(config_mode == 0 && idle_mode == 0){
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
      if(idle_mode == 0){     
        if(fsm2.state==0 && blink_var == 1){
          fsm2.new_state = 1;
        }else if(blink_var == 0){
          fsm2.new_state = 0;
        }else if(fsm2.state == 1 && fsm2.tis >= blink_period){
          fsm2.new_state = 2;
        }else if(fsm2.state == 2 && fsm2.tis >= blink_period){
        fsm2.new_state = 1;
      }
      }
      
      // Calculate state transitions for the third state machine
      // state0 = standby
      // state1 = button pressed
      // state2 = time config mode
      // state3 = counting efect config mode
      // state4 = colour config mode
      if(idle_mode == 0){
        if(Sup && !prevSup && fsm3.state != 1){
        // config_led += 1;
        // if(config_led > n_led-3){
        //   config_led = 0;
        // }
        fsm3.new_state = 1;
        }else if(!Sup && fsm3.state == 1){
          if(fsm3.prev_state == 0){
            fsm3.new_state = fsm3.prev_state; 
          }else{
            fsm3.new_state = fsm3.prev_state+1;
            if(fsm3.prev_state == 4){
              fsm3.new_state = 2;
            }
            config_led = fsm3.new_state-2;
          }
        }else if(fsm3.state == 1 && fsm3.tis >= press_time && fsm3.prev_state == 0){
          fsm3.new_state = 2;
        }else if(fsm3.state == 1 && fsm3.tis >= press_time && fsm3.prev_state != 0){
        fsm3.new_state = 0;
      }
      }
      
      // Calculate state transitions for the fourth state machine
      // state0 = standby
      // state1 = led_off
      // state2 = led_on
      if(fsm4.state == 0 && fsm4_flag == 1){
        fsm4.new_state = 1;
      }else if(fsm4.state == 1 && fsm4.tis >= t_led/16){
        fsm4.new_state = 2;
      }else if(fsm4.state == 2 && fsm4.tis >= t_led/16){
        fsm4.new_state = 1;
      }else if(fsm4_flag == 0){
        fsm4.new_state = 0;
      }

      // Calculate state transitions for the fifth state machine
      if(config_mode == 0){
        fsm5.new_state = 0;
      }else if(fsm5.state == 0 && config_mode == 1){
        fsm5.new_state = 1;
      }else if(fsm3.state != 0 && fsm3.state != 1 && fsm5.prev_state != fsm3.state-1 && fsm5.state != fsm3.state-1){
        fsm5.new_state = fsm3.state-1;
      }else if(fsm5.state == 1 && fsm5.tis >= t_led){
        fsm5.new_state = 11;
      }else if(fsm5.state == 11 && fsm5.tis >= 1000){
        fsm5.new_state = 1;
      }else if(fsm5.state == 2 && (led_mode == 0 || led_mode == 2) && fsm5.tis >= 2000){ 
        fsm5.new_state = 21;
      }else if(fsm5.state == 21 && fsm5.tis >= 1000){
        fsm5.new_state = 2;
      }

      // Calculate state transitions for the sixth state machine
      if(fsm6_flag == 0){
        fsm6.new_state = 0;
      }else if(fsm6.state == 0 && fsm6_flag == 1){
        fsm6.new_state = 1;
      }else if(fsm6.state == 1 && fsm6.tis >= 2000/2){
        time_buffer+=fsm6.tis;
        fsm6.new_state = 2;
      }else if(fsm6.state==2 && fsm6.tis >= 2000/16){
        time_buffer+=fsm6.tis;
        fsm6.new_state = 3;
      }else if(fsm6.state==3 && fsm6.tis >= 2000/16){
        time_buffer+=fsm6.tis;
        fsm6.new_state = 2;
      }else if(time_buffer > 2000){
        fsm6.new_state = 4;
      }else if(fsm6.state == 4 && fsm6.tis >= 1000){
        fsm6.new_state = 1;
      }

      // Calculate the state for the seventh state machine
      if(((fsm1.tis >= idle_time && fsm1.state != 1) || (fsm3.tis >= idle_time && fsm3.state != 0)) && (fsm7.state == 0 && fsm7.tis >= idle_time)){
        fsm7.new_state = 1;
      }else if((Sup || Sdown || Sgo) && fsm7.state == 1){
        fsm7.new_state = 0;
      }

      // Calculate next state for the first state machine
      if(config_mode == 0 && idle_mode == 0){
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
          led_on+=1;
          led_added+=1;
        }

        if(fsm1.new_state==3){
          // if paused update time elapsed
          time_elapsed += fsm1.tis;
        }

        led_effect(led_mode, time_elapsed+fsm1.tis-(led_added*t_led));
        // if time elapsed and time in stae is grater time for the led to be on
        // TODO: 
        if((time_elapsed+fsm1.tis-(led_added*t_led) >= (5-led_on)*t_led) && fsm1.new_state != 3){
          //Turn off the led
          led_on-=1;
        }
        // turn on the right pixels  

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
      // TODO: Para o exercicio e alterar essa maquina
      if(idle_mode == 0){
        if(fsm2.state == 0){
          //Show LED 
          // funcao (tipo mostragem)
          strip.neoPixelShow();
        }else if(fsm2.state == 1){
          //Show LED 
          strip.neoPixelShow();
        }else if(fsm2.state == 2){
        //Show LED 
        strip.neoPixelClear();
      }
      }
      
      // Calculate next state for the third state machine
      if(idle_mode == 0){
        if(fsm3.state==0){
          // wait for button press
          config_led = 0;
          config_mode = 0;
        }else if(fsm3.state==1){
          // wait for time
        }else if(fsm3.state==2){
          config_mode = 1;
          fsm1.new_state = 0;
          blink_var = 0;
          // turn on the right pixels
          if(Sdown && !prevSdown){
            if(index_time == 3){
              index_time = 0;
            }else{
              index_time += 1;
            }
            t_led = possible_times[index_time];
          }

          // config_leds(fsm3.state-2);
        }else if(fsm3.state == 3){
          config_mode = 1;
          fsm1.new_state = 0;
          // turn on the right pixels
          if(Sdown && !prevSdown){
            if(led_mode == 2){
              led_mode = 0;
            }else{
              led_mode += 1;
            }
          }

          // config_leds(fsm3.state-2);
        }else if(fsm3.state == 4){
        if(Sdown && !prevSdown){
          led_color +=1;
          if(led_color >= 7){
            led_color =0;
          }
        }

        color[0] = colors[led_color][0];
        color[1] = colors[led_color][1];
        color[2] = colors[led_color][2];
        // config_leds(fsm3.state-2);
      }
      }
      
      // Calculate next state for the fourth state machine
      if(config_mode == 0 && idle_mode == 0){
        if(fsm4.state == 0){
          for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
            if(i<=led_on){
              strip.neoPixelSetValue(i, color[0],color[1], color[2]);  
            }else{
              strip.neoPixelSetValue(i, 0, 0, 0);  
            }     
          }
        }else if(fsm4.state == 1){
          for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
            if(i<led_on){
              strip.neoPixelSetValue(i, color[0],color[1], color[2]);  
            }else{
              strip.neoPixelSetValue(i, 0, 0, 0);  
            }     
          }  
        }else if(fsm4.state == 2){
          for(uint8_t i = 0; i < MAXIMUM_NUM_NEOPIXELS; i++) {
            if(i<=led_on){
              strip.neoPixelSetValue(i, color[0],color[1], color[2]);  
            }else{
              strip.neoPixelSetValue(i, 0, 0, 0);  
            }     
          }  
        }
      }

      // Calculate next state for the fifth state machine
      if(fsm5.state == 0){
        // standby
        fsm6_flag = 0;
      }else if(fsm5.state == 1){
        // turn on pixels 0 and 4 (led_config and led_exmp)
        fsm6_flag = 0;
        strip.neoPixelFill(0,0,0);
        strip.neoPixelSetValue(config_led, 25, 25, 0);
        strip.neoPixelSetValue(4, 25, 25, 0);
      }else if(fsm5.state == 11){
        // turn on pixels 0 and 5 (led_config and led_exmp)
        fsm6_flag = 0;
        strip.neoPixelFill(0,0,0);
        strip.neoPixelSetValue(config_led, 25, 25, 0);
      }else if(fsm5.state == 2){
        strip.neoPixelFill(0,0,0);
        strip.neoPixelSetValue(config_led, 25, 25, 0);
        if(led_mode == 0){
          fsm6_flag = 0;
          strip.neoPixelSetValue(4, 25, 25, 0);
        }if(led_mode ==1){
          fsm6_flag = 1;
        }if(led_mode == 2){
          fsm6_flag = 0;
          if(fsm5.tis > 2000){
            percent = 0;
          }else{
            percent = (float)(2000-fsm5.tis)/(float)(2000);
          }
          strip.neoPixelSetValue(4, percent*25,percent*25, 0);
        }
      }else if(fsm5.state == 3){
        fsm6_flag = 0;
        strip.neoPixelFill(0,0,0);
        strip.neoPixelSetValue(config_led, 25, 25, 0);
        strip.neoPixelSetValue(4, color[0],color[1],color[2]);
      }else if(fsm5.state == 21){
        fsm6_flag = 0;
        strip.neoPixelFill(0,0,0);
        strip.neoPixelSetValue(config_led, 25, 25, 0);
      }

      // Calculate next state for the sixth state machine
      if(fsm6.state == 0){
        time_buffer = 0;
      }else if(fsm6.state == 1){
        strip.neoPixelSetValue(4, 25, 25, 0);
      }else if(fsm6.state == 2){
        // led apagado
      }else if(fsm6.state == 3){
        strip.neoPixelSetValue(4, 25, 25, 0);
      }else if(fsm6.state == 4){
        time_buffer = 0;
      }

      if(fsm7.state == 0){
        // standby
        idle_mode = 0;
        hue = 0;

      }else if(fsm7.state == 1){
        idle_mode = 1;

        RGB_wave(step,hue,cycle);
        hue++;
        if(hue >= 215){
          cycle += 1;
          if(cycle >=3){
            cycle = 0;
          }
          hue = 0;
        }
      }

      
      // Update the states
      set_state(fsm1, fsm1.new_state);
      set_state(fsm2, fsm2.new_state);
      set_state(fsm3, fsm3.new_state);
      set_state(fsm4, fsm4.new_state);
      set_state(fsm5, fsm5.new_state);
      set_state(fsm6, fsm6.new_state);
      set_state(fsm7, fsm7.new_state);

      // // Debug using the serial port
      // Serial.print("Sgo: ");
      // Serial.print(Sgo);
      // Serial.print(" Sup: ");
      // Serial.print(Sup);
      // Serial.print(" Sdown: ");
      // Serial.print(Sdown);

      Serial.print(" fsm1.state: ");
      Serial.print(fsm1.state);
      
      Serial.print(" fsm1.tis: ");
      Serial.print(fsm1.tis);

      // Serial.print(" fsm2.state: ");
      // Serial.print(fsm2.state);
      
      // Serial.print(" fsm2.tis: ");
      // Serial.print(fsm2.tis);

      // Serial.print(" fsm3.state: ");
      // Serial.print(fsm3.state);

      // Serial.print(" fsm3.tis: ");
      // Serial.print(fsm3.tis);

      // Serial.print(" fsm4.state: ");
      // Serial.print(fsm4.state);

      // Serial.print(" fsm4_flag: ");
      // Serial.print(fsm4_flag);

      // Serial.print(" fsm5.state: ");
      // Serial.print(fsm5.state);

      // Serial.print(" fsm5.tis: ");
      // Serial.print(fsm5.tis);

      // Serial.print(" fsm5.prev_state: ");
      // Serial.print(fsm5.prev_state);

      Serial.print(" fsm7.state: ");
      Serial.print(fsm7.state);

      Serial.print(" led_on: ");
      Serial.print(led_on);
      
      Serial.print(" t_led: ");
      Serial.print(t_led);

      Serial.print(" led_mode: ");
      Serial.print(led_mode);

      Serial.print(" time_elapsed: ");
      Serial.print(time_elapsed);

      Serial.print(" hue: ");
      Serial.print(hue);

      Serial.print(" percent: ");
      Serial.print(hue*step);


      Serial.println();

    }
    
}

