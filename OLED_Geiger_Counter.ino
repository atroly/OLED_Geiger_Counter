/*
 * Code for Arduino Geiger counter front end with OLED display
 *
 * Copyright (C) 2018 Chris Taylor
 *
 * This program is free software: you can redistribute it and/or modify
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <Arduino.h>
#include <EEPROM.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define COUNT_SIZE 20

const float ver = 1.2;          //  firmware version
const int ledPin = LED_BUILTIN; //  output pin for LED
const int batt_pin = A1;        //  input pin for the battery voltage
const int modePin = 2;          //  input pin for the mode button
const int countPin = 3;         //  input pin for the counter signal
const long interval = 1000;     //  update interval - allows for display * calculation (milliseconds)

// U8g2 Contructor (Picture Loop Page Buffer)
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R2, /* clock=*/ 12, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ U8X8_PIN_NONE);

// End of constructor list

volatile unsigned int switch_mode = 0; // current mode as set by panel switch
int newmode = 1;                       // flag set by switch interrupt
volatile unsigned int count = 0;       // number of geiger tube events since last reset
char str[12];                           // holds string value to display
int first_time = 1;                    // special treatment of counts per minute until COUNT_SIZE is complete
unsigned int per_sec[COUNT_SIZE];      // array for counts per minute calculation;
unsigned int seconds = 0;              // index into per_sec array
unsigned int current_count = 0;        // current display counts per second
unsigned int max_count = 0;            // maximum recorded counts per second
unsigned int eeprom_index = 0;         // index into EEPROM

void setup(void) {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(modePin, INPUT_PULLUP);
  u8g2.begin();
  Serial.println("Initialising counter");
  for (int i = 0 ; i < (EEPROM.length() - 1) ; i++) {
    if ((EEPROM.read(i) == 255) && (EEPROM.read(i+1) == 255)) {
      eeprom_index = i;                 // set index counter to next EEPROM location
      break;
    }
    sprintf(str, "%d %d", i, EEPROM.read(i)); // output saved values
    Serial.println(str);
  }
  if (digitalRead(modePin) == LOW) {   // reset EEPROM index counter if button held on powerup
    Serial.println("EEPROM reset");
    display(str, "EEPROM reset");  
    eeprom_index=0;   
    delay(2000);
  }
  dtostrf(ver, 6, 2, str); // arduino can't sprintf floats
  display(str, "Firmware Version");
  attachInterrupt(digitalPinToInterrupt(modePin), modeHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(countPin), countHandler, FALLING);
  delay(1000);                       // let user read firmware message
}

void loop(void) {
  if (newmode || update_check()) {
    if (switch_mode == 0)
      show_cpm();
    if (switch_mode == 1)
      show_cps();
    if (switch_mode == 2)
      show_max();
    if (switch_mode == 3)
      show_index();
    if (switch_mode == 4)
      voltage();
    newmode=0;  
  }
}



void show_cpm(void) {
  int i;
  long total;
  
  total=0;
  if (first_time) {
    for (i=0;i<=seconds;i++)
      total+=per_sec[i];   
    total = (total*COUNT_SIZE)/(seconds+1);  
  } else {
    for (i=0;i<COUNT_SIZE;i++)
      total+=per_sec[i];
  }
  total=total*60/COUNT_SIZE;   // adjust to counts per minute
  sprintf(str, "%06ld", total);
  display(str, "Counts per minute");

}

void show_cps(void) {
  sprintf(str, "%06d", current_count);
  display(str, "Counts per second");
}

void show_max(void) {
  sprintf(str, "%06d", max_count);
  display(str, "Maximum CPS");
}

void show_index(void) {
  sprintf(str, "%06d", eeprom_index);
  display(str, "EEPROM Index");
}

void voltage(void) {
  static int setmv;
  int mv;
  float volts;

  mv = analogRead(batt_pin) * readVcc() / 1024;
  if (newmode || abs(mv - setmv) > 35) {
    setmv=mv;  
    volts = mv / 1000.0;
    dtostrf(volts, 6, 2, str); // arduino can't sprintf floats
    display(str, "    battery voltage");
  }
}

void display(char *value, const char *text) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso32_tn);
    u8g2.drawStr(0,40,value);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(0,61,text);    
  } while ( u8g2.nextPage() );  
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1120000L / result; // Back-calculate AVcc in mV (corrected for RobotDyn Nano)
  return result;
}

int update_check() {
  // check to see if it's time to update the count variables
  // and flash heartbeat LED
  // 
  static int ledState = LOW;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {    
    previousMillis = currentMillis; // save last update time
    per_sec[seconds]=count;
    current_count=count;
    if (count > max_count) {
      max_count = count;
    }
    if (eeprom_index < EEPROM.length() - 2) {
      if (count < 128) {
        EEPROM.write(eeprom_index, (byte)count);
        ++eeprom_index;
      }
      else {
        EEPROM.write(eeprom_index, (byte)((count >> 8)) + 128);
        ++eeprom_index;
        EEPROM.write(eeprom_index, (byte)count);
        ++eeprom_index;
      }
      EEPROM.write(eeprom_index, 255);
      EEPROM.write(eeprom_index+1, 255);
    }
    count=0;
    if (++seconds == COUNT_SIZE) {
      seconds=0;
      first_time=0;
    }
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);  // heartbeat
    return 1;
  }
  return 0;
}

#define BOUNCE_DURATION 200   // define an appropriate bounce time in ms

volatile unsigned long bounceTime=0; // ms count to debounce a pressed switch

void modeHandler() {  
// this is the interrupt handler for button presses
// it ignores presses that occur in intervals less then the bounce time
  if (millis() > bounceTime) {
    newmode = 1;
    ++switch_mode;
    if (switch_mode > 4) {
      switch_mode=0;
    }
    bounceTime = millis() + BOUNCE_DURATION;  // set whatever bounce time in ms is appropriate
  }
}

void countHandler() {
  ++count;
}

