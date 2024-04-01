// /// амплитудный анализ звука

// #include <Arduino.h>
// #include <U8g2lib.h>

// #include <Wire.h>

// U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// //U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

// #define U8LOG_WIDTH 20
// #define U8LOG_HEIGHT 8
// uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
// U8G2LOG u8g2log;


// #include "VolAnalyzer.h"
// VolAnalyzer analyzer(A0);
// #define dig D3;

// void setup() {
//   Serial.begin(115200);

//   u8g2.begin();  
//   u8g2.setFont(u8g2_font_5x7_tr);	// set the font for the terminal window
//   u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
//   u8g2log.setLineHeightOffset(0);	// set extra space between lines in pixel, this can be negative
//   u8g2log.setRedrawMode(1);
// }

// void loop() {
//   char c;
//   while (Serial.available() > 0) {
//     c = Serial.read();			// read from Serial Monitor
//     u8g2log.print(c);               // print to display
//     Serial.print(c);                // and print back to monitor
//     Serial.print(">Analog: ");
//     u8g2log.print(">Analog: ");
//     Serial.println(analogRead(A0));
//     u8g2log.println(analogRead(A0));

//   if (analyzer.tick()) {
//     // Serial.print(">Puls Volume: ");
//     // Serial.println(analyzer.pulse() * 20); // скачок громкости

//     // Serial.print(">Volume: ");
//     // Serial.println(analyzer.getVol());    // громкость 0-100

//     Serial.print(">RAW: ");
//     Serial.println(analyzer.getRaw());    // сырая величина

//     Serial.print(">AnalogTick: ");
//     Serial.println(analogRead(A0));

//     Serial.print(">MaxAmplitude: ");
//     Serial.println(analyzer.getMax());  // амплитудная огибающая
//   }
//   }


// }
/*

  StateBufferLoop.ino
  
  This example implements the for/next picture drawing loop as a state machine.
  
  Pro: Display refresh is distributed over several steps (only one page is drawn at a time,
  which consumes lesser time than refreshing the complete screen)
  
  Contra: (A) Artefacts may be visible. (B) It is more difficult to switch between different screens.

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

void draw(void);
void high_speed_task(void);
void u8g2_prepare(void);


// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Picture Loop Page Buffer)
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

int pinData = A0;
int pinLed = LED_BUILTIN;
int pinPot = A0;

float mass[44];
const float average = 2;
int potValue;
int potValuePrev;

float readData;


void setup(void) {

  /* U8g2 Project: SSD1306 Test Board */
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  pinMode(pinLed, OUTPUT);

  u8g2.begin();  
  u8g2.setFont(u8g2_font_6x10_tf);
}

uint16_t seconds = 0;


// this task should be called as often as possible
void high_speed_task(void) {
  // place your commands  here, they will be executed very often
  // as an example, calculated the seconds since startup...
  seconds = millis() / 1000;
}


// this procedure contains the u8g2 draw commands to draw someting on the screen
// void draw() {
//     u8g2.setCursor(0,10);
//     u8g2.print(seconds);
// //    u8g2.drawLine();
// }


// this is the state machine, which will replace the do - while loop
// void draw_page(void) {
//   static uint8_t is_next_page = 0;
  
//   // call to first page, if required
//   if ( is_next_page == 0 )
//   {
//     u8g2.firstPage();
//     is_next_page = 1;
//   }
  
//   // draw our screen
//   draw();
  
//   // call to next page
//   if ( u8g2.nextPage() == 0 ) {
//     is_next_page = 0;			// ensure, that first page is called
//   }  
// }

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void draw(void) {
  u8g2_prepare();
}

// arduno main loop
void loop(void) {
  high_speed_task();
  //draw_page();

  for (int i = 0; i <= 130; i = i+3) {
    u8g2.drawLine(i, 64-mass[i/3], i+3, 64-mass[(i+3)/3]);
  }

  readData = map(analogRead(pinData), 0, 1023, 0, 64);
  int Prev = mass[0];
  int Prev2;

  mass[0] *= average-1;
  mass[0] += readData;
  mass[0] /= average;

  for (int i = 1; i <= 43; i++) {
    Prev2 = mass[i];
    mass[i] = Prev;
    Prev = Prev2;
  }

  for (int i = 1; i <= 130; i = i+3) {
    u8g2.drawLine(i, 64-mass[i/3], i+3, 64-mass[(i+3)/3]);
  }

  potValue = analogRead(pinPot/16);
  if (potValuePrev != potValue) {
    u8g2.drawLine(0, potValuePrev, 130, potValuePrev);
    potValuePrev = potValue;
  }
  u8g2.drawLine(0, potValue, 130, potValue);

  if (mass[0] > 64 - potValue) {
    digitalWrite(pinLed, HIGH);
  }
  else {
    digitalWrite(pinLed, LOW);
  }

  u8g2.firstPage();  
  do {
    draw();
  } while( u8g2.nextPage() );
}

