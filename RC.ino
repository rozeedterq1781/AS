#define PPM_PIN 2

#include <Servo.h>

bool isArmed = false;
bool isConnected = false;
bool isFirstConnect = true;

int throttle, roll, pitch, yaw;

Servo esc1, esc2, esc3, esc4;

volatile uint16_t ppm_values[8];

void readPPM();

void setup() {
   Serial.begin(115200);
   pinMode(PPM_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, RISING);
   esc1.attach(3), esc2.attach(4), esc3.attach(5), esc4.attach(6);
}

void loop() {
   if (isConnected) {
     if (ppm_values[6] < 1100) {
       esc1.writeMicroseconds(0);
       esc2.writeMicroseconds(0);
       esc3.writeMicroseconds(0);
       esc4.writeMicroseconds(0);
       if (isArmed) {
         Serial.print("Disconnect : ");
         Serial.println(ppm_values[6]);
         isArmed = false;
       }
     } else if (ppm_values[6] < 2000) {
       esc1.writeMicroseconds(1000);
       esc2.writeMicroseconds(1000 );
       esc3.writeMicroseconds(1000);
       esc4.writeMicroseconds(1000);
       if (!isArmed) {
         if (isFirstConnect) {
           Serial.print("");
           isFirstConnect = false;
         }
         Serial.print("Connect : ");
         Serial.println(ppm_values[6]);
         isArmed = true;
       }
       Serial.print("Roll:");
if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
else Serial.print("-+-");
Serial.print(receiver_input_channel_1);

Serial.print(" Nick:");
if(receiver_input_channel_2 - 1480 < 0)Serial.print("<<<");
else if(receiver_input_channel_2 - 1520 > 0)Serial.print(">>>");
else Serial.print("-+-");
Serial.print(receiver_input_channel_2);

Serial.print(" Throttle:");
if(receiver_input_channel_3 - 1050 < 0)Serial.print("<<<");
else if(receiver_input_channel_3 - 1950 > 0)Serial.print(">>>");
else Serial.print("-+-");
Serial.print(receiver_input_channel_3);

Serial.print(" Yaw:");
if(receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
else if(receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
else Serial.print("-+-");
Serial.println(receiver_input_channel_4);
}
       }
     } else {
       Serial.println("PPM Ready!");
       isFirstConnect = true;
     }
     delay(50);
   }
}


void readPPM() {
  static uint8_t counter = 0;
  static uint32_t last_interrupt_time = 0;
  uint32_t interrupt_time = micros();
  uint32_t time_since_last_interrupt = interrupt_time - last_interrupt_time;

  if (time_since_last_interrupt > 5000) {
    counter = 0;
  } else {
    ppm_values[counter] = time_since_last_interrupt;
    counter++;
  }

  last_interrupt_time = interrupt_time;

  if (counter == 8) {
    counter = 0;
    isConnected = true;
  }
}