/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */
bool buttonstate = false;
bool buttonreset = false;
int reset = 3;
bool go_crazy = 0;
int rescue = 0;
int bCounter = 0;
#define button1 6
//d6 button
bool prev_button_state1 = false;
bool buttonstate1 = false;

#define button2 4
//d4 button
bool prev_button_state2 = false;
bool buttonstate2 = false;

#define button3 3
//d3 button
bool prev_button_state3 = false;
bool buttonstate3 = false;

#define button4 5
//d5 button
bool prev_button_state4 = false;
bool buttonstate4 = false;

int delayTime = 100;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  pinMode(button1,INPUT_PULLUP);
  pinMode(button2,INPUT_PULLUP);
  pinMode(button3,INPUT_PULLUP);
  pinMode(button4,INPUT_PULLUP);
  Serial.begin(115200,SERIAL_8N1);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);

  //Read input on analog pin A1
  int sensorValueY = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltageY = sensorValueY * (5.0 / 1023.0);

  int bState = 0;

  prev_button_state1 = buttonstate1;
  buttonstate1 = digitalRead(button1);
  if (buttonstate1 != prev_button_state1 and !buttonstate1){
  bState = 6;
  }

  prev_button_state2 = buttonstate2;
  buttonstate2 = digitalRead(button2);
  if (buttonstate2 != prev_button_state2 and !buttonstate2){
  bState=4;
  }


  prev_button_state3 = buttonstate3;
  buttonstate3 = digitalRead(button3);
  if (buttonstate3 != prev_button_state3 and !buttonstate3){
  bState=3;
  }

  prev_button_state4 = buttonstate4;
  buttonstate4= digitalRead(button4);
  if (buttonstate4 != prev_button_state4 and !buttonstate4){
  bState = 5;
  }
  


  
  // print out the value you read:
  Serial.print("d: ");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print(voltage);
  Serial.print("\t");
  Serial.print(sensorValueY);
  Serial.print("\t");
  Serial.print(voltageY);
  Serial.print("\t");
  Serial.println(bState);
//  Serial.print("\t");
//  Serial.print(reset);
//  Serial.print("\t");
//  Serial.println(go_crazy);
  delay(delayTime);


}
