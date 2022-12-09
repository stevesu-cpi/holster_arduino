//this code displays number seconds and counts when digital pin goes from low to high
#include <LiquidCrystal.h>
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//int pin_counter_no = 12;
int analog_pin = A1;
int current_count = 0;
int previous_counter_status = LOW;
int current_counter_status = LOW;


void setup()
{
 lcd.begin(16, 2);              // start the library
 lcd.setCursor(0,0);
 lcd.print("seconds  count"); // print a simple message
 lcd.setCursor(9,1);            // move cursor to second line "1" and 9 spaces over
 lcd.print('0');
 pinMode(analog_pin, INPUT);
 Serial.begin(9600);
}
  
void loop()
{
 lcd.setCursor(0,1);            // move to the begining of the second line
 lcd.print(millis()/1000);      // display seconds elapsed since power-up
 
 lcd.setCursor(9,1);            // move cursor to second line "1" and 9 spaces over
 delay(100);
 Serial.print("previous_counter_status: ");
 Serial.println(previous_counter_status);
 current_counter_status = analogRead(analog_pin);
//Serial.print("analog read ");
// Serial.println(current_counter_status);
 if (current_counter_status > 500)
  current_counter_status = HIGH;
 else
  current_counter_status = LOW;
 

  Serial.print("current_counter_status: ");
  Serial.println(current_counter_status);
 
 if (previous_counter_status == LOW && current_counter_status == HIGH) {
  current_count++;
  Serial.print("*********current count ******* ");
  Serial.println(current_count);
  lcd.print(current_count);
  previous_counter_status = HIGH;
 }
 if (previous_counter_status == HIGH && current_counter_status == LOW) {
  previous_counter_status = LOW;
 }
 

//if previous_counter_status == HIGH && current_counter_status == HIGH continue with same status
//if previous_counter_status == LOW && current_counter_status == LOW continue with same status


}
