#include <LiquidCrystal_I2C.h>
/*#include <Wire.h>*/

byte paired = 0;
byte I2C_address = 0x27;
/*byte count = 0;*/
char junk;
char object_status;
String inputString_x="";
String inputString_y="";
byte a[9];


LiquidCrystal_I2C lcd(I2C_address, 16, 2);

void setup()                    
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();   
  lcd.print("Waiting for");
  lcd.setCursor(0,1); 
  lcd.print("BT pairing...");
  /*
  Wire.begin();
  Serial.println ("I2C address scanning...");
  for(byte i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0)
    {
      I2C_address = i;
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
    }
  }
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  */
}

void loop()
{
  if(Serial.available())
  {
    if(paired == 0)
    {
      lcd.clear();
      lcd.print("x data    y data");
      paired = 1;
    }
    
    Serial.readBytes(a, 9);
    lcd.setCursor(0,1);
    
    for(byte i = 1; i < 9; i++)
    {
      lcd.print((char)a[i]);
      if(i == 4)
      {
        lcd.setCursor(10,1);
      }
    }              
    delay(500);
  }
}
