#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
  
void OledInitialization() {
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

    // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("OpenRadiation V2");
  display.println("Test SW");
  display.setCursor(0,0);
  display.display(); // actually display all of the above
}

void OledClear()
{
  display.clearDisplay();
}
void OledLine1(void)
{
  display.setCursor(0,0);
  display.println("OpenRadiation V2");
  display.display(); // actually display all of the above
}
void OledLine2(void)
{
  display.setCursor(0,1);
  display.println("Test SW");
  display.display(); // actually display all of the above
}
void OledLine3(void)
{
  
}
void OledLine4(void)
{
  
}
