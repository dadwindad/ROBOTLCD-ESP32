#include <math.h>
#include <JPEGDecoder.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>

const char* ssid     = "KAK-AP";
const char* password = "000000000";

bool serverStatus = false;

WiFiServer server(80);

TFT_eSPI tft = TFT_eSPI();

#include "menu.h"
#include "about.h"

#include <TFT_eFEX.h>              // Include the extension graphics functions library
TFT_eFEX  fex = TFT_eFEX(&tft);    // Create TFT_eFX object "efx" with pointer to "tft" object

//====================================================================
// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))
//====================================================================

String CurrentPage = "init";
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/

//####################################################################################################
// Print image information to the serial port (optional)
//####################################################################################################
void jpegInfo() {
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}

//####################################################################################################
// Draw a JPEG on the TFT, images will be cropped on the right/bottom sides if they do not fit
//####################################################################################################
// This function assumes xpos,ypos is a valid screen coordinate. For convenience images that do not
// fit totally on the screen are cropped to the nearest MCU size and may leave right/bottom borders.
void renderJPEG(int xpos, int ypos) {

  // retrieve infomration about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.read()) {
    
    // save a pointer to the image block
    pImg = JpegDec.pImage ;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // calculate how many pixels must be drawn
    uint32_t mcu_pixels = win_w * win_h;

    tft.startWrite();

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
    {

      // Now set a MCU bounding window on the TFT to push pixels into (x, y, x + width - 1, y + height - 1)
      tft.setAddrWindow(mcu_x, mcu_y, win_w, win_h);

      // Write all MCU pixels to the TFT window
      while (mcu_pixels--) {
        // Push each pixel to the TFT MCU area
        tft.pushColor(*pImg++);
      }

    }
    else if ( (mcu_y + win_h) >= tft.height()) JpegDec.abort(); // Image has run off bottom of screen so abort decoding

    tft.endWrite();
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;

  // print the results to the serial port
  Serial.print(F(  "Total render time was    : ")); Serial.print(drawTime); Serial.println(F(" ms"));
  Serial.println(F(""));
}

//####################################################################################################

void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos) {

  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);
  
  jpegInfo(); // Print information from the JPEG file (could comment this line out)
  
  renderJPEG(x, y);
  
  Serial.println("#########################");
}

//====================================================================
void setup(void) {
  //////////////////////SENSOR////////////////////////////////
  //sda,scl
  //Wire.begin(25,26);
  pinMode(13, OUTPUT); //IN1-M1
  pinMode(12, OUTPUT); //IN2-M1
  pinMode(14, OUTPUT); //IN3-M2
  pinMode(27, OUTPUT); //IN4-M2
  ////////////////////TFT//////////////////////////////////
  
  Serial.begin(115200);
  Serial.println("\n\nStarting...");

  uint16_t PosX, PosY;

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(0xFFFF);
  tft.setTextColor(0xFFFFFF);
  
  // Draw a jpeg image stored in memory at x,y
  //drawArrayJpeg(logoKKU, sizeof(logoKKU), 0, 0);

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    server.begin();

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

//====================================================================

void loop() {
  uint16_t PosX, PosY;
  static uint16_t color;
  boolean pressed = tft.getTouch(&PosX, &PosY);
//  //inverse y position
//  PosY = 320-PosY;

  //measure , measure525 and other page except menu cant go to menu
  if (pressed && CurrentPage != "menu" && CurrentPage != "measure" && CurrentPage != "measure525") {
    tft.fillScreen(0xFFFFFF);
    drawArrayJpeg(menu, sizeof(menu), 0, 0);
    CurrentPage = "menu";
    PosX = 1000, PosY = 1000;
  }

  if (PosX >=50 && PosX <=100 && PosY >=50 && PosY <=120 && CurrentPage == "menu") {
    tft.fillScreen(0xFFFFF);
    drawArrayJpeg(about, sizeof(about), 0, 0);
    CurrentPage = "about";
  }
  
  if (PosX >=0 && PosX <=0 && PosY >=0 && PosY <=0 && CurrentPage == "menu") {
    tft.fillScreen(0xFFFFF);
    //drawArrayJpeg(manual, sizeof(manual), 0, 0);
    CurrentPage = "manual";
  }
  
  if (PosX >=0 && PosX <=0 && PosY >=0 && PosY <=0 && CurrentPage == "menu") {
    tft.fillScreen(0x000000);
    //drawArrayJpeg(setting, sizeof(setting), 0, 0);
    CurrentPage = "wifi";

    tft.setCursor(200, 100, 4);
    tft.println(WiFi.localIP());
    Serial.print("Waiting for client.");
    
  while(1){
    Serial.print(".");
    WiFiClient client = server.available();   // listen for incoming clients

    if (client) {                             // if you get a client,
      Serial.println("New Client.");           // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          // Check to see if the client request was "GET /H" or "GET /L":
          if (c == 'H') {
            digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
          }
          if (c == 'L') {
            digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
          }
        }
      }
      }
      // close the connection:
      client.stop();
      Serial.println("Client Disconnected.");
      delay(10);
    }
  }

  if (PosX >=0 && PosX <=0 && PosY >=0 && PosY <=0 && CurrentPage == "menu") {
    tft.fillScreen(0xFFFFF);
    //drawArrayJpeg(setting, sizeof(setting), 0, 0);
    CurrentPage = "sensor";
  }
  ///////////////////////////////////////////////////////////////////////////////////
  if (CurrentPage == "menu") {
//    Serial.printf("x: %i     ", PosX);
//    Serial.printf("y: %i     ", PosY);
//    Serial.printf("z: %i \n", tft.getTouchRawZ());
  
    if (pressed) {
      tft.fillRect(0,0,80,50,0x000000);
      tft.setCursor(5, 5, 2);
      tft.printf("x: %i     ", PosX);
      tft.setCursor(5, 20, 2);
      tft.printf("y: %i    ", PosY);
   
      //tft.drawPixel(PosX, PosY, color); 320-Y //inverse y position=
      tft.fillEllipse(PosX, 320-PosY, 5, 5, color);
      color += 55;
    }
  }
}

//====================================================================
