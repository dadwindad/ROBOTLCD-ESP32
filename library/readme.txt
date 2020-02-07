#define ILI9488_DRIVER
// WARNING: Do not connect ILI9488 display SDO to MISO if other devices share the SPI bus (TFT SDO does NOT tristate when CS is high)

#define TFT_MISO 19 	// do not connect - share SDO to TOUCH
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

//connect GPIO SDI(18),SOD(19) and TOUCH_CHIP SELECT(21)
#define TOUCH_CS 21 