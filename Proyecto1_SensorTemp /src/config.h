//************************ Adafruit IO Config *******************************

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "mar21020"
#define IO_KEY       "aio_guns5837nZQfgpv8LUFfaAdDmsuN"

//******************************* WIFI **************************************

#define WIFI_SSID "Belle"
#define WIFI_PASS "belle1234"
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
