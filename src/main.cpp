#include "Arduino.h"
#include "mongooseStart.h"

void setup() {
  delay(5000);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(50);

  ethernet_init();
  mongoose_init();
}

void loop() {
  mongoose_poll();
}
