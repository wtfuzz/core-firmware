#include "application.h"

void setup()
{
  Serial.begin(115200);
}

/* This function loops forever --------------------------------------------*/
void loop()
{
  Serial.println(millis());
}
