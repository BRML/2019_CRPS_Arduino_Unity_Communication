#include "Wire.h"
#include "Adafruit_MLX90614.h"

//#include <Wire.h>
//#include <mlx90615.h>
Adafruit_MLX90614 mlx;
void setup()
{
Serial.begin(9600);
Serial.println("Melexis MLX90615 infra-red temperature sensor test");
mlx.begin();
Serial.print("Sensor ID number = ");
}
void loop()
{
Serial.print("Ambient = ");
Serial.print(mlx.readObjectTempC());
delay(500);
}
