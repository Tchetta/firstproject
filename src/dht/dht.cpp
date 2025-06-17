#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp

DHTesp dht;

void dhtSetup(int dhtPin) {
  dht.setup(dhtPin, DHTesp::DHT11); // Connect DHT sensor to GPIO 17
}

float getHumidity() {
    delay(dht.getMinimumSamplingPeriod());

    float humidity = dht.getHumidity();

    return humidity;
}

float getTemperatureDEG() {
    delay(dht.getMinimumSamplingPeriod());

    float temperature = dht.getTemperature();

    return temperature;
}