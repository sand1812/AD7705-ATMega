#include <Arduino.h>
#include <AD7705ADC.h>

AD7705ADC *adc1;

double v1;
double v2;
long l1;
long l2;
int i = 0;



// Pinout for Arduino Nano :
// D13 = SCK = CS
// D12 = MISO = DOUT
// D11 = MOSI = DN

#define POLARITY_BIPOLAR

#ifdef POLARITY_UNIPOLAR
#define POLARITY_VAL AD7705ADC::UNIPOLAR
#else
#define POLARITY_VAL AD7705ADC::BIPOLAR
#endif

#define PIN_SS 10 // Slave Select = CS
#define PIN_DR 2  // Data Ready

void setup() {

  Serial.begin(115200);
  Serial.println("Begin setup");
  adc1 = new AD7705ADC(PIN_SS,PIN_DR);
  Serial.println("Instance created");

  adc1->reset();
  Serial.println("Reset Done");

  adc1->setClockRegister(AD7705ADC::CLOCK_4_9152,0);
  Serial.println("Set clock register");

  adc1->setSetupRegister(
    AD7705ADC::CHANNEL_0,                    //  Channel
    AD7705ADC::AD7705ADC::MODE_SELF_CAL,    //      CAL mode
    AD7705ADC::GAIN_1,                                          //  Gain = 1
    POLARITY_VAL,                     //      Polarity
    false,                                                              //  Not buffered
    false); //  F Sync off
  Serial.println("Setup register");

  adc1->reset();
  Serial.println("Reset done");

  adc1->setSetupRegister(
    AD7705ADC::CHANNEL_1,                    //  Channel
    AD7705ADC::AD7705ADC::MODE_SELF_CAL,    //      CAL mode
    AD7705ADC::GAIN_1,                                          //  Gain = 1
    POLARITY_VAL,                     //      Polarity
    false,                                                              //  Not buffered
    false); //  F Sync off
  Serial.println("Setup register chan_1");

}

void loop(){
  float f;
  uint16_t value;
  Serial.print("Ch1 : ");
  adc1->reset();
  value = adc1->getValue(AD7705ADC::CHANNEL_0);
  Serial.print(value);
  Serial.print(" => ");
  Serial.print(value-32768);
  Serial.print(" = ");
#ifdef POLARITY_BIPOLAR
  f = 10.0*float(value-32768)/65536.0;
#else
  f = 5.0*float(value)/65536.0;
#endif
  Serial.println(f);

  Serial.print("Ch2 : ");
  adc1->reset();
  value = adc1->getValue(AD7705ADC::CHANNEL_1);
  Serial.print(value);
  Serial.print(" => ");
  Serial.print(value-32768);
  Serial.print(" = ");
  #ifdef POLARITY_BIPOLAR
    f = 10.0*float(value-32768)/65536.0;
  #else
    f = 5.0*float(value)/65536.0;
  #endif
  Serial.println(f);
  Serial.println("");


  delay(500);
}
