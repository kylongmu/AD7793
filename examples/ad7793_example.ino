/*
This is an example sketch for testing communication between the Arduino and the
AD7793 using the AD7793 library on the Analog Devices web site adapted for the Arduino
by Ph. Sonnet, April 8, 2019

The AD7793 and the Arduino Atmega328 R3 are connected in the following way : 

Arduino R3 pin#    AD7793 pin#     Pullup resistors
    8   GPIO      3   ~CS             50 KOhm
    9   RDY       15  DOUT/~RDY  
    12  MISO      15  DOUT/~RDY
    11  MOSI      16  DIN             50 KOhm
    10  CS        not connected   
    13  SCK       1   CLK             50 KOhm

The sketch reads the ID number of the AD7793, then uses the ADC to
measure the voltage on the analog voltage pin 13 and the temperature
inside the chip. These are the 3 functions that can be performed by the
barebone chip (not counting in the pullup resistors which must be present).  


*/
#include <AD7793.hpp>
#include <SPI.h>
//#include "USBSerial.h"
//#include "USB.h"
//USBCDC USBSerial;

unsigned long conv; /* The 24 bit output code resulting from a conversion by the ADC and read from the data register */

float Vref = 1.17; /* AD7783 internal reference voltage */ 
float GAIN; /* Gain of the AD7793 unternal instrumentation amplifier */
float AVDD;
float V; /* The voltage read on the analog input channel 2 (should be between -0.57 +0.57 when gain is set to 1) */
float RREF = 4990.0; /* The reference resistor: here, 4.99 Kohm, 0.1%, 10ppm/C */
float RRTD; /* The measured resistance of the RTD */ 
float Temp;

AD7793 ad7793;
SPIClass spi(FSPI);
void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  //delay(1000);
  //unsigned char answer = ad7793.Init();   /* Initializes the AD7793 and checks if the device is present*/
  unsigned char answer = ad7793.begin(SS,MISO,&spi);   /* Initializes the AD7793 and checks if the device is present*/
  //ad7793.Reset();
  //unsigned char answerl = ad7793.GetRegisterValue(AD7793_REG_ID, 1, 1);
  Serial.print("AD7793 status = ");
  Serial.println(answer); /* Answer is 1 when the device is initialized and the ID is read and recognized */ 
  Serial.println("");
  //USB.productName("ESP32S2-USB");
  //USB.begin();
  
  //USBSerial.begin(115200);
  //SerialUSB.begin(115200);
  AD7793_Config();
}

void loop() {
  unsigned char status;
  while(ad7793.GetRegisterValue(AD7793_REG_STAT, 1, 1)&AD7793_STAT_RDY){
    delay(1);
  }
  conv = ad7793.ContinuousSingleRead();
  //RRTD = RREF * (conv - 8388608.0) / (8388608.0 * GAIN); /* Computes the RTD resistance from the conversion code */
  //Serial.println(conv);  
  //if(conv&0x00800000)
    //conv|= 0xff000000;//MSB
  conv = medianFilter(conv);
  Serial.write((unsigned char*)&conv,4);
  //delay(50);
}
void AD7793_Config(){
  ad7793.SetChannel(AD7793_CH_AVDD_MONITOR);  /* AVDD Monitor, gain 1/6, internal 1.17V reference */
  unsigned long conv = ad7793.SingleConversion();  /* Returns the result of a single conversion. */                                          
  AVDD = ((conv - 8388608.0) / 8388608.0) * 1.17 / (1/6.0) ; /* Note: 8388608 = 2exp(23) = 0x8000000 = the output code of 0 V in bipolar mode */
  Serial.print("Analog supply voltage (AVDD) = ");
  Serial.print(AVDD, 4);
  Serial.println(" V");

  ad7793.SetChannel(AD7793_CH_TEMP); /* Temp Sensor, gain 1, Internal current reference */
  conv = ad7793.SingleConversion();  /* Returns the result of a single conversion. */
  Temp = (((conv - 8388608.0) / 8388608.0) * 1.17 * 1000 / 0.810) - 273;  /* Sentitivity is approximately 0.81 mV/Â°K, according to AD7793 datasheet */
                                                                                /* To improve precision, it should be further calibrated by the user. */
  Serial.print("Chip temperature = ");
  Serial.print(Temp, 2);
  Serial.println(" C");

    ad7793.SetChannel(AD7793_CH_AIN1P_AIN1M); /* Selects channel 1 of AD7793 */
    ad7793.SetGain(AD7793_GAIN_1); /* Sets the gain to 1 */
    GAIN = 1.0;
    //ad7793.EnableUnipolar();
    ad7793.SetClockSource(AD7793_CLK_INT);
    ad7793.SetIntReference(AD7793_REFSEL_INT); /* Sets the reference source for the ADC. */
    /* As the gain of the internal instrumentation amplifier has been changed, Analog Devices recommends performing a calibration  */
    ad7793.Calibrate(AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN1P_AIN1M); /* Performs Internal Zero calibration to the specified channel. */
    ad7793.Calibrate(AD7793_MODE_CAL_INT_FULL, AD7793_CH_AIN1P_AIN1M); /* Performs Internal Full Calibration to the specified channel. */
    //After calibrate, the device enter IDLE mode
    ad7793.SetMode(AD7793_MODE_CONT);  /* Continuous Conversion Mode */
    ad7793.SetFilterUpdateRate(2);//242Hz with SINC4 filter, This must be the final step.
   //conv = ad7793.SingleConversion(); 
    //delay(100);

}
inline void bubbleSort(unsigned long  data[], int size)
{
    unsigned long temp;
    while(size > 1)
    {
        for(int i = 0; i < size - 1; i++)
        {
            if(data[i] > data[i + 1])
            {
                temp = data[i];
                data[i] = data[i + 1];
                data[i + 1] = temp;
            }
        }
        size --;
    }
}
unsigned long  medianFilter(unsigned long  data)
{
  const unsigned char filter_size = 9;
  static unsigned long x[filter_size];
  unsigned long buble_data[filter_size];
  unsigned long median;
  unsigned char i;

  for(i=0;i<filter_size-1;i++){
    x[i] = x[i+1];
    buble_data[i] = x[i]; 
  }
  x[i]= data;
  buble_data[i] = x[i]; 

  bubbleSort(buble_data,filter_size);

  return (buble_data[filter_size/2+1]);
}
