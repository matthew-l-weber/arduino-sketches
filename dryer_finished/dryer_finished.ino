
#include "Wire.h"
#include <MPU6050_light.h>

#include <Ethernet.h>
#include <EthernetUdp.h>

//#define INLINE_FFT 1
#define ARDUINOLIB_FFT 1

#define DEBUG 1

IPAddress logging_server_ip(192, 168, 1, 25);
unsigned int logging_server_port = 20000;

EthernetUDP Udp;
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
uint8_t mac[] = {
  0x90, 0xA2, 0xDA, 0x00, 0x30, 0xB9
};


void logNet(char * entry) {
  Udp.beginPacket(logging_server_ip, logging_server_port);
  Udp.write(entry);
  Udp.endPacket();
}

#ifdef INLINE_FFT
//---------------------------------------------------------------------------//
const byte sine_data [91]=
 {
0,  
4,    9,    13,   18,   22,   27,   31,   35,   40,   44, 
49,   53,   57,   62,   66,   70,   75,   79,   83,   87, 
91,   96,   100,  104,  108,  112,  116,  120,  124,  127,  
131,  135,  139,  143,  146,  150,  153,  157,  160,  164,  
167,  171,  174,  177,  180,  183,  186,  189,  192,  195,       //Paste this at top of program
198,  201,  204,  206,  209,  211,  214,  216,  219,  221,  
223,  225,  227,  229,  231,  233,  235,  236,  238,  240,  
241,  243,  244,  245,  246,  247,  248,  249,  250,  251,  
252,  253,  253,  254,  254,  254,  255,  255,  255,  255
  };
float f_peaks[5]; // top 5 frequencies peaks in descending order
//---------------------------------------------------------------------------//

//-----------------------------FFT Function----------------------------------------------//

float FFT(int in[],byte N,float Frequency)
{
/*
Code to perform FFT on arduino,
setup:
paste sine_data [91] at top of program [global variable], paste FFT function at end of program
Term:
1. in[]     : Data array, 
2. N        : Number of sample (recommended sample size 2,4,8,16,32,64,128...)
3. Frequency: sampling frequency required as input (Hz)

If sample size is not in power of 2 it will be clipped to lower side of number. 
i.e, for 150 number of samples, code will consider first 128 sample, remaining sample  will be omitted.
For Arduino nano, FFT of more than 128 sample not possible due to mamory limitation (64 recomended)
For higher Number of sample may arise Mamory related issue,
Code by ABHILASH
Contact: abhilashpatel121@gmail.com 
Documentation:https://www.instructables.com/member/abhilash_patel/instructables/
*/

const unsigned int data[13]={1,2,4,8,16,32,64,128,256,512,1024,2048};
int a,c1,f,o,x;
a=N;  
                                 
      for(int i=0;i<12;i++)                 //calculating the levels
         { if(data[i]<=a){o=i;} }
      
int in_ps[data[o]]={};     //input for sequencing
float out_r[data[o]]={};   //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
           
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
         {
          c1=data[b];
          f=data[o]/(c1+c1);
                for(int j=0;j<c1;j++)
                    { 
                     x=x+1;
                     in_ps[x]=in_ps[j]+f;
                    }
         }

 
      for(int i=0;i<data[o];i++)            // update input array as per bit reverse order
         {
          if(in_ps[i]<a)
          {out_r[i]=in[in_ps[i]];}
          if(in_ps[i]>a)
          {out_r[i]=in[in_ps[i]-a];}      
         }
         


int i10,i11,n1;
float e,c,s,tr,ti;

    for(int i=0;i<o;i++)                                    //fft
    {
     i10=data[i];              // overall values of sine/cosine  :
     i11=data[o]/data[i+1];    // loop with similar sine cosine:
     e=360/data[i+1];
     e=0-e;
     n1=0;

          for(int j=0;j<i10;j++)
          {
          c=cosine(e*j);
          s=sine(e*j);    
          n1=j;
          
                for(int k=0;k<i11;k++)
                 {
                 tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                 ti=s*out_r[i10+n1]+c*out_im[i10+n1];
          
                 out_r[n1+i10]=out_r[n1]-tr;
                 out_r[n1]=out_r[n1]+tr;
          
                 out_im[n1+i10]=out_im[n1]-ti;
                 out_im[n1]=out_im[n1]+ti;          
          
                 n1=n1+i10+i10;
                  }       
             }
     }

/*
for(int i=0;i<data[o];i++)
{
Serial.print(out_r[i]);
Serial.print("\t");                                     // un comment to print RAW o/p    
Serial.print(out_im[i]); Serial.println("i");      
}
*/


//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
    for(int i=0;i<data[o-1];i++)               // getting amplitude from compex number
        {
         out_r[i]=sqrt(out_r[i]*out_r[i]+out_im[i]*out_im[i]); // to  increase the speed delete sqrt
         out_im[i]=i*Frequency/N;
         /*
         Serial.print(out_im[i]); Serial.print("Hz");
         Serial.print("\t");                            // un comment to print freuency bin    
         Serial.println(out_r[i]); 
         */    
        }




x=0;       // peak detection
   for(int i=1;i<data[o-1]-1;i++)
      {
      if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
      {in_ps[x]=i;    //in_ps array used for storage of peak number
      x=x+1;}    
      }


s=0;
c=0;
    for(int i=0;i<x;i++)             // re arraange as per magnitude
    {
        for(int j=c;j<x;j++)
        {
            if(out_r[in_ps[i]]<out_r[in_ps[j]]) 
                {s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;}
        }
    c=c+1;
    }



    for(int i=0;i<5;i++)     // updating f_peak array (global variable)with descending order
    {
    f_peaks[i]=out_im[in_ps[i]];
    }



}
    

float sine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[j];}
  else if(j>90  && j<181){out= sine_data[180-j];}
  else if(j>180 && j<271){out= -sine_data[j-180];}
  else if(j>270 && j<361){out= -sine_data[360-j];}
  return (out/255);
}

float cosine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[90-j];}
  else if(j>90  && j<181){out= -sine_data[j-90];}
  else if(j>180 && j<271){out= -sine_data[270-j];}
  else if(j>270 && j<361){out= sine_data[j-270];}
  return (out/255);
}

//------------------------------------------------------------------------------------//

#endif


void initEthernet() {
  Serial.println(F("Ethernet: Starting..."));
  Serial.print(F("Initialize Ethernet with DHCP: "));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. :("));
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println(F("Ethernet cable is not connected."));
    }
    while (true) {
      delay(1);
    }
  }
  Serial.println(Ethernet.localIP());
  
  // start UDP
  Udp.begin(20000);
  
  Serial.println(F("Ethernet: Init Complete."));
}

void updateEthernet() {
  switch (Ethernet.maintain()) {
    case 1:
      Serial.println(F("Ethernet: Error renewed fail"));
      break;

    case 2:
      Serial.println(F("Ethernet: Renewed success"));
      Serial.print(F("My IP address: "));
      Serial.println(Ethernet.localIP());
      break;

    case 3:
      Serial.println(F("Ethernet: Error rebind fail"));
      break;

    case 4:
      Serial.println(F("Ethernet: Rebind success"));
      Serial.print(F("My IP address: "));
      Serial.println(Ethernet.localIP());
      break;

    default:
      //nothing happened
      break;
  }

  
}

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;
unsigned int lightStateLast = 0;

void initLightSensor() {
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  Serial.println(F("LightSensor: Init Complete."));
}

void updateLightSensor() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  if(DEBUG)
    Serial.println(average);
}





MPU6050 mpu(Wire);
void initVibrationSensor() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("VibrationSensor: status -> "));
  Serial.println(status);
  if (status != 0) {
    Serial.println(F("Failed to configure VibrationSensor, halting"));
    while (true) {
      delay(1);
    }
  }
  Serial.println(F("Calculating offsets, do not move sensor array"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero TBD may not need to setup gyros.....?????????????
  Serial.println(F("VibrationSensor: Init Complete."));
}



#define SAMPLES 64
#define EXPANDED_SAMPLES 64000000
double vReal[SAMPLES];

#ifdef ARDUINOLIB_FFT
#include "arduinoFFT.h"
arduinoFFT FFT = arduinoFFT();
double vImag[SAMPLES];
#endif

void updateVibrationSensor() {
  unsigned long t = micros();
  for(int i = 0; i < SAMPLES; i++)
  {
    mpu.update();
    vReal[i]=mpu.getAccX();
    //vReal[i]=mpu.getAccY();
    //vReal[i]=mpu.getAccZ();
#ifdef ARDUINOLIB_FFT
    vImag[i] = 0;
#endif
    delayMicroseconds(1);
  }

  // Getting the duration
  t = micros() - t;
  t = EXPANDED_SAMPLES / t;

#ifdef ARDUINOLIB_FFT
  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, t);

  for(int i=0; i<(SAMPLES/2); i++)
  {
      /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
       
      //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
      //Serial.print(" ");
      Serial.print(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
      Serial.print(F(","));
  }
 
  Serial.print(F("SampleFreq: "));
  Serial.print(t);
  Serial.print(F(" Peak: "));
  Serial.println(peak);  
#endif


#ifdef INLINE_FFT  
  FFT(vReal, SAMPLES, t);

  if(DEBUG) {
    Serial.print(f_peaks[0]);
    Serial.print(",");
    Serial.print(f_peaks[1]);
    Serial.print(",");
    Serial.print(f_peaks[2]);
    Serial.print(",");
    Serial.print(f_peaks[3]);
    Serial.print(",");
    Serial.println(f_peaks[4]); 
  }   
#endif

  Serial.print(F("Temp: "));Serial.println(mpu.getTemp());
//    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
//    Serial.print("\tY: ");Serial.print(mpu.getAccY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
//    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
//    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
//    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
//    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
//    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
//    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
//    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
//    timer = millis();
//  }
}





void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  initLightSensor();
  
  initVibrationSensor();

  initEthernet();
}

#define LIGHT_SENSOR_PERIOD 2000000
#define ETHERNET_PERIOD 5000000
unsigned long ticks, runLightTicks=0, runEthTicks=0;
void loop() {
  ticks = micros();

  // continuous aquire vibration data
  // TBD need to measure what this is
  // FFT compute on 64samples to start
  updateVibrationSensor();


  if(ticks - runLightTicks > LIGHT_SENSOR_PERIOD) {
    updateLightSensor();  
    runLightTicks = ticks;
  }

  if(ticks - runEthTicks > ETHERNET_PERIOD) {
    updateEthernet();
    runEthTicks = ticks;
  }
}
