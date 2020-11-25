#include <Ethernet.h>
#include <EthernetUdp.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

EthernetUDP Udp;

/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-light-sensor
 */

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // start the Ethernet connection:
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
  // print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  
  // start UDP
  Udp.begin(20000);
}

unsigned int lightStateLast = 0;
void loop() {
  // reads the input on analog pin A0 (value between 0 and 1023)
  int analogValue = analogRead(A0);
  IPAddress ip_logger(192, 168, 1, 25);
  unsigned int lightState = 0;

  Serial.print("Analog reading = ");
  Serial.println(analogValue);   // the raw analog reading


  if (analogValue < 10) {
    lightState = 0;
    if(lightStateLast != lightState) {
      Udp.beginPacket(ip_logger, 20000);
      Udp.write(" - Dark");
      Udp.endPacket();
    }
  } else if (analogValue < 200) {
    lightState = 1;
    if(lightStateLast != lightState) {
      Udp.beginPacket(ip_logger, 20000);
      Udp.write(" - Dim");
      Udp.endPacket();
    }
  } else if (analogValue < 500) {
    lightState = 2;
    if(lightStateLast != lightState) {
      Udp.beginPacket(ip_logger, 20000);
      Udp.write(" - Light");
      Udp.endPacket();
    }
  } else if (analogValue < 800) {
    lightState = 3;
    if(lightStateLast != lightState) {
      Udp.beginPacket(ip_logger, 20000);
      Udp.write(" - Bright");
      Udp.endPacket();
    }
  } else {
    lightState = 4;
    if(lightStateLast != lightState) {
      Udp.beginPacket(ip_logger, 20000);
      Udp.write(" - Very bright");
      Udp.endPacket();
    }
  }

  lightStateLast = lightState;

  switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      Serial.println("Error: renewed fail");
      break;

    case 2:
      //renewed success
      Serial.println("Renewed success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    case 3:
      //rebind fail
      Serial.println("Error: rebind fail");
      break;

    case 4:
      //rebind success
      Serial.println("Rebind success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    default:
      //nothing happened
      break;
  }

  delay(500);
}
