// Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
// This is a human-readable summary of (and not a substitute for) the license.
// Disclaimer
//
// You are free to:
// Share — copy and redistribute the material in any medium or format
// Adapt — remix, transform, and build upon the material
// The licensor cannot revoke these freedoms as long as you follow the license terms.
//
// Under the following terms:
// Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
// NonCommercial — You may not use the material for commercial purposes.
// ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.
// No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
//
// Notices:
// You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation.
// No warranties are given. The license may not give you all of the permissions necessary for your intended use. For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
//
// github: https://github.com/gusgonnet/infraredReplicator
// hackster: https://www.hackster.io/gusgonnet
//
// Free for personal use.
//
// https://creativecommons.org/licenses/by-nc-sa/4.0/

// Tip for future version:
// I suggest you connect an electrolytic capacitor 10-20uF close to emitter of transistor and
//  anode of LED. The capacitor will quickly discharged to LED significantly increase power of
//   emitted IR pulse and 'save' other part from spikes on power rail.
// source: comment on https://www.hackster.io/BuddyC/wifi-ir-blaster-af6bca

#include "IRremote.h"

#define APP_NAME "infraredReplicator"
String VERSION = "Version 0.01";

/*******************************************************************************
 * changes in version 0.01:
       * Initial version
*******************************************************************************/

//enable the user code (our program below) to run in parallel with Particle's cloud connectivity code
// source: https://docs.particle.io/reference/firmware/photon/#system-thread
SYSTEM_THREAD(ENABLED);

#define IR_COMMAND_LENGTH 67
#define IR_CARRIER_FREQUENCY 38

/* IR commands and pulses
 - 3 means an 8500 microseconds pulse or more
 - 2 means a 4000 microseconds pulse
 - 1 means a 1500 microseconds pulse
 - 0 means a 500 microseconds pulse
See explanation below.
*/

/*

Setup
You need to build the circuit and connect the usb of the photon to your computer.
In Ubuntu Linux, I did this to monitor the console:

$ sudo chmod 666 /dev/ttyACM0
$ particle serial monitor

Depending on your computer and operating system, the port might change and the chmod might not need to be executed.
Once that is done, you can point the remote you want to decode to the IR receiver (the black LED-alike component).


Decode a command

Once you point your remote to the IR received and press a button, if all goes well you will be facing something like this:
1FEA05F
29168950,8500,4050,500,1500,550,500,550,450,550,500,500,1550,500,500,550,500,500,500,550,500,500,500,550,450,550,500,550,1500,500,500,550,500,500,500,550,500,500,550,500,1500,550,1500,550,450,550,1500,550,500,500,1500,550,500,550,450,550,500,500,500,550,END
3,3,2,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,END

We can discard the first value, 29168950 in this case, since the IR library states the following:
"the receive buffer starts with the duration of the gap space before the first mark".
We don't care about that duration, since is the time that passed by in between our tests, or you 
hitting a button in the remote.

So from this command:
29168950,8500,4050,500,1500,550,500,550,450,550,500,500,1550,500,500,550,500,500,500,550,500,500,500,550,450,550,500,550,1500,500,500,550,500,500,500,550,500,500,550,500,1500,550,1500,550,450,550,1500,550,500,500,1500,550,500,550,450,550,500,500,500,550,END

this is what is really interesting:
8500,4050,500,1500,550,500,550,450,550,500,500,1550,500,500,550,500,500,500,550,500,500,500,550,450,550,500,550,1500,500,500,550,500,500,500,550,500,500,550,500,1500,550,1500,550,450,550,1500,550,500,500,1500,550,500,550,450,550,500,500,500,550,END


We could now send those values with the following code:

unsigned int ircommand[59]={8550,4000,550,1500,550,450,550,500,500,550,500,1550,500,500,500,500,550,500,500,500,550,500,500,500,550,500,500,1500,550,500,500,500,550,500,500,500,550,450,550,1500,550,1500,550,500,500,1500,550,500,550,1500,500,500,550,500,500,500,550,450,550};
irsend.sendRaw(ircommand,59,38);


Making the command more readable
If we identify values around 500 with a 0 and 1500 with a 1, this selection being arbitrary, 
also assigning 3 to values over 5000 and 2 to values around 4000, we end up with the following representation:
3,2,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,END

Now remember that in this totally arbitrary encoding:
 - 3 means an 8500 microseconds pulse
 - 2 means a 4000 microseconds pulse
 - 1 means a 1500 microseconds pulse
 - 0 means a 500 microseconds pulse

Why encoding in numbers, you may ask? it helps visualising and comparing commands, I hope. This way, it becomes easier to 
compare pulses that change or remain the same among different commands.
Example: you want to compare what changes between a command to set the temperature to 18 and then 19 degreess.

*/

/*
from: http://www.righto.com/2009/08/multi-protocol-infrared-remote-library.html

The raw data for received IR measures the duration of successive spaces and marks in 50us ticks. 
The first measurement is the gap, the space before the transmission starts. The last measurement is the final mark.

The raw data for sending IR holds the duration of successive marks and spaces in microseconds. 
The first value is the first mark, and the last value is the last mark.

There are two differences between the raw buffers for sending and for receiving. The send buffer 
values are in microseconds, while the receive buffer values are in 50 microsecond ticks. 
The send buffer starts with the duration of the first mark, while the receive buffer starts with 
the duration of the gap space before the first mark.

from: http://www.maartendamen.com/2010/05/jeenode-infrared-project-part-1-getting-started/

source: http://www.sbprojects.com/knowledge/ir/
In serial communication we usually speak of 'marks' and 'spaces'. The 'space' is the default signal,
 which is the off state in the transmitter case. No light is emitted during the 'space' state. 
 During the 'mark' state of the signal the IR light is pulsed on and off at a particular frequency. 
 Carrier frequencies between 30kHz and 60kHz are commonly used in consumer electronics. 
 The most common one is 38kHz though. 
At the receiver side a 'space' is usually represented by a high level of the receiver's output. 
A 'mark' is then automatically represented by a low level.

Please note that the 'marks' and 'spaces' are not the 1-s and 0-s we want to transmit. 
The real relationship between the 'marks' and 'spaces' and the 1-s and 0-s depends on 
the protocol that's being used. More information about that can be found on the pages 
that describe the protocols.

transmit example from the Particle community:
https://community.particle.io/t/cash-bounty-for-irremote-irrecv-port-solved/23349/51?u=gusgonnet

*/

// source : https://github.com/babean/BeanSQ-IRremote/blob/master/examples/IRrecord/IRrecord.ino
// Storage for the recorded code
int codeType = -1;             // The type of code
unsigned long codeValue;       // The code value if not raw
unsigned int rawCodes[RAWBUF]; // The durations if raw
int codeLen;                   // The length of the code
int toggle = 0;                // The RC5/6 toggle state

int RECV_PIN = D6;
IRrecv irrecv(RECV_PIN);
decode_results results;

int TX_PIN = A5; //this is hardcoded in IRremoteInt.h line 488: #define TIMER_PWM_PIN        A5
IRsend irsend;

/*******************************************************************************
 * Function Name  : setup
 * Description    : this function runs once at system boot
 *******************************************************************************/
void setup()
{
  // publish startup message with firmware version
  Particle.publish(APP_NAME, VERSION);

  Serial.begin(9600);
  Serial.println("Hi there!");

  irrecv.enableIRIn(); // Start the receiver

  // send a samsung volume up/down code (good for testing your circuit)
  Particle.function("samsungVolUp", sendSamsungVolumeUp);
  Particle.function("samsungVolDn", sendSamsungVolumeDown);
}

/*******************************************************************************
 * Function Name  : loop
 * Description    : this function runs continuously while the project is running
 *******************************************************************************/
void loop()
{
  decodeIRcodes();
}

/*******************************************************************************
********************************************************************************
********************************************************************************
 INFRARED FUNCTIONS
********************************************************************************
********************************************************************************
*******************************************************************************/

/*******************************************************************************
 * Function Name  : convertToPulseDuration
 * Description    : this function sents the IR command to set the temperature of the device
                    Now remember that in this totally arbitrary encoding:
                     - 3 means an 8500 microseconds pulse
                     - 2 means a 4000 microseconds pulse
                     - 1 means a 1500 microseconds pulse
                     - 0 means a 500 microseconds pulse
 *******************************************************************************/
int convertToPulseDuration(unsigned int code)
{
  switch (code)
  {
  case 0:
    return 500;
    break;
  case 1:
    return 1500;
    break;
  case 2:
    return 4000;
    break;
  case 3:
    return 8500;
    break;
  }
  return -1;
}

// these are codes for a Samsung TV. I leave them here as an easy way to test your IR sending circuit
// (handy if you have a samsung TV around)
unsigned int SamsungVolumeUp[68] = {4600, 4350, 650, 1550, 700, 1500, 700, 1550, 700, 400, 700, 400, 700, 450, 650, 450, 700, 400, 700, 1500, 700, 1550, 650, 1550, 700, 400, 700, 400, 700, 450, 650, 450, 700, 400, 700, 1500, 700, 1550, 650, 1550, 700, 400, 700, 450, 700, 400, 700, 400, 700, 400, 700, 450, 650, 450, 650, 450, 650, 1550, 700, 1500, 700, 1550, 700, 1500, 700, 1550, 650};
unsigned int SamsungVolumeDown[68] = {4450, 4450, 550, 1650, 550, 1650, 550, 1650, 550, 550, 550, 600, 500, 600, 500, 600, 500, 600, 500, 1650, 600, 1600, 600, 1600, 600, 550, 550, 550, 550, 550, 550, 550, 550, 550, 550, 1650, 550, 1650, 550, 600, 500, 1650, 550, 600, 550, 550, 500, 600, 550, 550, 550, 550, 550, 550, 550, 1650, 550, 550, 550, 1650, 550, 1650, 550, 1650, 550, 1650, 550};

/*******************************************************************************
 * Function Name  : sendSamsungVolumeUp
 * Description    : send a samsung volume up code (good for testing your circuit)
 *******************************************************************************/
int sendSamsungVolumeUp(String dummy)
{
  irsend.sendRaw(SamsungVolumeUp, 68, 38);
  Particle.publish(APP_NAME, "Sent Samsung volume UP code", PRIVATE);
  return 0;
}

/*******************************************************************************
 * Function Name  : sendSamsungVolumeDown
 * Description    : send a samsung volume down code (good for testing your circuit)
 *******************************************************************************/
int sendSamsungVolumeDown(String dummy)
{
  irsend.sendRaw(SamsungVolumeDown, 68, 38);
  Particle.publish(APP_NAME, "Sent Samsung volume DOWN code", PRIVATE);
  return 0;
}

/*******************************************************************************
 * Function Name  : decodeIRcodes
 * Description    : this will print on the serial port the codes your remote is sending
                    enable this function in the loop function if you intend to use it
                    Now remember that in this totally arbitrary encoding:
                     - 3 means an 8500 microseconds pulse
                     - 2 means a 4000 microseconds pulse
                     - 1 means a 1500 microseconds pulse
                     - 0 means a 500 microseconds pulse
 *******************************************************************************/
void decodeIRcodes()
{

  if (irrecv.decode(&results))
  {
    unsigned long hash = decodeHash(&results);

    Serial.println(results.value, HEX);

    String decodif = "";
    for (int i = 0; i < results.rawlen; i++)
    {
      decodif = decodif + String(results.rawbuf[i] * 50) + ",";
    }
    decodif = decodif + "END";
    Serial.println(decodif);

    String decodif0123 = "";
    String tempChar = "";

    // Now remember that in this totally arbitrary encoding:
    //  - 3 means an 8500 microseconds pulse
    //  - 2 means a 4000 microseconds pulse
    //  - 1 means a 1500 microseconds pulse
    //  - 0 means a 500 microseconds pulse
    for (int i = 0; i < results.rawlen; i++)
    {
      if (results.rawbuf[i] * 50 > 5000)
      {
        tempChar = "3";
      }
      else if (results.rawbuf[i] * 50 > 2000)
      {
        tempChar = "2";
      }
      else if (results.rawbuf[i] * 50 > 1000)
      {
        tempChar = "1";
      }
      else
      {
        tempChar = "0";
      }
      decodif0123 = decodif0123 + tempChar + ",";
    }
    decodif0123 = decodif0123 + "END";
    Serial.println(decodif0123);

    Particle.publish(String(results.value), decodif + "|" + String(results.rawlen), PRIVATE);
    // storeCode(&results);
    irrecv.resume(); // Receive the next value
  }
}

#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

int compare(unsigned int oldval, unsigned int newval)
{
  if (newval < oldval * .8)
  {
    return 0;
  }
  else if (oldval < newval * .8)
  {
    return 2;
  }
  else
  {
    return 1;
  }
}
unsigned long decodeHash(decode_results *results)
{
  unsigned long hash = FNV_BASIS_32;
  for (int i = 1; i + 2 < results->rawlen; i++)
  {
    int value = compare(results->rawbuf[i], results->rawbuf[i + 2]);
    // Add value into the hash
    hash = (hash * FNV_PRIME_32) ^ value;
  }
  return hash;
}
