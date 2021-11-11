#include <Oregon_NR.h>
#include <Oregon_TM.h>
---------------------------------------------------------------------------------------------------------------------------------------------------
//This example describes a packet relay
//May be useful for increasing the range of signal reception from some sensors
---------------------------------------------------------------------------------------------------------------------------------------------------
#define LED 13             //LED output
#define MAX_SEND_BUFFER 30 //maximum size of the transmit buffer in nibls
Oregon_NR oregon(2, 0, 255, true, MAX_SEND_BUFFER, true);   //Receiver on D2, interrupt 0, no LED needed, 30 nib buffers, packet build on
Oregon_TM transmitter(4, MAX_SEND_BUFFER);   //D4 transmitter, 30 nibs buffer

---------------------------------------------------------------------------------------------------------------------------------------------------
void(* resetFunc) (void) = 0;

void setup() {
   Serial.begin(115200);
   oregon.start();        //Turn on the receiver
   pinMode(LED, OUTPUT);
   digitalWrite(LED,LOW);
   Serial.println("START");
}

---------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  
  oregon.capture(0);
  if (oregon.captured)  
  {
   if (micros() > 3600000000) resetFunc();
    transmitter.buffer_size = 0;
    for (int q = 0; q < oregon.packet_length; q++) 
    {
      if (oregon.valid_p[q] != 0x00)
      {
        transmitter.buffer_size++;
        Serial.print(oregon.packet[q], HEX);
      }
      else break;
    }
    //Checking if this packet needs to be relayed
    if ((oregon.packet[0] == 0x0E && oregon.packet[1] == 0x0C)
    //For example, we need to retransmit a packet if the first two nibbles of the packet are 19h (WGR800 sensor)
     || (oregon.packet[0] == 0x01 && oregon.packet[1] == 0x09)
     || (oregon.packet[0] == 0x05 && oregon.packet[1] == 0x05)
     || (oregon.packet[0] == 0x01 && oregon.packet[1] == 0x0D))
    
    {
      Serial.println("\t -> ");
      delay(500); 

        
      //Preparing the transfer buffer
      //Add a flag to the tail that this is a relayed packet
      oregon.packet[transmitter.buffer_size] = 0x00;
      transmitter.buffer_size ++;
      oregon.packet[transmitter.buffer_size] = 0x00;
      transmitter.buffer_size ++;
      //rewrite the buffer to the transmitter
      if (transmitter.buffer_size > MAX_SEND_BUFFER) transmitter.buffer_size = MAX_SEND_BUFFER;
      //We rewrite the buffer from the receiver to the transmitter
      for ( int q = 0; q < transmitter.max_buffer_size; q++)
        transmitter.SendBuffer[q] = oregon.packet[q*2+1] + oregon.packet[q*2]*16;        

      //We will transmit in the same protocol as we received
      transmitter.protocol = oregon.ver;
        
      //We transfer data
      digitalWrite(13,HIGH);
      transmitter.SendPacket();
      digitalWrite(13,LOW);
    }
    else Serial.println(' ');
  }
}

