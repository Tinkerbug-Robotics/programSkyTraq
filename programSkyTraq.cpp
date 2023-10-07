/**********************************************************************
*
* Copyright (c) 2023 Tinkerbug Robotics
*
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer. Redistributions in binary form must
reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the
distribution.

The software package includes some companion executive binaries or shared
libraries necessary to execute APs on Windows. These licenses succeed to the
original ones of these software.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/

#include "programSkyTraq.h"
#include "ArduinoLog.h"

// Constructor
programSkyTraq::programSkyTraq(){}


void programSkyTraq::init(Stream &input_port)
{
    
    port = &input_port;
    
    // Initialize with log level and log output. 
    Log.begin (LOG_LEVEL_NOTICE, &Serial, false);
}


int programSkyTraq::sendGenericMsg(uint8_t *msg_id,
                                   int msg_id_length,
                                   uint8_t *payload_length,
                                   int payload_length_length,
                                   uint8_t *msg_body,
                                   int msg_body_length)
{
    
    // Concatenate ID and body to make the payload
    uint8_t msg_payload[256];
    
    int j = 0;
    for (int i=0;i<msg_id_length;i++)
    {
        msg_payload[i] = msg_id[i];
        j++;
    }
    for (int i = 0;i < msg_body_length;i++)
    {
        msg_payload[j] = msg_body[i];
        j++;
    }
    
    // Use payload to calculate checksum value
    int payload_size = msg_id_length + msg_body_length;
    uint8_t cs_byte = calculateCheckSum(msg_payload,payload_size);
    
    // Build the whole message array
    uint8_t msg_whole[256];
    
    // Add fixed start sequence to start of whole message
    msg_whole[0] = 0xA0;
    msg_whole[1] = 0xA1;
    
    // Add payload length fields to whole message
    int k = 2;
    for (int i = 0; i < payload_length_length; i++)
    {
        msg_whole[k] = payload_length[i];
        k++;
    }
    
    // Add message payload (message ID plus message body) to whole message
    for (int i = 0; i < payload_size; i++)
    {
        msg_whole[k] = msg_payload[i];
        k++;
    }
    
    // Add checksum byte
    msg_whole[k] = cs_byte;
    
    // Add fixed end sequence
    msg_whole[k+1] = 0X0D;
    msg_whole[k+2] = 0X0A;
    
    // Send message
    int total_size = 2+payload_length_length+payload_size+1+2;
    return sendMessage(msg_whole,total_size);

}

int programSkyTraq::sendMessage(uint8_t *msg_send, 
                                int msg_size)
{
    port->write(msg_send,msg_size);
    Log.trace("sendMessage Sending ");
    for (int i=0;i<msg_size;i++)
    {
        Log.trace("%x ",msg_send[i]);
    }
    Log.trace("\n");
    
    // Time to wait for response (milliseconds)
    int wait_duration = 500;
    unsigned long time_start = millis();
    int input_pos = 0;
    
    Log.trace("sendMessage Receiving ");

    uint8_t response_msg[16];
    uint8_t last_byte_read = 0x00;
    bool read_response = false;
    int i_response = 0;
    
    // Wait for a response from the receiver
    while (millis() < time_start + wait_duration)
    {
        if (port->available())
        {
            // Read data from serial
            uint8_t byte_read = port->read();
            Log.trace("%x ", byte_read);
            
            // Detect the start sequence for a message
            if(byte_read == 0xA1 && last_byte_read == 0XA0)
            {
                read_response = true;
                byte_read = 0x00;
                last_byte_read = 0x00;
                i_response = 2;
            }
            
            // The fifth byte is the message ID for the response message
            if (read_response && i_response == 5)
            {
                if (byte_read == 0x83)
                {
                    Log.trace("\nReceived ACK from SkyTraq receiver\n");
                    return 1; 
                }
                else if (byte_read == 0x84)
                {
                    Log.error("\nReceived NACK from SkyTraq receiver request\n");
                    return 0;
                }
                read_response = false;
            }

            // Read and discard payload length bits
            if (read_response)
                i_response++;
            
            last_byte_read = byte_read;

        }
    }
    
    // Timeout
    Log.trace("\nsendMessage - No valid response from SkyTraq Receiver\n");
    return 2;
}

uint8_t programSkyTraq::calculateCheckSum(uint8_t *msg_payload, int payload_size) 
{
    // Sum up bits in payload to compute checksum value
    uint8_t checksum = 0;
    for (int i = 0; i < payload_size; i++) 
    {
        checksum ^= msg_payload[i];
    }
    
    return checksum;
}




