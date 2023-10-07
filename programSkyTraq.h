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

/**
 * @file programSkyTraq.h
 * @author Christian Pedersen
 * @date 30 Aug 2023
 * @brief Helper class to configure SkyTraq receivers
 *
 * This class configures the SkyTraq PX100 series receivers by sending binary
 * messages as defined in the SkyTraq Binary Protocol Application Note 39
 * version 1.4.42. https://navspark.mybigcommerce.com/content/AN0039.pdf
 * 
 * @see http://
 */

#ifndef PROGRAMSKYTRAQ_h
#define PROGRAMSKYTRAQ_h

#include <arduino.h>

class programSkyTraq
{
    public:
  
        /**
         * @brief Constructor
         *
         */
        programSkyTraq();
        
        /**
         * @brief Initialization routine for SkyTraq message reader
         *
         * This method initializes the SkyTraq receiver to send binary data
         * and initializes member variables.
         *
         * @param input_port Serial port to send/receive binary data to/from
         * the SkyTraq PX100 series reciever, e.g. Serial1
         *
         * @note Call this method from the Arduino sketch's setup() method
         */
        void init(Stream &input_port);
        
        /**
         * @brief Sends a message with the provided topic and payload
         *
         * This method sends the SkyTraq receiver the provided message with the
         * provided topic. 
         *
         * @param id Message ID can be a single byte, e.g. {0X22}, or two bytes for 
         * ID and sub ID, e.g. {0X69, 0X05}
         * @param payload_length Hex value for length of payload, constant per msg
         * type
         * @param msg_body Body of message in byte array
         *
         * @return 0: No valid response from receiver
         *         1: ACK received - success
         *         2: NACK recieved - failure
         *
         */
        int sendGenericMsg(uint8_t *msg_id,
                           int msg_id_length,
                           uint8_t *payload_length,
                           int payload_length_length,
                           uint8_t *msg_body,
                           int msg_body_length);
                           
        
    protected:
        
    private:
    
        /**
         * @brief Data port for serial connection to receiver, e.g. Serial1
         *
         */
        Stream *port;
        
        /**
         * @brief Communicates with SkyTraq to send message and monitor response
         *
         * This method sends input bytes directly to the receiver 
         *
         * @param msg_send Message to send
         * @param msg_size Size of message to send
         *
         */
        int sendMessage(uint8_t *msg_send, 
                        int msg_size);

    
        /**
         * @brief Calculates check sum value for a message
         *
         * @param msg_payload Message payload characters
         * @param payload_size Size of payload in the message
         *
         */
        uint8_t calculateCheckSum(uint8_t *msg_payload, int payload_size);

        
};

#endif



