//-----------------------------------------------------------------------------
// This file is not part of OpenPLCv3.  The file is licensed as GPLv2.
// When you start Modbus TCP, Modbus RTU server is also started.
// Change the settings here if you don't want /dev/ttyUSB0 115200,E,8,1
// Copyleft 2020, miaoski.
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <modbus.h>     // libmodbus

#include "ladder.h"

#define MODBUS_RTU_SLAVE 0x01
#define BAUDRATE 9600 
#define SERIAL_BITS 8
#define PARITY 'E'
#define STOP_BITS 1
#define USB_DEV "/dev/ttyUSB0"

#define NET_BUFFER_SIZE 1024

//-----------------------------------------------------------------------------
// Function to start the server.
//-----------------------------------------------------------------------------
void modbusRTUStartServer()
{
    unsigned char log_msg[1000];
    int reqlen, msglen;
    modbus_t *mb_rtu;
    uint8_t buffer[NET_BUFFER_SIZE + 6];       // TCP header: tx ID, prot ID, length = 6 bytes
    unsigned char *p;
    unsigned char *eop = log_msg + sizeof(log_msg) - 2;    // end of log_msg w/o \n\0
    int i;

    sprintf(log_msg, "Initiating Modbus/RTU Server.\n");
    log(log_msg);
    
    mb_rtu = modbus_new_rtu(USB_DEV, BAUDRATE, PARITY, SERIAL_BITS, STOP_BITS);
    if(mb_rtu == NULL) {
            sprintf(log_msg, "RTU Server: Unable to create the libmodbus context\n");
            log(log_msg);
            return;
    }
    
    while(run_modbus_rtu)
    {
        modbus_set_slave(mb_rtu, MODBUS_RTU_SLAVE);
        if(modbus_connect(mb_rtu) == -1) {
            sprintf(log_msg, "RTU Server: RTU connection failed: %s\n", modbus_strerror(errno));
            log(log_msg);
            break;
        }
        reqlen = modbus_receive(mb_rtu, &buffer[6]);
        if(reqlen == -1) {
            sprintf(log_msg, "RTU Server: Error receiving Modbus/RTU: %s\n", modbus_strerror(errno));
            log(log_msg);
            break;
        }
        if(reqlen == 0) {
            // request to another RTU Slave.  Ignore it.
        } else {
            if(buffer[7] > 0x04) {          // We don't want to log readings
                sprintf(log_msg, "Recv (len=%d):", reqlen);
                for(i = 0, p = log_msg + strlen(log_msg); p < eop && i < reqlen; i++) {
                    sprintf(p, " %02x", buffer[i + 6]);
                    p += 3;
                }
                *p++ = '\n';
                *p++ = '\0';
                log(log_msg);
            }
            // padding to Modbus TCP and process
            buffer[0] = 0;              // Transaction ID
            buffer[1] = 0;
            buffer[2] = 0;              // Protocol ID
            buffer[3] = 0;
            reqlen = reqlen - 2;                        // -CRC
            buffer[4] = reqlen / 256;
            buffer[5] = reqlen % 256;
            msglen = processModbusMessage(buffer, reqlen + 6);     	// + TCP Header
            reqlen = modbus_send_raw_request(mb_rtu, &buffer[6], buffer[4] * 256 + buffer[5]);
            if(reqlen == -1) {
                sprintf(log_msg, "RTU Server: Error responding to Modbus/RTU: %s\n", modbus_strerror(errno));
                log(log_msg);
            }
        }
    }
    modbus_close(mb_rtu);
    modbus_free(mb_rtu);
    sprintf(log_msg, "Terminating Modbus/RTU Server thread\r\n");
    log(log_msg);
}
