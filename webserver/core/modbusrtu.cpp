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

//-----------------------------------------------------------------------------
// Function to start the server.
//-----------------------------------------------------------------------------
void modbusRTUStartServer()
{
    unsigned char log_msg[1000];
    int reqlen, msglen;
    bool *run_server = &run_modbus_rtu;
    modbus_t *mb_rtu;
    struct {
        uint16_t tx_id = 0;
        uint16_t prot_id = 0;
        uint8_t msglen_h = 0;
        uint8_t msglen_l = 0;
        uint8_t req[NET_BUFFER_SIZE];
    } msg_t;
    
    mb_rtu = modbus_new_rtu(USB_DEV, BAUDRATE, PARITY, SERIAL_BITS, STOP_BITS);
    if(mb_rtu == NULL) {
            sprintf(log_msg, "RTU Server: Unable to create the libmodbus context\n");
            log(log_msg);
            return;
    }
    
    while(*run_server)              // only 1 client for RTU
    {
        modbus_set_slave(mb_rtu, MODBUS_RTU_SLAVE);
        if(modbus_connect(mb_rtu) == -1) {
            sprintf(log_msg, "RTU Server: RTU connection failed: %s\n", modbus_strerror(errno));
            log(log_msg);
            break;
        }
        reqlen = modbus_receive(mb_rtu, &msg_t.req);
        sprintf(log_msg, "RTU Server: RTU connection failed: %s\n", modbus_strerror(errno));
        log(log_msg);
        if(reqlen == -1) {
            sprintf(log_msg, "RTU Server: Error receiving Modbus/RTU: %s\n", modbus_strerror(errno));
            log(log_msg);
            break;
        }
        if(reqlen == 0) {
            // request to another RTU Slave.  Ignore it.
        } else {
            sprintf(log_msg, "Recv (len=%d): %02x %02x %02x %02x %02x %02x\n", reqlen, 
                    msg_t.req[0], msg_t.req[1], msg_t.req[2], msg_t.req[3], msg_t.req[4], msg_t.req[5]);
            log(log_msg);
            // padding to Modbus TCP and process
            reqlen = reqlen - 2;                                        // -CRC
            msg_t.msglen_h = reqlen / 256;
            msg_t.msglen_l = reqlen % 256;
            msglen = processModbusMessage(&msg_t, reqlen + 6 - 2);     // + TCP Header - CRC
            reqlen = modbus_send_raw_request(mb_rtu, &msg_t.req, msg_t.msglen_h * 256 + msg_t.msglen_l);
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
