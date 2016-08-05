/*
    CommProtocol.hpp

    Copyright (C) 2015 Italian Institute of Technology

    Developer: Luca Muratore  (luca.muratore@iit.it)
               Sabino Colonna (sabino.colonna@iit.it)
*/

#ifndef _COMM_PROTOCOL_HPP_
#define _COMM_PROTOCOL_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>

#include <exception>
#include <stdexcept>

#define HEADER_SIZE      3
#define MAX_PAYLOAD_SIZE 1024

#define TCP_COMMAND 0xFF
#define TCP_REPLY   0xFE
#define UDP_COMMAND 0xFF
#define UDP_REPLY   0xFE
#define UDP_BCAST   0xFD

#define PROGRAMMED_FW_VER_PCK_SIZE 28
#define PROGRAMMED_FW_VER_VERSION_SIZE (8+1) //string length INCLUDING the terminator character
#define PROGRAMMED_FW_VER_DATETIME_SIZE (20+1)//string length INCLUDING the terminator character


/**
 * @brief List of the possible TCP/UDP commands/replies to/from the firmware
 *
 */
enum Commands {

        //////////////////
        // TCP Commands //
        //////////////////

        GET_BOARD_TYPE,
        GET_FIRMWARE_VERSION,
        SET_FIRMWARE_VERSION,

        CLEAR_BOARD_FAULT,
        GET_BOARD_FAULT,

        SET_BCAST_RATE,
        GET_BCAST_RATE,
        SET_BCAST_POLICY,
        GET_BCAST_POLICY,

        CMD_UPGRADE,
        SAVE_PARAMS_TO_FLASH,
        LOAD_PARAMS_FROM_FLASH,
        LOAD_DEFAULT_PARAMS,

        CALIBRATE_OFFSETS,
        SET_BOARD_NUMBER,
        GET_CALIBRATION_OFFSETS,
        SET_RESOLUTION,
        GET_RESOLUTION,

        SET_TEMP_FACTORS,
        GET_TEMP_FACTORS,
        GET_CAL_TEMP,

        SET_CONVERSION_FACTORS,
        GET_CONVERSION_FACTORS,

        SET_AVERAGE_SAMPLES,
        GET_AVERAGE_SAMPLES,

        SET_MATRIX_ROW,
        GET_MATRIX_ROW,

        SET_ETHERNET_PORT,
        SET_IP_ADDR,
        SET_NET_MASK,
        GET_ETHERNET_PORT,
        GET_IP_ADDR,
        GET_NET_MASK,


        /////////////////
        // TCP Replies //
        /////////////////

        REPLY_BOARD_TYPE,
        REPLY_FIRMWARE_VERSION,

        REPLY_BOARD_FAULT,

        REPLY_BCAST_RATE,
        REPLY_BCAST_POLICY,

        REPLY_CALIBRATION_OFFSETS,
        REPLY_RESOLUTION,
        REPLY_TEMP_FACTORS,
        REPLY_CAL_TEMP,

        REPLY_CONVERSION_FACTORS,

        REPLY_AVERAGE_SAMPLES,

        REPLY_MATRIX_ROW,

        REPLY_ETHERNET_PORT,
        REPLY_IP_ADDR,
        REPLY_NET_MASK,


        //////////////////
        // UDP Commands //
        //////////////////

        CHECK_PROTOCOL,
        GET_ACTIVE_BOARDS,
        
        SET_SINGLE_UDP_PACKET_POLICY,
        GET_SINGLE_UDP_PACKET,
        UDP_CALIBRATE_OFFSETS,


        /////////////////
        // UDP Replies //
        /////////////////

        BCAST_DATA_PACKET_MT,

        REPLY_CHECK_PROTOCOL_MT,
        REPLY_ACTIVE_BOARDS,

        NUM_OF_COMMANDS
};

/**
 * @brief Command Info struct
 *
 */
typedef struct CmdInfo {
        uint8_t cmdType;
        uint8_t cmdId;
} CmdInfo;

/**
 * @brief Command Info table: NOTE Must match 'Commands' enum
 *
 */
const CmdInfo cmdsInfo[NUM_OF_COMMANDS] = {

        //////////////////
        // TCP Commands //
        //////////////////

        { TCP_COMMAND, 0x10 }, // GET_BOARD_TYPE
        { TCP_COMMAND, 0x11 }, // GET_FIRMWARE_VERSION
        { TCP_COMMAND, 0x51 }, // SET_FIRMWARE_VERSION

        { TCP_COMMAND, 0x52 }, // CLEAR_BOARD_FAULT
        { TCP_COMMAND, 0x12 }, // GET_BOARD_FAULT

        { TCP_COMMAND, 0x66 }, // SET_BCAST_RATE
        { TCP_COMMAND, 0x26 }, // GET_BCAST_RATE
        { TCP_COMMAND, 0x67 }, // SET_BCAST_POLICY
        { TCP_COMMAND, 0x27 }, // GET_BCAST_POLICY

        { TCP_COMMAND, 0x01 }, // CMD_UPGRADE
        { TCP_COMMAND, 0x2B }, // SAVE_PARAMS_TO_FLASH
        { TCP_COMMAND, 0x2C }, // LOAD_PARAMS_FROM_FLASH
        { TCP_COMMAND, 0x2D }, // LOAD_DEFAULT_PARAMS

        { TCP_COMMAND, 0x3B }, // CALIBRATE_OFFSETS
        { TCP_COMMAND, 0x88 }, // SET_BOARD_NUMBER
        { TCP_COMMAND, 0x40 }, // GET_CALIBRATION_OFFSETS
        { TCP_COMMAND, 0x81 }, // SET_RESOLUTION
        { TCP_COMMAND, 0x41 }, // GET_RESOLUTION

        { TCP_COMMAND, 0x82 }, // SET_TEMP_FACTORS
        { TCP_COMMAND, 0x42 }, // GET_TEMP_FACTORS
        { TCP_COMMAND, 0x48 }, // GET_CAL_TEMP

        { TCP_COMMAND, 0x83 }, // SET_CONVERSION_FACTORS
        { TCP_COMMAND, 0x43 }, // GET_CONVERSION_FACTORS

        { TCP_COMMAND, 0x30 }, // SET_AVERAGE_SAMPLES
        { TCP_COMMAND, 0x3C }, // GET_AVERAGE_SAMPLES

        { TCP_COMMAND, 0x31 }, // SET_MATRIX_ROW
        { TCP_COMMAND, 0x38 }, // GET_MATRIX_ROW

        { TCP_COMMAND, 0xD4 }, // SET_ETHERNET_PORT
        { TCP_COMMAND, 0xD6 }, // SET_IP_ADDR
        { TCP_COMMAND, 0xD7 }, // SET_NET_MASK
        { TCP_COMMAND, 0xDC }, // GET_ETHERNET_PORT
        { TCP_COMMAND, 0xDE }, // GET_IP_ADDR
        { TCP_COMMAND, 0xDF }, // GET_NET_MASK


        /////////////////
        // TCP Replies //
        /////////////////

        { TCP_REPLY, 0x90 }, // REPLY_BOARD_TYPE
        { TCP_REPLY, 0x91 }, // REPLY_FIRMWARE_VERSION

        { TCP_REPLY, 0x92 }, // REPLY_BOARD_FAULT

        { TCP_REPLY, 0xA6 }, // REPLY_BCAST_RATE
        { TCP_REPLY, 0xA7 }, // REPLY_BCAST_POLICY

        { TCP_REPLY, 0xC0 }, // REPLY_CALIBRATION_OFFSETS
        { TCP_REPLY, 0xC1 }, // REPLY_RESOLUTION
        { TCP_REPLY, 0xC2 }, // REPLY_TEMP_FACTORS
        { TCP_REPLY, 0xC8 }, // REPLY_CAL_TEMP

        { TCP_REPLY, 0xC3 }, // REPLY_CONVERSION_FACTORS

        { TCP_REPLY, 0xAC }, // REPLY_AVERAGE_SAMPLES

        { TCP_REPLY, 0xAB }, // REPLY_MATRIX_ROW

        { TCP_REPLY, 0xEC }, // REPLY_ETHERNET_PORT
        { TCP_REPLY, 0xEE }, // REPLY_IP_ADDR
        { TCP_REPLY, 0xEF }, // REPLY_NET_MASK


        //////////////////
        // UDP Commands //
        //////////////////

        { UDP_COMMAND, 0x01 }, // CHECK_PROTOCOL
        { UDP_COMMAND, 0x02 }, // GET_ACTIVE_BOARDS
        
        { UDP_COMMAND, 0x03 }, // SET_SINGLE_UDP_PACKET_POLICY
        { UDP_COMMAND, 0x04 }, // GET_SINGLE_UDP_PACKET
        { UDP_COMMAND, 0x05 }, // UDP_CALIBRATE_OFFSETS


        /////////////////
        // UDP Replies //
        /////////////////

        { UDP_BCAST, 0xBC }, // BCAST_DATA_PACKET_MT

        { UDP_REPLY, 0x83 }, // REPLY_CHECK_PROTOCOL_MT
        { UDP_REPLY, 0x82 }, // REPLY_ACTIVE_BOARDS
};


/**
 * @brief Communication Packet class: it handles the packet structure used to communicate with the FT17 board
 *
 */
class CommPacket
{
public:
        CommPacket ( int cmdId );
        uint8_t getPayloadSize();
        int appendData ( const uint8_t *data, int nBytes );
        int readData ( uint8_t *data, int nBytes );
        int getContent ( uint8_t *dst );

protected:
        static const uint8_t OFFSET_CMD_TYPE     = 0;
        static const uint8_t OFFSET_PAYLOAD_SIZE = sizeof ( OFFSET_CMD_TYPE );
        static const uint8_t OFFSET_CMD_ID       = OFFSET_PAYLOAD_SIZE + sizeof ( OFFSET_PAYLOAD_SIZE );

        uint8_t cmdType;
        uint8_t cmdId;
        uint8_t payloadSize;

        int readOffset;
        int writeOffset;

        uint8_t content[HEADER_SIZE+MAX_PAYLOAD_SIZE+1]; // 1 = checksum

        bool verifyHeader();
        void fillHeader();
        bool verifyChecksum();
        void setChecksum();
};

/**
 * @brief TCP comunication
 *
 */
class TCPCommPacket : public CommPacket
{
public:
        TCPCommPacket ( int cmdId );
        int sendToTCPSocket ( int socketId );
        int recvFromTCPSocket ( int socketId );
};

/**
 * @brief UDP communication
 *
 */
class UDPCommPacket : public CommPacket
{
public:
        UDPCommPacket ( int cmdId );
        int sendToUDPSocket ( int socketId, sockaddr *to, socklen_t toLen );
        int recvFromUDPSocket ( int socketId, sockaddr *from, socklen_t *fromLen );
};


#endif // _COMM_PROTOCOL_HPP_







