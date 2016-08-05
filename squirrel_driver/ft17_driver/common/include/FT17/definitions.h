/*
	defienitions.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <FT17/Broadcast_data.hpp>


#define MAX_FT_BOARDS    6
#define MAX_DSP_BOARDS   (MAX_FT_BOARDS)

#define BCAST_FT_DATA_PACKETS 0xBC // BCAST_DATA_PACKETS ForceTorqueSens

#define _1_SEC          1000
// 10 mins at 1Khz loop
//#define LOG_SIZE    _1_SEC * 60 * 10
// 1 min secs at 1Khz loop
#define LOG_SIZE    _1_SEC * 60

/**
 * @defgroup DataStructures Data Structures
 * @ingroup FT17_driver
 * @brief Here are the data structures with brief descriptions:
 *
 * @{
 */

/**
 */
typedef struct {
        uint64_t    ts;
        uint16_t    ft_bc_policy;
} info_data_t;

/**
 * common udp header tx by DSP boards NOTE unused
 */
typedef struct {
        unsigned char _header;
        unsigned char _n_bytes;
        unsigned char _command;
        unsigned char _board_id;
} bc_header_t;

/**
 * convenient way to access broadcast data
 */
typedef	union {
        bc_header_t     bc_header;   // NOTE unused header embedded inside ft_bc_data_t
        ft_bc_data_t    ft_bc_data;
} bc_data_t;

/**
 * timestamped DSP broadcast data
 */
typedef struct {
        uint64_t    ts_rx;
        bc_data_t   raw_bc_data;
} ts_bc_data_t;

/**
 * timestamped DSP single data
 */
typedef struct {
        uint64_t    ts_rx;
        bc_data_t   raw_bc_data;
} ts_single_data_t;

/**
 * @brief List of the possible TCP/UDP commands/replies to/from the firmware
 *
 */
typedef enum {
    STREAMING,
    POLLING
} operational_mode;

#if __XENO__
static const std::string pipe_prefix ( "/proc/xenomai/registry/rtipc/xddp/" );
#else
static const std::string pipe_prefix ( "/tmp/" );
#endif

/** @}
 */

#endif


