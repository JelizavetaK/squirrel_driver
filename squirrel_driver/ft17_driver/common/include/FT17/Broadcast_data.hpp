/*
    Broadcast_data.hpp

    Copyright: (C) 2015 Italian Institute of Technology

    Developer: Luca Muratore  (luca.muratore@iit.it)
               Alessio Margan (alessio.margan@iit.it)

*/

#ifndef __BROADCAST_DATA_HPP__
#define __BROADCAST_DATA_HPP__

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <map>
#include <string>

typedef std::map<std::string, int> bc_data_map_t;

typedef struct {

        unsigned char _header;
        unsigned char _n_bytes;
        unsigned char _command;
        unsigned char _board_id;

        int FT[6];
        int ModFT[6];
        unsigned short ch[6];
        long temp_Vdc;
        long tStamp;
        int fault;
        int ModFTFiltered[6];

        unsigned char _chk;

} __attribute__ ( ( __packed__ ) ) ft_bc_data_t;

#endif
