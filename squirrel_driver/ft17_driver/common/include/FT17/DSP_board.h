/*
    DSP_board.h

    Copyright (C) 2015 Italian Institute of Technology

    Developer: Luca Muratore  (luca.muratore@iit.it)
               Alessio Margan (alessio.margan@iit.it)

*/

#ifndef __DSP_BOARDS_H__
#define __DSP_BOARDS_H__

#include <netinet/in.h>
#include <arpa/inet.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/error_of.hpp>
#include <boost/accumulators/statistics/error_of_mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/circular_buffer.hpp>

#include <FT17/definitions.h>
#include <iostream>

//------------------------------------------------

//------------------------------------------------
using namespace boost::accumulators;

/**
 * @class Dsp_Board
 * @defgroup Dsp_Board Dsp board
 * @ingroup FT17_driver
 * @brief Dsp_Board base class to interface with DSP boards,
 *        each single instance handle TCP/IP communication with
 *        the relative board and process its broadcast data
 *        dispatched by Boards_ctrl class
 */
class Dsp_Board
{
        typedef accumulator_set<double,
                features<
                tag::count,
                tag::mean,
                tag::min,
                tag::max,
                tag::variance ( lazy ),
                droppable<tag::error_of<tag::mean> >
                >
                > accum_t;

public:
        Dsp_Board ( uint8_t  *, int );
        virtual ~Dsp_Board();

        /**
        * @brief send a set command
        *
        *
        * @param reqCmd
        * @param src
        * @param srcBytes
        *
        * @return int
        */
        int setItem ( int reqCmd, void *src, int srcBytes );

        /**
        * @brief send a request command and wait for reply
        *
        * @param reqCmd
        * @param src
        * @param srcBytes
        * @param resCmd
        * @param dst
        * @param dstBytes
        *
        * @return int
        */
        int getItem ( int reqCmd, void *src, int nSrcBytes,
                      int resCmd, void *dst, int nDstBytes );

        virtual void configure_streaming ( uint8_t bc_rate, uint16_t policy ) = 0;
        virtual void configure_polling ( uint16_t policy ) = 0;

        virtual void on_bc_data ( uint8_t * );
        virtual void get_bc_data ( ts_bc_data_t & );
        
        virtual void get_single_data ( ts_single_data_t & );
        
        virtual void print_me ( void );
        virtual void check_bc_data_rx ( void );

        /**
        * @brief start/stop broadcast, set bc_rate via TCP packet
        *
        * @param start_stop    true/false;
        *
        */
        void start_stop_bc ( uint8_t );

        uint8_t     stopped;
        uint8_t     bId;
        uint8_t     bType;

protected:

        virtual uint32_t get_bc_data_size() {
                return 0;
        }

        char        ip_addr[16];
        //
        int         sock_fd;
        sockaddr_in sock_addr;
        //
        uint16_t    policy;
        uint16_t    extra_policy;
        uint8_t     bc_rate;
        //
        ts_bc_data_t    ts_bc_data;
        //
        int udp_sock;
        //
        operational_mode mode;

private:

         /**
         * @brief fill the FT data based on the policy setted.
         *
         * @param raw_bc_buff raw broadcast data
         * @return void
         */
        void fill_ft_data ( uint8_t *raw_bc_buff );

        /**
        * @brief write log file in /tmp/log_bId_<...>.txt, see LOG_SIZE define
        *        for circular buffer dimension
        *
        */
        void dump_log ( void );
        void measure_bc_freq ( void );
        void print_stat ( void );

        uint64_t _bc_tStart;
        uint64_t _rx_bc_prec;
        accum_t bc_freq, tmp_bc_freq;

        pthread_mutex_t dsp_data_mutex;
        pthread_mutex_t dsp_sock_mutex;

        boost::circular_buffer<ts_bc_data_t> dsp_log;

};


/**
 * @class FtBoard
 * @defgroup FtBoard Force torque sensor board
 * @ingroup Dsp_Board
 * @ingroup Dsp_Board
 * @brief FtBoard class derived from Dsp_Board, implement
 *        derived virtual method to carry out force torque
 *        sensor protocol
 */
class FtBoard : public Dsp_Board
{
public:
        FtBoard ( uint8_t *_, int udp_sock ) : Dsp_Board ( _, udp_sock ) { }
        
        bool calibrate_offsets();

protected:
        virtual void configure_streaming ( uint8_t bc_rate, uint16_t policy );
        virtual void configure_polling ( uint16_t policy );
        virtual void print_me ( void );
        virtual uint32_t get_bc_data_size() {
                return sizeof ( ft_bc_data_t ); //TBD do it checkingthe policy
        }


};

#endif
