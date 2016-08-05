/*
   Boards_iface.h

   Copyright (C) 2015 Italian Institute of Technology

   Developer: Luca Muratore (luca.muratore@iit.it)
              Alessio Margan (alessio.margan@iit.it)

*/


/**
 * @defgroup FT17_driver FT17_driver
 *
 * @brief Real-time library for Low-Level Interaction with FT17 sensors.
 *
 * %FT17_driver real-time library that allows a GNU/Linux/Xenomai/RTnet PC to interact
 * with FT17 sensors by means of an Ethernet cable.
 * All you need on your PC is an RTnet-compliant network adapter, i.e. supported for hard real-time communication:
 * for a list of all supported adapters, please see <a href="http://rtnet.org/"> RTnet homepage </a>.
 * Each sensor is controlled by means of a DSP board.
 *
 * %FT17_driver handles two kinds of connections:
 * @li UDP, established to send/receive broadcast data to/from all active sensors (i.e. all powered on sensors);
 * @li TCP, established with each active sensor.
 *
 * %The protocol is implemented by Boards_ctrl class, DSP_board
 * class and its derived classes FtBoard
 * @li instance of Boards_ctrl class handle UDP braodcast
 *     communication and record reference of "ALIVE" boards that
 *     at start up time reply to the controller
 * @li instance of DSP_Board class handle TCP communication with
 *     the associated sensor

 * %FT17_driver library should be used according to the following steps:
 * @li instantiate the Boards_ctrl class;
 * @li call Boards_ctrl::init() method;
 * @li call Boards_ctrl::scan4active() method;
 * @li call Boards_ctrl::configure_boards();
 * @li call Boards_ctrl::start_stop_control() true to start the
 *     dsp controller;
 * @li call Boards_ctrl::start_stop_bc_boards() true to start
 *     broadcast data transmission from boards;
 * @li on shutdown call :
 *     - Boards_ctrl::start_stop_control() false to stop
 *       controller;
 *     - Boards_ctrl::stop_rx_udp();
 *     - Boards_ctrl::start_stop_bc_boards() false to stop
 *       broadcast data transmission;
 *
 * Before starting your own applications making use of %FT17_driver library, you must load all the necessary Xenomai modules
 * and configure RTnet network which PC and remote boards belong to .
 *
 * @author Luca Muratore (luca.muratore@iit.it)
 *         Alessio Margan (alessio.margan@iit.it)
*/

#ifndef __BOARDS_IFACE_H__
#define __BOARDS_IFACE_H__

#include <netinet/in.h>
#include <arpa/inet.h>
#include <bitset>
#include <pthread.h>
#include <vector>
#include <map>

#include <FT17/definitions.h>
#include <FT17/DSP_board.h>
#include <FT17/Broadcast_data.hpp>
#include <FT17/CommProtocol.hpp>

/**
 * @class Boards_ctrl
 * @defgroup Boards_controller Boards_controller
 * @ingroup FT17_driver
 * @brief Boards_ctrl class
 */

class Boards_ctrl
{

public:
        typedef std::map<int, Dsp_Board*>   dsp_map_t;
        typedef std::map<int, FtBoard*>     fts_map_t;

        /**
        * @brief Boards_ctrl constructor
        *        create udp socket and bind to Boards_ctrl::local_addr
        *
        * @param iface eth iface
        * @param expected_num_boards expected number of boards. Defaults to 1.
        */
        Boards_ctrl ( std::string iface, int expected_num_boards );

        /**
        * @brief Boards_ctrl deconstructor
        *
        *        delete recorded DSP_board instances and close udp socket
        *
        */
        ~Boards_ctrl();

        Dsp_Board * get_board ( uint8_t bId ) {
                return ( _boards.find ( bId ) != _boards.end() ) ? _boards[bId] : NULL;
        }
        Dsp_Board * operator[] ( int bId ) {
                return ( _boards.find ( bId ) != _boards.end() ) ? _boards[bId] : NULL;
        }

        /**
        * @brief stop Boards_ctrl::rx_udp(void *_) thread
        *
        */
        void stop_rx_udp();

        void get_sync_data ( void * );

        /**
        * @brief get a bc_data snapshot from dsp boards
        *
        */
        void get_bc_data ( ts_bc_data_t * );


        /**
        * @brief create Boards_ctrl::rx_udp(void *_) thread and initialize
        * mutex and condition variable
        *
        * @return 0
        */
        int init ( void );
        int test ( void );
        bool resetBoard ( int bId, uint8_t bc_rate, uint16_t policy );
        bool ping_board ( int bId );
        bool check_if_pinged ( int bId );
        int getActiveNum ( void ) {
                return _boards.size();
        }
        unsigned int boards_counter;


        /**
         * @brief configure each DSP board in the streaming mode calling Dsp_Board::configure() with possible different bc_rates and policies for different boards
         *
         * @param  bc_rate broadcast rate vector
         * @param  policy requested policy vector
         * @return void
         */
        bool configure_streaming ( std::vector<uint8_t> bc_rate, std::vector<uint16_t> policy );

         /**
         * @brief configure the DSP board in the streaming mode calling Dsp_Board::configure() setting the same bc_rate and policy for all the boards.
         *
         * @param  bc_rate broadcast rate
         * @param  policy requested policy
         * @return void
         */
        void configure_streaming ( uint8_t bc_rate, uint16_t policy );
        
        /**
        * start/stop broadcast of all scanned board
        * for each board set bc_rate via TCP packet
        *
        * @param start_stop    true/false;
        *
        */
        void start_stop_bc_boards ( uint8_t start_stop );
        
        /**
         * @brief configure each DSP board in the polling mode calling Dsp_Board::configure() with possible different policies for different boards
         *
         * @param  policy requested policy vector
         * @return void
         */
        bool configure_polling ( std::vector<uint16_t> policy );
        
        /**
         * @brief configure the DSP board in the polling mode calling Dsp_Board::configure() setting the policy for all the boards.
         *
         * @param  policy requested policy
         * @return void
         */
        void configure_polling ( uint16_t policy );

        /**
        * @brief get a single packet from dsp boards in POLLING mode
        *
        */
        void get_single_data ( ts_single_data_t * );

        /**
        *
        * @return Boards_ctrl::fts_map_t
        */
        fts_map_t get_fts_map();

        /**
        * @brief send a GET_ACTIVE_BOARDS broadcast UDP packet .
        *
        * each board reply with a REPLY_ACTIVE_BOARDS UDP packet, the
        * payload contain information about board type, board ID and IP
        * address.
        *
        * the method Boards_ctrl::factory_board(uint8_t * buff) use
        * this information the create proper DSP_board instance
        *
        * @retval number of active boards;
        *
        **/
        int scan4active ( void );

protected:

        dsp_map_t _boards;
        fts_map_t _fts;

        /**
        * @brief create instances of derived DSP_board class depending on
        * bType
        *
        * @param buff  payload of REPLY_ACTIVE_BOARDS packet
        */
        void factory_board ( uint8_t * );


        /**
        * @brief static thread routine receive all the UDP traffic
        *
        *        call Boards_ctrl::factory_board() and disptch broadcast data
        *        received from boards
        *
        * @param _ actual Board_ctrl object pointer
        *
        * @return void* 0
        */
        static void * rx_udp ( void * );
        void on_bc_data ( uint8_t * );
        int sendUdpPkt ( UDPCommPacket &pkt );

private:

        int         udp_sock;
        pthread_t   rx_upd_thread;
        int         expected_num_boards;

        struct sockaddr_in local_addr;
        struct sockaddr_in  dest_addr;

        std::map<int,bool> pinged_boards;
        pthread_mutex_t udp_sock_mutex;
        pthread_mutex_t data_sync_mutex;
        pthread_cond_t  data_sync_cond;

};



#endif
