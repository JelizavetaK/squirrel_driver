/*
   FT17Interface.h

   Copyright (C) 2015 Italian Institute of Technology

   Developer: Luca Muratore (luca.muratore@iit.it)

*/

#include <string>
// #include <vector>

#include <FT17/Boards_iface.h>
#include <FT17/Broadcast_data.hpp>

#define BROADCAST_SLEEP 0.2
#define WRENCH_SIZE 6
#define MOD_FT_SCALE_FACTOR 1000000.0

typedef struct {

        unsigned int board_id;
        int ch_offs[6];
        float FT[6];
        unsigned short ch_raw[6];
        long temp_Vdc;
        long tStamp;
        int fault;
        float FT_filt[6];

} ft_data;

class FT17Interface
{

private:

        /**
         * @brief copy the ft data and scale it if necessary
         *
         * @param data ft data for the API user
         * @param bc_data the broadcast data to copy
         * @return void
         */
        void copy_and_scale_ft_data ( ft_data& data, const ft_bc_data_t& bc_data );

        /**
         * @brief ethernet interface
         *
         */
        std::string eth_iface;

        /**
         * @brief number of boards
         *
         */
        int boards_num;

        /**
         * @brief broadcast rate
         *
         */
        uint8_t rate;

        /**
         * @brief policy
         *
         */
        uint16_t policy;
        
        /**
         * @brief the operational mode: can be streaming or polling
         * 
         */
        operational_mode mode;

        /**
         * @brief bool flags: true if the boards are configured
         *
         */
        bool configured;

        /**
         * @brief bool flags: true if the boards are initialized
         *
         */
        bool initted;

        /**
         * @brief board controller
         *
         */
        Boards_ctrl *boards_crtl;

        /**
         * @brief timestamped broadcast data
         *
         */
        std::vector<ts_bc_data_t> ts_bc_data;
        
        /**
         * @brief timestamped single data
         *
         */
        std::vector<ts_single_data_t> ts_single_data;

        /**
         * @brief ft map
         *
         */
        Boards_ctrl::fts_map_t ft_boards_map;

public:
        /**
         * @brief Construct the high-level interface with the FT17
         *
         * @param eth_iface eth interface to use
         */
        FT17Interface ( std::string eth_iface );
        
        /**
         * @brief initialize the boards controller and the DSPs
         *
         * @return true on success, false otherwise
         */
        bool init();

        /**
         * @brief configure the DSP in the streaming mode setting the broadcast rate and the policy
         *
         * @param rate requested broadcast rate
         * @param policy requested policy
         * @return void
         */
        void configure_streaming ( uint8_t rate, uint16_t policy );
        
        /**
         * @brief get the broadcast data
         *
         * @param data data that will be filled
         * @return void
         */
        void get_broadcast_data ( ft_data& data );
        
        /**
         * @brief configure the DSP in the polling mode setting the policy
         *
         * @param policy requested policy
         * @return void
         */
        void configure_polling ( uint16_t policy );
        
        /**
         * @brief get the single data in POLLING mode
         *
         * @param data data that will be filled
         * @return void
         */
        void get_single_data ( ft_data& data );

        /**
         * @brief start the broadcast of data
         *
         * @return true if success, false otherwise
         */
        bool start_broadcast();

        /**
         * @brief stop the broadcast of data
         *
         * @return void
         */
        void stop_broadcast();

        /**
         * @brief calibrate offset for the FT boards
         *
         * @return void
         */
        void calibrate_offset();

        /**
         * @brief getter method for the broadcast rate
         *
         * @return the requested broadcast rate
         */
        uint8_t get_broadcast_rate();

        /**
         * @brief getter method for the policy
         *
         * @return the requested policy
         */
        uint16_t get_policy();
        
        /**
         * @brief getter method for the operational mode
         *
         * @return the actual operational mode enum
         */
        operational_mode get_operational_mode();

};
