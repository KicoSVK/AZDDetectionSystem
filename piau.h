#ifndef	PIAU_H_
#define PIAU_H_
#include <thread>

#if __has_include("cameraGrabModule.h")
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include <iostream>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include "defines.h"
#include <fstream>

#include <sys/ioctl.h>

#include "utilities.h"
#include <sys/statvfs.h>

#if __has_include("cameraGrabModule.h")    
#include "mutualJsonClient.h"

#endif

#define CEKANI_NA_10H 0
#define CEKANI_NA_16H 1
#define PRIJEM_DAT    2
#define CEKANI_CRC1   3
#define CEKANI_CRC2   4
#define PRIJAT_PAKET  5
#define KRATKY_BUFFER 6
#define NESEDI_CRC    7

#define MOC_DAT       10

#define MAX_PACKET_LEN 2000

const unsigned short int crc16_table[256] =
{
0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

bool receiveData(int fd, unsigned char* buffer, int& delka_dat, unsigned char* last, int* last2);

bool sendData(int fd, const unsigned char* data, const int count, unsigned char* last, int* last2);

bool setIfaceAttribs(int fd, const int speed, const int parity);

void printBuffer(const unsigned char* buffer, const int count, const char* preLine);

#define IS0(n) (is_set((n), 0))
#define IS1(n) (is_set((n), 1))
#define IS2(n) (is_set((n), 2))
#define IS3(n) (is_set((n), 3))
#define IS4(n) (is_set((n), 4))
#define IS5(n) (is_set((n), 5))
#define IS6(n) (is_set((n), 6))
#define IS7(n) (is_set((n), 7))

#define INR(a, b, c) ( ( (b) <= (a) ) && ( (a) <= (c) ) )

extern bool stopGrabFrames;

/*! Piau class is for comunnication with PIAU board, prepare and send status message */
class Piau {

    public:

        /**
         * Default constructor for Piau class
         */
		Piau();

        /**
         * Function for start periodical send status message
         *
         * @param std::string stringToClean is string which will be cleaned
         * @return std::thread with periodical sending status message
         */
        std::thread startStatusSending();

        

    private:
        bool end = false; /*!< Bool for end */
        int serial_desc; /*!< Int for serial DESC */
        bool connected = false; /*!< Bool wheter PIAU comunnication is connected or not  */
        const char* default_portname = "/dev/ttyTHS1"; /*!< Char* for define serial port  */

        unsigned char lp_recv[MAX_PACKET_LEN]; /*!< Unsigned char for data receiving */
        unsigned char lp_sent[MAX_PACKET_LEN]; /*!< Unsigned char for data sending */
        int lpr_len = 0; /*!< Int for lpr len */
        int lps_len = 0; /*!< Int for lps len */

        /**
         * Function for connect to PIAU board        
         */
        void connect();
        bool is_set(const char num, const int pos);

        /**
         * Function for get requiered values from PIAU board
         *         
         * @return std::tuple with values:
               *temperature, 
               *temperature from AD1, 
               *humidity, 
               *voltage0, 
               *voltage1,  
               *voltage2, 
               *voltage3, 
               *current0, 
               *current1, 
               *current2, 
               *current3, 
               *up time
         */
        std::tuple<float, float, float, float, float, float, float, float, float, float, float, int> getPiauValues();

        /**
         * Function for get actual free disk space in percentage
         *         
         * @return int actual free disk space in percentage
         */
        int getDiskFreeSpace();

        /**
         * Function for get actual free NVME disk space in percentage
         *
         * @return int actual free NVME disk space in percentage
         */
        int getNVMEDiskFreeSpace();

        /**
         * Function for get actual free system memory in percentage
         *
         * @return int actual free system memory in percentage
         */
        int getFreeSystemMemory();

        /**
         * Function for get actual CPU usage
         *
         * @return int actual actual CPU usage
         */
        int getCpuUsage();

        /**
         * Function for get actual GPU usage in percentage
         *
         * @return int actual actual GPU usage in percentage
         */
        int getGpuUsage();

        /**
         * Function for get actual system temperature
         *
         * @return float actual system temperature
         */
        float getSystemTemp();

        /**
         * Function for get actual GPUT temperature
         *
         * @return float actual GPU temperature
         */
        float getGPUTemp();

        /**
         * Function for get actual system (Jetson) uptime
         *
         * @return float actual system (Jetson) uptime
         */
        int getSystemUptime();

        /**
         * Function for generate status JSON
         * @param std::tuple with values:
               *temperature, 
               *temperature from AD1, 
               *humidity, 
               *voltage0, 
               *voltage1,  
               *voltage2, 
               *voltage3, 
               *current0, 
               *current1, 
               *current2, 
               *current3, 
               *up time
         * @return nlohmann::json status JSON
         */
        nlohmann::json getStatusJson(std::tuple<float, float, float, float, float, float, float, float, float, float, float, int> PIAUValues);

        /**
         * Function for send status message       
         */
        void statusSending();      
        
};
#else
class Piau {

	public:
	    Piau();
        void statusSending();
        std::thread startStatusSending();

	private:
        
};
#endif
#endif
