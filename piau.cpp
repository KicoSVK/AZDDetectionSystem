#include "piau.h"


#if __has_include("cameraGrabModule.h")    
	Piau::Piau()
	{
	    connect();
	}


void Piau::connect()
{
    if (!connected)
    {
        serial_desc = open( default_portname, O_RDWR | O_NOCTTY | O_SYNC );
        if ( serial_desc < 0 )
        {
            printf(" Error no %d while opening %s\n", errno, default_portname);
            printf(" Error description: %s\n\n", strerror(errno));
        }
        else
        {
			std::cout << "" << std::endl;
            printf("PIAU [Connected to %s !]\n", default_portname);
            if ( setIfaceAttribs( serial_desc, B115200, 0 ) )
            {
                //printf(" Attributes have been set (B115200, 0)\n\n");
                connected = true;
            }
            else
            {
                printf(" Error setting attributes when connection start!\n\n");
                connected = false;
            }
            ioctl( serial_desc, TCFLSH, 2 );
        }
    }
    else
    {
        printf(" Already connected!\n\n");
    }
}

bool Piau::is_set(const char num, const int pos)
{
    return (num & (1 << pos));
}

std::tuple<float, float, float, float, float, float, float, float, float, float, float, int> Piau::getPiauValues()
{	
	unsigned short int teplota, vlhkost;
	short int out0volt, out0curr, out1volt, out1curr, out2volt, out2curr, out3volt, out3curr, out4volt, out4curr;
	int delka_behu;
	short int ai1, ai2, ai3;
	unsigned short int in1ad, in2ad;

	if (connected)
	{		
		unsigned char data[250];
		data[0] = 0x81;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x0A;
		int data_len = 4;
		if (sendData(serial_desc, data, data_len, lp_sent, &lps_len))
		{
			usleep(100000);
			if (receiveData(serial_desc, data, data_len, lp_recv, &lpr_len))
			{
				if (data_len != 88)
				{
					printf(" Inappropriate data length...\n");
				}
				else
				{					
					memcpy((void*)(&delka_behu), (void*)(data + 13), 4);					

					
					memcpy((void*)(&ai1), (void*)(data + 22), 2);
					memcpy((void*)(&ai2), (void*)(data + 24), 2);
					memcpy((void*)(&ai3), (void*)(data + 26), 2);

					memcpy((void*)(&teplota), (void*)(data + 40), 2);
					memcpy((void*)(&vlhkost), (void*)(data + 42), 2);

					memcpy((void*)(&out0volt), (void*)(data + 44), 2);
					memcpy((void*)(&out0curr), (void*)(data + 46), 2);

					memcpy((void*)(&out1volt), (void*)(data + 48), 2);
					memcpy((void*)(&out1curr), (void*)(data + 50), 2);

					memcpy((void*)(&out2volt), (void*)(data + 52), 2);
					memcpy((void*)(&out2curr), (void*)(data + 54), 2);

					memcpy((void*)(&out3volt), (void*)(data + 56), 2);
					memcpy((void*)(&out3curr), (void*)(data + 58), 2);


					memcpy((void*)(&out4volt), (void*)(data + 60), 2);
					memcpy((void*)(&out4curr), (void*)(data + 62), 2);

					

					memcpy((void*)(&in1ad), (void*)(data + 74), 2);
					memcpy((void*)(&in2ad), (void*)(data + 77), 2);					

					unsigned short int casovac;
					memcpy((void*)(&casovac), (void*)(data + 86), 2);		
				}

				float temp = (165.0f * (((float)teplota) / 65536.0f)) - 40.0f;
				float temperatureFromAD1 = (-30.0f + (110.0f * ((((float)in1ad) * 0.01) / 10.0f)));
				float humidity = 100.0f * ((float)vlhkost) / 65536.0f;
				float voltage0 = ((float)out0volt) * 0.00125f;
				float voltage1 = ((float)out1volt) * 0.00125f;
				float voltage2 = ((float)out2volt) * 0.00125f;
				float voltage3 = ((float)out3volt) * 0.00125f;

				float current0 = ((float)out0curr) * 0.00025f;
				float current1 = ((float)out1curr) * 0.00025f;
				float current2 = ((float)out2curr) * 0.00025f;
				float current3 = ((float)out3curr) * 0.00025f;

				/*returnedValues[0] = temp;
				returnedValues[1] = humidity;

				returnedValues[2] = voltage0;
				returnedValues[3] = voltage1;
				returnedValues[4] = voltage2;
				returnedValues[5] = voltage3;
				returnedValues[6] = voltage4;

				returnedValues[7] = current0;
				returnedValues[8] = current1;
				returnedValues[9] = current2;
				returnedValues[10] = current3;
				returnedValues[11] = current4;*/


				/*returnedValues[12] = delka_behu;*/
				return { temp, temperatureFromAD1, humidity, voltage0, voltage1,  voltage2, voltage3, current0, current1, current2, current3, delka_behu };

				
			}
			else
			{
				printf(" PIAU: Incorrect packet\n");
			}
		}
		else
		{
			printf(" PIAU Not sent!\n");
		}
	}
	else
	{
		printf(" PIAU: Need to be connected first!\n\n");
	}		
}

int Piau::getDiskFreeSpace()
{
	//https://stackoverflow.com/questions/965615/discrepancy-between-call-to-statvfs-and-df-command
	//printf("total free disk space of the partition: %d GB \n",(stat.f_bavail) * 8 / 2097152);
	//512 is 2^9 - one half of a kilobyte. 
	//A kilobyte is 2^10. A megabyte is 2^20. A gigabyte is 2^30. A terabyte
	//is 2^40. And so on. The common computer units go up by 10's of powers
	//of 2 like that.

	struct statvfs fiData;
	struct statvfs* fpData;
	char fnPath[128];

	float freeSpace = 0;

	strcpy(fnPath, "/");
	if ((statvfs(fnPath, &fiData)) < 0) {
		printf("Failed to stat %s:\n", fnPath);
	}
	else {		
		freeSpace = fiData.f_bsize * fiData.f_bavail;
		freeSpace = freeSpace / (fiData.f_blocks * fiData.f_bsize);
	}
	

	return (int)100 - (round((freeSpace * 100.0)));
}

int Piau::getNVMEDiskFreeSpace()
{
	//https://stackoverflow.com/questions/965615/discrepancy-between-call-to-statvfs-and-df-command
	//printf("total free disk space of the partition: %d GB \n",(stat.f_bavail) * 8 / 2097152);
	//512 is 2^9 - one half of a kilobyte. 
	//A kilobyte is 2^10. A megabyte is 2^20. A gigabyte is 2^30. A terabyte
	//is 2^40. And so on. The common computer units go up by 10's of powers
	//of 2 like that.

	struct statvfs fiData;
	struct statvfs* fpData;
	char fnPath[128];

	float freeSpace = 0;

	strcpy(fnPath, "/mnt/nvme");
	if ((statvfs(fnPath, &fiData)) < 0) {
		printf("Failed to stat %s:\n", fnPath);
	}
	else {
		freeSpace = fiData.f_bsize * fiData.f_bavail;
		freeSpace = freeSpace / (fiData.f_blocks * fiData.f_bsize);
	}

	return (int)100 - (round((freeSpace * 100.0)));
}

//https://stackoverflow.com/questions/2513505/how-to-get-available-memory-c-g
int Piau::getFreeSystemMemory()
{
	float pages = (float)sysconf(_SC_AVPHYS_PAGES);
	float page_size = (float)sysconf(_SC_PAGE_SIZE);
	float pages_total = (float)sysconf(_SC_PHYS_PAGES);
	int memoryUsage;

	memoryUsage = (int)round((1 - (pages / pages_total)) * 100);
	
	return 	memoryUsage;
}

int Piau::getCpuUsage()
{
	ifstream loadAvgFile("/proc/loadavg");
	std::string loadAvg;
	float loadAvgFloat;
	if (loadAvgFile.good())
	{		
		getline(loadAvgFile, loadAvg);		
		loadAvg = loadAvg.substr(0, loadAvg.find(" ")); 
	}	
	loadAvgFloat = std::stof(loadAvg);
	if(loadAvgFloat >= 6.0)
	{
		loadAvgFloat = 100.0;
	}else
	{
		loadAvgFloat = loadAvgFloat / 6.0;
	}	
	loadAvgFloat = round(loadAvgFloat * 100);
	return (int)loadAvgFloat;
}

int Piau::getGpuUsage()
{
	ifstream loadAvgFile("/sys/devices/gpu.0/load");
	std::string loadAvg;
	float loadAvgFloat;
	if (loadAvgFile.good())
	{		
		getline(loadAvgFile, loadAvg);
		loadAvg = loadAvg.substr(0, loadAvg.find(" "));
	}
	
	loadAvgFloat = round(std::stof(loadAvg) / 10);
	return (int)loadAvgFloat;
}

float Piau::getSystemTemp()
{
	ifstream loadAvgFile("/sys/devices/virtual/thermal/thermal_zone0/temp");
	std::string loadAvg;
	float loadAvgFloat;
	if (loadAvgFile.good())
	{		
		getline(loadAvgFile, loadAvg);
		loadAvg = loadAvg.substr(0, loadAvg.find(" "));
	}
	
	loadAvgFloat = std::stof(loadAvg) / 1000;
	return loadAvgFloat;
}

float Piau::getGPUTemp()
{
	ifstream loadAvgFile("/sys/devices/virtual/thermal/thermal_zone2/temp");
	std::string loadAvg;
	float loadAvgFloat;
	if (loadAvgFile.good())
	{
		getline(loadAvgFile, loadAvg);
		loadAvg = loadAvg.substr(0, loadAvg.find(" "));
	}

	loadAvgFloat = std::stof(loadAvg) / 1000;
	return loadAvgFloat;
}

int Piau::getSystemUptime()
{
	ifstream loadAvgFile("/proc/uptime");
	std::string loadAvg;
	float loadAvgFloat;
	if (loadAvgFile.good())
	{
		getline(loadAvgFile, loadAvg);
		loadAvg = loadAvg.substr(0, loadAvg.find(" "));
	}

	loadAvgFloat = round(std::stof(loadAvg));
	return int(loadAvgFloat);
}

nlohmann::json Piau::getStatusJson(std::tuple<float, float, float, float, float, float, float, float, float, float, float, int> PIAUValues)
{
	//"sourceId":"059897a9-a940-4ce3-94f2-cdb6c89ddb53",
	//	"time" : "2020-03-17T09:30:00+00",
	//	"temperature1" : 36.8,
	//	"temperature2" : -20.5,
	//	"humidity1" : 89,
	//	"humidity2" : 84,
	//	"freeDiskSpace" : 10000,
	//	"freeMemSpace" : 100,
	//	"cpuUsage" : 25,
	//	"gpuUsage" : 10,
	//	"voltage" : 14.75,
	//	"batteryState" : "discharging",
	//	"batteryAvailableCapacity" : 98


	/// ////////////////////new version//////////////////////
	///
	// "sourceId": "c9931362-1ed9-4c76-97cc-43a71b49fb60",
	// "time": "2022-03-16T19:55:00+00",
	//	"temperature1" : 11,
	//	"temperature2" : 12,
	//	"temperature3" : 13,
	//	"temperature4" : 14,
	//	"temperature5" : 15,
	//	"uptime1" : 21,
	//	"uptime2" : 22,
	//	"humidity1" : 31,
	//	"humidity2" : 32,
	//	"freeDiskSpace1" : 51,
	//	"freeDiskSpace2" : 42,
	//	"voltage1" : 51,
	//	"voltage2" : 52,
	//	"voltage3" : 53,
	//	"voltage4" : 54,
	//	"voltage5" : 55,
	//	"current1" : 61,
	//	"current2" : 62,
	//	"current3" : 63,
	//	"current4" : 64,
	//	"current5" : 65,
	//	"freeMemSpace" : 70,
	//	"cpuUsage" : 69,
	//	"gpuUsage" : 70,
	//	"batteryState" : "connected",
	//	"batteryAvailableCapacity" : 89


	json statusJson;

	char timeInChars[100];
	std::string timInString;
	time_t nowTime = time(0);
	struct tm localTime = { 0 };
	localtime_r(&nowTime, &localTime);
	
	strftime(timeInChars, 100, "%Y-%m-%dT%H:%M:%S", localtime(&nowTime));
	timInString = timeInChars;
	std::stringstream timeOff;	
	timeOff << std::setw(2) << std::setfill('0') << 0;
	timInString = timInString + std::string("+") + timeOff.str();	
	
	statusJson["sourceId"] = cameraIdJson;	
	statusJson["time"] = timInString.c_str();	
	//statusJson["temperature1"] = roundf(temperature1 * 100) / 100;
	statusJson["temperature1"] = std::get<0>(PIAUValues);
	statusJson["temperature2"] = getSystemTemp();
	statusJson["temperature3"] = getGPUTemp();
	//statusJson["temperature4"] = getGPUTemp();
	statusJson["temperature5"] = std::get<1>(PIAUValues);

	statusJson["uptime1"] = getSystemUptime();
	statusJson["uptime2"] = std::get<11>(PIAUValues);
	
	statusJson["humidity1"] = (int)round(std::get<2>(PIAUValues));
	//statusJson["humidity2"] = (int)round(std::get<1>(PIAUValues));

	//statusJson["freeDiskSpace1"] = (int)100-(round((getDiskFreeSpace() * 100.0)));
	//statusJson["freeDiskSpace2"] = (int)100-(round((getNVMEDiskFreeSpace() * 100.0)));	
	statusJson["freeDiskSpace1"] = getDiskFreeSpace();
	statusJson["freeDiskSpace2"] = getNVMEDiskFreeSpace();

	statusJson["freeMemSpace"] = getFreeSystemMemory();

	statusJson["cpuUsage"] = getCpuUsage();
	statusJson["gpuUsage"] = getGpuUsage();

	statusJson["voltage1"] = std::get<3>(PIAUValues);
	statusJson["voltage2"] = std::get<4>(PIAUValues);
	statusJson["voltage3"] = std::get<5>(PIAUValues);
	statusJson["voltage4"] = std::get<6>(PIAUValues);


	statusJson["current1"] = std::get<7>(PIAUValues);
	statusJson["current2"] = std::get<8>(PIAUValues);
	statusJson["current3"] = std::get<9>(PIAUValues);
	statusJson["current4"] = std::get<10>(PIAUValues);


	statusJson["batteryState"] = "disconnected";
	statusJson["batteryAvailableCapacity"] = 0;

	//std::cout << "event status: " << statusJson.dump() << std::endl;
	return statusJson;
}

void Piau::statusSending()
{
	
	while (!stopGrabFrames)
	{
		try
		{
			//float actualPiauValues[4] = { 0.0, 0.0, 0.0, 0.0 };
			/*getPiauValues(actualPiauValues);*/
			json actualStatusJson = getStatusJson(getPiauValues());

			mutualJsonClientSendStatus(targetIPaddress, targetPort, sslAderosCertificatePath, sslCameraCertificatePath, sslCameraKeyPath, actualStatusJson);
			sleep(60000);			
		}
		catch (const std::exception& e)
		{
			std::cout << "Exception was caught. Message: '" << e.what() << std::endl;
			std::cout << __LINE__ << " " << __func__ << std::endl;

			std::ofstream fileToWrite;
			fileToWrite.open("exceptions.log");
			fileToWrite << "Exception was caught. Message: '" << e.what() << "'\n";
			fileToWrite << __LINE__ << " " << __func__ << std::endl << std::endl;
			fileToWrite.close();
		}
	}
}


std::thread Piau::startStatusSending()
{
	std::thread statusSendingThread = std::thread(&Piau::statusSending, this);
	return statusSendingThread;	
}


bool setIfaceAttribs(int fd, const int speed, const int parity)
{
	struct termios tty;
	if (tcgetattr(fd, &tty) != 0)
	{
		printf(" Error %d from tcgetattr\n", errno);
		return false;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading

	tty.c_iflag &= ~(INLCR | ICRNL);// 0x0d 0x0a issue

	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf(" Error %d from tcsetattr\n", errno);
		return false;
	}
	return true;
}

bool sendData(int fd, const unsigned char* data, const int count, unsigned char* last_packet_sent = NULL, int* last_sent_len = NULL)
{
	bool ret_val = false;

	if ((count + 6) <= MAX_PACKET_LEN)
	{
		unsigned char tx_buff[MAX_PACKET_LEN];
		int i = 0;

		unsigned short crc = 0;

		// sestaveni ramce k vyslani (pridani, DLE, SYN, ... a CRC
		tx_buff[i++] = 0x10; 				               	// start sekvence DLE SYN
		tx_buff[i++] = 0x16;
		for (int j = 0; j < count; j++)
		{
			tx_buff[i++] = data[j];
			crc = crc16_table[(crc >> 8) ^ data[j]] ^ (crc << 8);
			if (data[j] == 0x10)
				tx_buff[i++] = 0x10;      				 			// transparentni znak DLE
		}
		tx_buff[i++] = 0x10;                				// stop sekvence DLE ETX
		tx_buff[i++] = 0x03;
		tx_buff[i++] = (unsigned char)(crc >> 8); 	// CRC Hi
		tx_buff[i++] = (unsigned char)crc; 	      // CRC Lo

		int k;
		if ((k = write(fd, tx_buff, i)) == i)
		{
			ret_val = true;

			//printBuffer(tx_buff, i);
			//printf("\n\n");
			if (last_packet_sent != NULL)
			{
				memcpy(last_packet_sent, tx_buff, i);
				if (last_sent_len != NULL)
				{
					(*last_sent_len) = i;
				}
			}
		}
	}

	return ret_val;
}

bool receiveData(int fd, unsigned char* buffer, int& delka_dat, unsigned char* last_packet_recv = NULL, int* last_recv_len = NULL)
{
	int rx_state = CEKANI_NA_10H;

	unsigned short crc = 0;
	unsigned short crcDebug;
	unsigned char recv_char, crcDebug1, crcDebug2;
	int i = 0;
	bool prijat_paket = false;
	bool flag_DLE = false;

	//printf("receiveData: ");
	if (last_recv_len != NULL)
	{
		*last_recv_len = 0;
	}

	while (!prijat_paket)
	{
		//printf("reading....\n");

		//while (true)
		//{
		int pocet_bytu = read(fd, &recv_char, 1);
		//}

		//int pocet_bytu = 0;

		//do
		//{
			//printf("pocet bytu %d\n", pocet_bytu);
		//	int pocet_bytu = read(fd, &recv_char, 1);
		//} while (pocet_bytu != 0);

		if (pocet_bytu != 1)
		{
			//printf("bacha bacha\n");
			return false;
		}

		if (last_packet_recv != NULL)
		{
			last_packet_recv[*last_recv_len] = recv_char;
			(*last_recv_len)++;
		}

		if (i == MAX_PACKET_LEN)
		{
			rx_state = KRATKY_BUFFER;
			break;
		}

		switch (rx_state)
		{
		case CEKANI_NA_10H:
			if (recv_char == 0x10)
			{
				rx_state = CEKANI_NA_16H;
			}
			break;


		case CEKANI_NA_16H:
			if (recv_char == 0x16)
			{
				crc = 0;
				rx_state = PRIJEM_DAT;
			}
			else
			{
				rx_state = CEKANI_NA_10H;
			}
			break;


		case PRIJEM_DAT:
			if (!flag_DLE)
			{
				// minule nebylo prijato DLE
				if (recv_char != 0x10)
				{
					// ted taky neni prijat znak DLE
					// ulozeni do bufferu, prubezny vypocet CRC
					buffer[i++] = recv_char;
					crc = crc16_table[(crc >> 8) ^ recv_char] ^ (crc << 8);
				}
				else
				{
					flag_DLE = true;
				}
			}
			else
			{
				// minule byl prijat znak DLE
				flag_DLE = false;

				switch (recv_char)
				{
				case 0x10:
					// prijato DLE DLE - znak DLE ulozime do bufferu
					buffer[i++] = recv_char;
					crc = crc16_table[(crc >> 8) ^ recv_char] ^ (crc << 8);
					break;

				case 0x16:
					// prijato DLE SYN - start sekvence, zustavame ve stavu 2
					i = 0;
					crc = 0;
					break;

				case 0x03:
					// prijato DLE ETX - ukoncovaci sekvence - dale ocekavej CRC Hi
					rx_state = CEKANI_CRC1;
					break;

				default:
					// po DLE prijato neco jineho nez DLE, SYN, ETX
					rx_state = CEKANI_NA_10H;
					break;
				}
			}

			break;

		case CEKANI_CRC1:
			// ocekavame prvni byte CRC Hi
			// prijato CRC Hi - zahrneme ho do vyppoctu
			crcDebug = crc;
			crcDebug1 = recv_char;

			crc = crc16_table[(crc >> 8) ^ recv_char] ^ (crc << 8);
			rx_state = CEKANI_CRC2;
			break;

		case CEKANI_CRC2:
			// ocekavame druhy byte CRC Lo
			crcDebug2 = recv_char;

			crc = crc16_table[(crc >> 8) ^ recv_char] ^ (crc << 8);

			// test CRC
			if (crc == 0)
			{
				// prijat ramec
				//FrameReceived (_rxDataBuffer, uk);

				//countRx++;
				rx_state = PRIJAT_PAKET;
			}
			else
			{
				//printf("%02X %02X -> %02X %02X\n", (crcDebug>>8) & 0xff, crcDebug & (0xff),
				//	crcDebug1, crcDebug2 );
				rx_state = NESEDI_CRC;
			}

			prijat_paket = true;
			break;

		} // END switch (rx_state)

	} // END while....

	printf("\n");
	delka_dat = i;
	//printf("return code %d\n", rx_state);
	return (rx_state == PRIJAT_PAKET);
}



void printBuffer(const unsigned char* buffer, const int count, const char* preLine)
{
	if (count > 0)
	{
		int i = 0;
		for (i = 0; i < count; i++)
		{
			if (i % 10 == 0)
			{
				printf("%s", preLine);
			}

			printf("%02x ", buffer[i]);
			if ((i + 1) % 10 == 0)
			{
				printf("\n");
			}
		}

		if (i % 10 != 0)
		{
			printf("\n");
		}
	}
}

#else
	Piau::Piau()
	{

	}

	void Piau::statusSending()
	{
		
	}

	std::thread Piau::startStatusSending()
	{
		std::thread statusSendingThread = std::thread(&Piau::statusSending, this);
		return statusSendingThread;
	}

#endif