#include "defines.h"
#include "piauSerComm.h"
#include <stdio.h>
#include <unistd.h>

bool receiveData( int fd, unsigned char * buffer, int& delka_dat, unsigned char * last_packet_recv = NULL, int * last_recv_len = NULL )
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
			printf("bacha bacha\n");

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
						crc = crc16_table [(crc >> 8) ^ recv_char] ^ (crc << 8);
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
							crc = crc16_table [(crc >> 8) ^ recv_char] ^ (crc << 8);
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

				crc = crc16_table [(crc >> 8) ^ recv_char] ^ (crc << 8);
				rx_state = CEKANI_CRC2;
				break;

			case CEKANI_CRC2:
				// ocekavame druhy byte CRC Lo
				crcDebug2 = recv_char;

				crc = crc16_table [(crc >> 8) ^ recv_char] ^ (crc << 8);

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


