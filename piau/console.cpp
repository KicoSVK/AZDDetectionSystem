#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>

#include <sys/ioctl.h>

#include "defines.h"
#include "piauSerComm.h"


bool end = false;

int serial_desc;
bool connected = false;

const char * default_portname = "/dev/ttyTHS1";


unsigned char lp_recv[MAX_PACKET_LEN];
unsigned char lp_sent[MAX_PACKET_LEN];
int lpr_len = 0;
int lps_len = 0;




bool is_set(const char num, const int pos)
{
	return ( num & (1 << pos) );
}



#define IS0(n) (is_set((n), 0))
#define IS1(n) (is_set((n), 1))
#define IS2(n) (is_set((n), 2))
#define IS3(n) (is_set((n), 3))
#define IS4(n) (is_set((n), 4))
#define IS5(n) (is_set((n), 5))
#define IS6(n) (is_set((n), 6))
#define IS7(n) (is_set((n), 7))


#define INR(a, b, c) ( ( (b) <= (a) ) && ( (a) <= (c) ) )


void print_menu(const char * pname)
{
        printf(" 0 ....... exit\n");
        printf(" 1 ....... print this menu\n");
        printf(" 2 ....... connect to piau (%s)\n", pname);
        printf(" 3 ....... disconnect\n");
        printf(" 4 ....... read serial number (0x01 0x00)\n");
        printf(" 5 ....... read and set timing (0x01 0x01)\n");
        printf(" 6 ....... read unit state (0x00 0x00)\n");
        printf(" 7 ....... read and set thermal limits (0x01 0x03)\n");
        printf(" 8 ....... configure pull-up resistors (0x01 0x02)\n");
        printf(" 9 ....... reset the unit (0xff 0xff)\n");
        printf(" 10 ...... configure peripherals (0x01 0x05)\n");
        printf(" 11 ...... print last datagram sent (in hexa)\n");
        printf(" 12 ...... print last datagram received (in hexa)\n");
}

int main(int argc, char * argv[])
{
	//int correct = 0;
	//int incorrect = 0;

	char portname[500];

	if (argc == 2)
	{
		strcpy(portname, argv[1]);
	}
	else
	{
		strcpy(portname, default_portname);
	}

	printf("Piau console v 1.0\n(c) AZD Praha s.r.o.\n\n");

	print_menu(portname);

	while (!end)
	{
		printf(" your choice: ");

                char str[256];
                fgets(str, 256, stdin);
                int val = atoi(str);

		switch (val)
		{
			case 0:
				// end this program
				printf(" Bye bye\n\n");
				end = true;
				break;

			case 1:
				print_menu(portname);
				break;

			case 2:
				// opening file (serial port) io
				if (!connected)
				{
					serial_desc = open( portname, O_RDWR | O_NOCTTY | O_SYNC );
					if ( serial_desc < 0 )
					{
						printf(" Error no %d while opening %s\n", errno, portname);
						printf(" Error description: %s\n\n", strerror(errno));
					}
					else
					{
						printf(" Connected to %s\n\n", portname);
						if ( setIfaceAttribs( serial_desc, B115200, 0 ) )
						{
							printf(" Attributes have been set (B115200, 0)\n\n");
							connected = true;
						}
						else
						{
							printf(" Error setting attributes!\n\n");
							connected = false;
						}
						ioctl( serial_desc, TCFLSH, 2 );
					}
				}
				else
				{
					printf(" Already connected!\n\n");
				}
				break;

			case 3:
				// closing open file (serial port) io
				if (connected)
				{
					if (close( serial_desc ) == 0)
					{
						printf(" Successfully disconnected\n\n");
					}
					else
					{
						printf(" Disconnected\n\n");
					}
					connected = false;
				}
				else
				{
					printf(" Not connected yet!\n\n");
				}
				break;

			case 4:
				// read serial number
				if (connected)
				{
					unsigned char data[250];
					data[0] = 0x81;
					data[1] = 0x01;
					data[2] = 0x00;
					data[3] = 0x0a;
					int data_len = 4;

					if (sendData(serial_desc, data, data_len, lp_sent, &lps_len) )
					{
						usleep( 100000 );
						if ( receiveData(serial_desc, data, data_len, lp_recv, &lpr_len) )
						{
							if (data_len != 13)
							{
								printf(" Inappropriate data length... (%d)\n", data_len);
							}
							else
							{
								printf(" Received:\n  Ack: %s\n  Family: %02x\n  S0 - S5: %02x %02x %02x %02x %02x %02x\n  CRC: %02x\n",
									(data[4] == 0x00 ? "OK" : "nepodporovana sluzba"), data[5],
									data[6], data[7], data[8], data[9], data[10], data[11],
									data[12]);
							}
						}
						else
						{
							printf(" Not received!\n");
						}
					}
					else
					{
						printf(" Not sent!\n");
					}
				}
				else
				{
					printf(" Need to be connected first!\n\n");
				}
				break;

			case 5:
				// read timer values
                                if (connected)
                                {
					printf(" set new value y/n: ");
                			char reply[256];
                			fgets(reply, 256, stdin);
                			bool set_val = (reply[0] == 'y') || (reply[0] == 'Y');

					unsigned short int new_val = 0;
					if ( set_val )
					{
						printf(" new value: ");
						fgets(reply, 256, stdin);
                				new_val = atoi(reply);
					}

                                        unsigned char data[250];
                                        data[0] = 0x81;
                                        data[1] = 0x01;
                                        data[2] = 0x01;
                                        data[3] = 0x00;
					data[4] = (set_val ? 0x00 : 0xff);
					memcpy( (void*)(data + 5), (void*)(&new_val), 2);

                                        int data_len = 7;
                                        if ( sendData(serial_desc, data, data_len, lp_sent, &lps_len) )
                                        {
						usleep( 100000 );
                                                if ( receiveData(serial_desc, data, data_len, lp_recv, &lpr_len) )
                                                {
                                                        if ((data_len == 7) || (data_len == 5))
                                                        {
                                                                printf(" Received:\n  Ack: %s\n",
                                                                        (data[4] == 0x00 ? "OK" : ( data[4] == 0x01 ? "nepodporovana sluzba" : "kratky ramec")) );

								if (data_len == 7)
								{
									unsigned short int casovac;
									memcpy( (void*)(&casovac), (void*)(data + 5), 2);
									printf("  Timer: %d sec (%d dni, %d hodin, %d minut, %d sekund)\n", casovac,
									casovac / (86400), (casovac % 86400) / 3600, (casovac % 3600) / 60, casovac % 60 );
								}
                                                        }
							else
							{
								printf(" Inappropriate data length...\n");
							}
                                                }
                                                else
						{
                                                        printf(" Not received!\n");
                                                }
                                        }
                                        else
                                        {
                                                printf(" Not sent!\n");
                                        }

                                }
                                else
                                {
                                        printf(" Need to be connected first!\n\n");
                                }
                                break;



			case 6:
				// read device state
				if (connected)
        	                {
                	        	unsigned char data[250];
                        	        data[0] = 0x81;
                                	data[1] = 0x00;
                                        data[2] = 0x00;
	                                data[3] = 0x0A;
        	                        int data_len = 4;
                	                if ( sendData(serial_desc, data, data_len, lp_sent, &lps_len) )
                        	        {
						usleep( 100000 );
                                		if ( receiveData(serial_desc, data, data_len, lp_recv, &lpr_len) )
                                        	{
							if (data_len != 88)
        	                                        {
                	                                	printf(" Inappropriate data length...\n");
                        	                        }
                                	                else
                                        	        {
                                                		printf(" Received:\n  Ack: %s\n", (data[4] == 0x00 ? "OK" : "nepodporovana sluzba"));
								printf("  FW version: %d.%d\n  Bootloader version: %d.%d\n", data[7], data[8], data[9], data[10]);

								printf("  Kod nazvu jednotky: %d\n", data[12]);

								int delka_behu;
								memcpy( (void*)(&delka_behu), (void*) (data + 13), 4);
								printf("  Doba behu: %d sek (%d dni, %d hodin, %d minut, %d sekund)\n", delka_behu,
									delka_behu / (86400), (delka_behu % 86400) / 3600, (delka_behu % 3600) / 60, delka_behu % 60);

								printf("  Zdroj resetu: hodnota %d\n   Reset po zapnuti: %s\n   Externi reset: %s\n   Reset podpetim: %s\n   Reset watchdogem: %s\n",
									data[19],
									( IS0(data[19]) ? "Y" : "N"),
									( IS1(data[19]) ? "Y" : "N"),
									( IS2(data[19]) ? "Y" : "N"),
									( IS3(data[19]) ? "Y" : "N") );

								short int ai1, ai2, ai3;
								memcpy( (void *)(&ai1), (void *)(data + 22), 2);
								memcpy( (void *)(&ai2), (void *)(data + 24), 2);
								memcpy( (void *)(&ai3), (void *)(data + 26), 2);

								printf(  "  Analogovy vstup 1: %d (%.03f V)\n  Analogovy vstup 2: %d (%.03f V)\n  Analogovy vstup 3: %d (%.03f V)\n",
									ai1, ((float)ai1) * 0.00588f, ai2, ((float)ai2) * 0.04f, ai3, ((float)ai3) * 0.016f);


								unsigned short int teplota, vlhkost;
								memcpy( (void*)(&teplota), (void*)(data+40), 2);
								memcpy( (void*)(&vlhkost), (void*)(data+42), 2);


								// prepocet a vypis
								printf( "  Teplota: %.02f C\n  Vlhkost: %.02f %%\n",
									(165.0f * (((float)teplota) / 65536.0f)) - 40.0f,
									100.0f * ((float)vlhkost) / 65536.0f );

								short int out0volt, out0curr, out1volt, out1curr, out2volt, out2curr, out3volt, out3curr, out4volt, out4curr;

								memcpy( (void*)(&out0volt), (void*)(data+44), 2);
								memcpy( (void*)(&out0curr), (void*)(data+46), 2);

								memcpy( (void*)(&out1volt), (void*)(data+48), 2);
								memcpy( (void*)(&out1curr), (void*)(data+50), 2);

								memcpy( (void*)(&out2volt), (void*)(data+52), 2);
								memcpy( (void*)(&out2curr), (void*)(data+54), 2);

								memcpy( (void*)(&out3volt), (void*)(data+56), 2);
								memcpy( (void*)(&out3curr), (void*)(data+58), 2);


								memcpy( (void*)(&out4volt), (void*)(data+60), 2);
								memcpy( (void*)(&out4curr), (void*)(data+62), 2);

								printf( "  Vystup 0: napeti %.03f V, proud %.03f A\n  Vystup 1: napeti %.03f V, proud %.03f A\n  Vystup 2: napeti %.03f V, proud %.03f A\n  Vystup 3: napeti %.03f V, proud %.03f A\n  Vystup 4: napeti %.03f V, proud %.03f A\n",
									((float)out0volt) * 0.00125f, ((float)out0curr) * 0.00025f,
									((float)out1volt) * 0.00125f, ((float)out1curr) * 0.00025f,
									((float)out2volt) * 0.00125f, ((float)out2curr) * 0.00025f,
									((float)out3volt) * 0.00125f, ((float)out3curr) * 0.00025f,
									((float)out4volt) * 0.00125f, ((float)out4curr) * 0.00025f );


								printf( "  Binarni vstup 1: %s\n  Binarni vstup 2: %s\n  Binarni vstup 3: %s\n",
									( IS0(data[68]) ? "high" : "low" ),
									( IS1(data[68]) ? "high" : "low" ),
									( IS2(data[68]) ? "high" : "low" ) );

								printf( "  Chybovy registr 0: hodnota %d\n   Porucha ventilatoru: %s\n   Porucha topeni: %s\n   Chyba komunikace s ID: %s\n",
									data[69],
									(IS0(data[69]) ? "Y" : "N"),
									(IS1(data[69]) ? "Y" : "N"),
									(IS2(data[69]) ? "Y" : "N") );

								printf( "  Chybovy registr 1: hodnota %d\n   Porucha mereni na vystupu PC: %s\n   Porucha mereni na vystupu ventilatoru: %s\n   Porucha mereni na vystupu kamery: %s\n   Porucha mereni na vystupu topeni: %s\n   Porucha obvodu mereni triggeru: %s\n   Porucha mereni teploty a vlhkosti: %s\n",
									data[70],
									(IS0(data[70]) ? "Y" : "N"),
									(IS1(data[70]) ? "Y" : "N"),
									(IS2(data[70]) ? "Y" : "N"),
									(IS3(data[70]) ? "Y" : "N"),
									(IS4(data[70]) ? "Y" : "N"),
									(IS7(data[70]) ? "Y" : "N") );

								printf( "  Stav zapnuti binarnich vystupu: hodnota %d\n   Vystup napajeni PC: %s\n   Vystup napajeni ventilatoru: %s\n   Vystup napajeni kamery: %s\n   Vystup napajeni topeni: %s\n   Vystup trigru: %s\n",
									data[71],
									(IS0(data[71]) ? "on" : "off"),
									(IS1(data[71]) ? "on" : "off"),
									(IS2(data[71]) ? "on" : "off"),
									(IS3(data[71]) ? "on" : "off"),
									(IS4(data[71]) ? "on" : "off") );

								printf( "  Zkrat / pretizeni binarnich vystupu: hodnota %d\n   Vystup napajeni PC: %s\n   Vystup napajeni ventilatoru: %s\n   Vystup napajeni kamery: %s\n   Vystup napajeni topeni: %s\n   Vystup trigru: %s\n",
									data[72],
									(IS0(data[72]) ? "zkrat/pretizeni" : "OK"),
									(IS1(data[72]) ? "zkrat/pretizeni" : "OK"),
									(IS2(data[72]) ? "zkrat/pretizeni" : "OK"),
									(IS3(data[72]) ? "zkrat/pretizeni" : "OK"),
									(IS4(data[72]) ? "zkrat/pretizeni" : "OK") );

								unsigned short int in1ad, in2ad;

								memcpy( (void*)(&in1ad), (void*)(data + 74), 2);
								memcpy( (void*)(&in2ad), (void*)(data + 77), 2);

								printf( "  AD prevodniky:\n   AD1: %d (%.03f V)\n   AD2: %d (%.03f V)\n",
									in1ad, ((float)in1ad) * 0.01,
									in2ad, ((float)in2ad) * 0.01 );

								printf( "  Stav pull-up rezistoru: hodnota %d\n   Vstup 1: %s\n   Vstup 2: %s\n   Vstup 3: %s\n",
									data[85],
									(IS0(data[85]) ? "on" : "off"),
									(IS1(data[85]) ? "on" : "off"),
									(IS2(data[85]) ? "on" : "off") );

								unsigned short int casovac;
								memcpy( (void*)(&casovac), (void*)(data + 86), 2);
								printf( "  Casovac: %d sek\n", casovac);

                                                        }


						}
        	                                else
                	                        {
							printf(" Incorrect packet\n");
                                        	}
	                                }
					else
					{
						printf(" Not sent!\n");
					}

        	                }
				else
                	        {
                        		printf(" Need to be connected first!\n\n");
                               	}


				//printf(" nekorektni: %d, celkem %d\n", incorrect, correct+incorrect);

                                break;


			case 7:
				// read/set thermal limits
                                if (connected)
                                {
                                        printf(" set new values y/n: ");
                                        char reply[256];
                                        fgets(reply, 256, stdin);
                                        bool set_val = (reply[0] == 'y') || (reply[0] == 'Y');

                                        char nv_zap_top = 0, nv_vyp_top = 0, nv_zap_vent = 0, nv_vyp_vent = 0, nv_vlhkost_zap = 0, nv_vlhkost_vyp = 0;
                                        if ( set_val )
                                        {
                                                printf(" new value (heating ON temperature, 5 .. 50 C): ");
                                                fgets(reply, 256, stdin);
                                                nv_zap_top = atoi(reply);

                                                printf(" new value (heating OFF temperature, 5 .. 50 C): ");
                                                fgets(reply, 256, stdin);
                                                nv_vyp_top = atoi(reply);

                                                printf(" new value (vent ON temperature, 5 .. 50 C): ");
                                                fgets(reply, 256, stdin);
                                                nv_zap_vent = atoi(reply);

                                                printf(" new value (vent OFF temperature, 5 .. 50 C): ");
                                                fgets(reply, 256, stdin);
                                                nv_vyp_vent = atoi(reply);

                                                printf(" new value (vent & heating ON humidity, 0 .. 100 %%): ");
                                                fgets(reply, 256, stdin);
                                                nv_vlhkost_zap = atoi(reply);

                                                printf(" new value (vent & heating OFF humidity, 0.. 100 %%): ");
                                                fgets(reply, 256, stdin);
                                                nv_vlhkost_vyp = atoi(reply);

                                        }


                                        if ( (!set_val) || ( INR( nv_zap_top, 5, 50) && INR(nv_vyp_top, 5, 50) && INR(nv_zap_vent, 5, 50) && INR(nv_vyp_vent, 5, 50)
                                                && INR(nv_vlhkost_zap, 0, 100) && INR(nv_vlhkost_vyp, 0, 100)
                                                && (nv_zap_top < nv_vyp_top) && (nv_vyp_vent < nv_zap_vent) && (nv_vlhkost_vyp < nv_vlhkost_zap) ) )
                                        {
                                                unsigned char data[250];
        	                                data[0] = 0x81;
                	                        data[1] = 0x01;
                        	                data[2] = 0x03;
                                	        data[3] = 0x00;
                                        	data[4] = (set_val ? 0x00 : 0xff);
	                                        data[5] = nv_zap_top;
						data[6] = nv_vyp_top;
						data[7] = nv_zap_vent;
						data[8] = nv_vyp_vent;
						data[9] = nv_vlhkost_zap;
						data[10] = nv_vlhkost_vyp;

	                                        int data_len = 11;
        	                                if ( sendData(serial_desc, data, data_len, lp_sent, &lps_len) )
                	                        {
                        	                        usleep( 100000 );
                                	                if ( receiveData(serial_desc, data, data_len, lp_recv, &lpr_len) )
                                        	        {
                                                	        if ((data_len == 11) || (data_len == 5))
                                                        	{
	                                                                printf(" Received:\n  Ack: %s\n",
        	                                                                (data[4] == 0x00 ? "OK" : ( data[4] == 0x01 ? "nepodporovana sluzba" : ( data[4] == 0x02 ? "kratky ramec" : "nastaveni neulozeno - hodnoty mimo rozsah nebo nesplnuji vzajemne vztahy" ) )) );

        	                                                        if (data_len == 11)
                	                                                {
                        	                                                printf("  Teplota pro zapnuti topeni: %d C\n  Teplota pro vypnuti topeni: %d C\n  Teplota pro zapnuti ventilatoru: %d C\n  Teplota pro vypnuti ventilatoru: %d C\n  Relativni vlhkost pro zapnuti top. & vent.: %d %%\n  Relativni vlhkost pro vypnuti top. & vent.: %d %%\n",
											data[5], data[6], data[7], data[8], data[9], data[10] );
                                        	                        }
                                                	        }
                                                        	else
	                                                        {
        	                                                        printf(" Inappropriate data length...\n");
                	                                        }
                        	                        }
                                	                else
                                        	        {
                                                	        printf(" Not received!\n");
	                                                }
        	                                }
                	                        else
						{
							printf(" Not sent!\n");
						}

					}
					else
					{
						printf(" These values cannot be set!\n");
					}

				}
				else
				{
					printf(" Need to be connected first!\n\n");
				}
				break;

			case 8:
                                // read/set pull-up resistors
                                if (connected)
                                {
                                        printf(" change pull-up configuration y/n: ");
                                        char reply[256];
                                        fgets(reply, 256, stdin);
                                        bool set_val = (reply[0] == 'y') || (reply[0] == 'Y');

                                        bool first = false, second = false, third = false;
                                        if ( set_val )
                                        {
                                                printf(" change 1st pull-up y/n: ");
                                                fgets(reply, 256, stdin);
                                                first = (reply[0] == 'y') || (reply[0] == 'Y');

                                                printf(" change 2nd pull-up y/n: ");
                                                fgets(reply, 256, stdin);
                                                second = (reply[0] == 'y') || (reply[0] == 'Y');

                                                printf(" change 3rd pull-up y/n: ");
                                                fgets(reply, 256, stdin);
                                                third = (reply[0] == 'y') || (reply[0] == 'Y');
                                        }

                                        unsigned char data[250];

					data[0] = 0x81;
					data[1] = 0x01;
                                        data[2] = 0x02;
                                        data[3] = 0x00;
                                        data[4] = (set_val ? 0x00 : 0xff);
                                        data[5] = ( set_val ? ( (first ? 1 : 0) + (second ? 2 : 0) + (third ? 4 : 0)  ) : 0x00);

                                        int data_len = 6;
                                        if ( sendData(serial_desc, data, data_len, lp_sent, &lps_len) )
                                        {
                                        	usleep( 100000 );
                                                if ( receiveData(serial_desc, data, data_len, lp_recv, &lpr_len) )
                                                {
                                                	if ((data_len == 6) || (data_len == 5))
                                                        {
                                                        	printf(" Received:\n  Ack: %s\n",
                                                                	(data[4] == 0x00 ? "OK" : ( data[4] == 0x01 ? "nepodporovana sluzba" : "kratky ramec" ) ) );

                                                                if (data_len == 6)
                                                                {
                                                                	printf( "  Stav pull-up rezistoru: hodnota %d\n   Vstup 1: %s\n   Vstup 2: %s\n   Vstup 3: %s\n",
                                                                        	data[5],
                                                                        	(IS0(data[5]) ? "on" : "off"),
                                                                        	(IS1(data[5]) ? "on" : "off"),
                                                                        	(IS2(data[5]) ? "on" : "off") );

                                                                }
							}
							else
                                                        {
                                                        	printf(" Inappropriate data length...\n");
                                                        }
						}
                                                else
                                                {
                                                	printf(" Not received!\n");
                                                }
					}
                                        else
                                        {
                                        	printf(" Not sent!\n");
                                        }
				}
                                else
                                {
                                        printf(" Need to be connected first!\n\n");
                                }
                                break;




			case 9:
			case 10:
				printf(" Not implemented yet!\n\n");
				break;

			case 11:
				printf( "  Last datagram sent:\n");
				if (lps_len == 0){
					printf("   NONE\n");
				}
				else
				{
					printBuffer(lp_sent, lps_len, "   ");
				}
				break;

			case 12:
				printf( "  Last datagram received:\n");
				if (lpr_len == 0)
				{
					printf("   NONE\n");
				}
				else
				{
					printBuffer(lp_recv, lpr_len, "   ");
				}
				break;

			default:
				printf(" Enter correct choice!\n\n");
				break;
		}

	}


	return 0;
}
