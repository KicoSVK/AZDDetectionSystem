#include <stdio.h>


void printBuffer( const unsigned char * buffer, const int count, const char * preLine)
{
	if (count > 0)
	{

		int i = 0;
		for (i = 0; i < count; i++)
		{
			if ( i % 10 == 0)
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


