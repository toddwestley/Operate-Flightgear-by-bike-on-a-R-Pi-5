//	gcc write_tire_size_to_speed_parameter.c -o write_tire_size_to_speed_parameter
#include <stdio.h>
#include <string.h>
/* 
0	#my sensor mac address
1	D5:1A:DE:86:93:E9
2	#my speed gradient {mph/mi}
3	12.000000
4	#my speed delta {mph}
5	10.000000
6	#my maximum speed {mph}
7	23.500000
8	#latest throttle value
9	32767
10	#my preferred unit of length {miles or km}
11	mi
12	#my wheel circumference in mm
13	2096
*/
int main(int argc, char *argv[])
{	int fromDIALOGUE;
	FILE *speedPARAMETERfile;
	char aLINEfromFILE[100];
	char aLINEtoWRITE[100];
	speedPARAMETERfile = fopen("speed_parameters_version_zero.txt","r+");
	for (int i = 0; i<13; i++)
		{	fgets(aLINEfromFILE, sizeof(aLINEfromFILE),speedPARAMETERfile);	}
	sscanf(argv[1],"%d",&fromDIALOGUE);
	sprintf(aLINEtoWRITE,"%s\n",argv[1]); //was %d
	fputs(aLINEtoWRITE, 	speedPARAMETERfile);
	fclose(	speedPARAMETERfile);
	//printf("percent argv[1] = %s\n",argv[1]);
	//printf(">> %s\n",fromDIALOGUE);
	return 0;
}
