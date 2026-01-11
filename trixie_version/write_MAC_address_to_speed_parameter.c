//	gcc write_MAC_address_to_speed_parameter.c -o write_MAC_address_to_speed_parameter -lrt
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stddef.h>
#include <stdlib.h> // Required for malloc
/* 
#my sensor mac address
D5:1A:DE:86:93:E9
#my speed gradient {mph/mi}
12.000000
#my speed delta {mph}
10.000000
#my maximum speed {mph}
23.500000
#latest throttle value
32767
#my preferred unit of length {miles or km}
mi
#my wheel circumference in mm
2096
#MAC D5:1A:DE:86:93:E9
6:93:E9
#my speed gradient {mph/mi}
12.000000
#my speed delta {mph}
10.000000
#my maximum speed {mph}
23.500000
#latest throttle value
32767
#my preferred unit of length {miles or km}
mi
#my wheel circumference in mm
2096
*/
char* slice_string_copy(const char* str, size_t start, size_t end) 
{
    size_t length = end - start;
    char* result = malloc(length + 1); // Allocate memory for the slice and null terminator
    if (result == NULL) return NULL;   // Check for allocation failure
    strncpy(result, str + start, length);
    result[length] = '\0';             // Manually add the null terminator
    return result;                     // Remember to free the result later
}
int main(int argc, char *argv[])
{	FILE *fp;
	FILE *parameterSTORAGEfile;
	char buffer[1024];
	char bufferTWO[1024];
	char scriptNAME[128];
	char firstLINE[128];
	char* MACaddress; //was [18]
	struct timespec theTIMEnow;
	struct timespec theTIMEthen;
	double elapsedTIME;
	size_t stringSTART=9;
	size_t stringEND=26;
	size_t stringLENGTH;
	int OKtoEXIT = 0;
	char *startingATneedle;
	
	parameterSTORAGEfile = fopen("speed_parameters_version_zero.txt","r+");
	sprintf(scriptNAME,"~/forFLIGHT/expect_script_find_and_trust_sensor");
	/* 
	for (int i = 0; i<13; i++)
		{	fgets(aLINEfromFILE, sizeof(aLINEfromFILE),speedPARAMETERfile);	}*/
	fgets(firstLINE, sizeof(firstLINE),parameterSTORAGEfile);
	
	fp = popen(scriptNAME, "r"); //add MAC address to ehancedSPEEDparameters file
    if (fp == NULL) 
    {	perror("popen failed");
	return 1;
    }
    clock_gettime(CLOCK_MONOTONIC, &theTIMEthen);
    /*
    do
	{	clock_gettime(CLOCK_MONOTONIC, &theTIMEnow);
		elapsedTIME = (theTIMEnow.tv_sec - theTIMEthen.tv_sec)+ (theTIMEnow.tv_nsec - theTIMEthen.tv_nsec) / 1000000000.0;
	}
	while (elapsedTIME < 5);
	*/
    while ((fgets(buffer, sizeof(buffer), fp) != NULL) && (OKtoEXIT < 2))
    {	printf("%s ",buffer);
		//when the buffer contains "Discovering: yes" wait 5 seconds and log al devices recognized
		char needle[] = " Device ";
		 
		//char *startingATneedle = strstr(bufferTWO,needle);
		strcpy(bufferTWO,buffer);
		int originalLENGTH = strlen(buffer);
		printf("Length %4d OKtoEXIT> %4d \n",originalLENGTH,OKtoEXIT);
		
		if ((originalLENGTH == 92) && (OKtoEXIT==1))//was 81 // && (OKtoEXIT==1)
			{	startingATneedle = strstr(buffer,needle);
				printf("startingATneedle <<%s>>\n",startingATneedle);	//backslash not needed
				//      startingATneedle << Device D5:1A:DE:86:93:E9 Wahoo SPEED 48DD
				//                         012345678901234567890123456789
				//                                   1         2
				stringSTART = (size_t) 8;
				stringEND = (size_t) 25;
				printf("stringSTART <<<%3zu>> stringEND <<%3zu>>\n",stringSTART,stringEND);
				//MACaddress = slice_string_copy(startingATneedle,stringSTART,stringEND);
			    /* 	char* slice_string_copy(const char* str, size_t start, size_t end) 
					{
						size_t length = end - start;
						char* result = malloc(length + 1); // Allocate memory for the slice and null terminator
						if (result == NULL) return NULL;   // Check for allocation failure
							strncpy(result, str + start, length);
						result[length] = '\0';             // Manually add the null terminator
						return result;                     // Remember to free the result later
					}
				*/
				stringLENGTH = stringEND-stringSTART;
				MACaddress = malloc(stringLENGTH+(size_t)1);
				strncpy(MACaddress,startingATneedle+stringSTART,stringLENGTH);
				MACaddress[stringLENGTH+(size_t)1] = '\0';
				printf("MACaddress <<%s>>\ns",MACaddress);
				fputs(MACaddress,parameterSTORAGEfile);
				OKtoEXIT++;
			}
		else
			if (originalLENGTH == 92)	
				OKtoEXIT++;
	}
	
	fclose(fp);
	fclose(parameterSTORAGEfile);
    return 0;
} 

