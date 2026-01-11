//	./expect_script_thirty D5:1A:DE:86:93:E9
//	google search:	"popen script"
//	gcc createAjoystickSOmyBIKEcanFLY.c -o createAjoystickSOmyBIKEcanFLY
// gcc createAjoystickSOmyBIKEcanFLY.c -o createAjoystickSOmyBIKEcanFLY -lm
//	string looking for:		27 91 ||[Wahoo SPEED 48DD:/service0022/char0023]# Iteration 22 of 5000\n  01 2f 00 00 00 2c aa
//
// google AI: "joystick emulation linux "c program*""
// https://github.com/GrantEdwards/uinput-joystick-demo/blob/master/uinput-demo.c

#include <stdio.h>
#include <string.h>
#include <stdlib.h> // Required for malloc
#include <math.h> // Required for the pow() function
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/uinput.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
//#define _POSIX_C_SOURCE 199309L

//#include <unistd.h>
//#include <unistd.h>
//#include <iostream>
//int joystick_node;
struct input_event ev[1]; 

static void setup_abs(int joystick_node, unsigned chan, int min, int max);
int findNewLineInstring (char buffer[1024])
{	int returnInteger = 0;
	while (((buffer[returnInteger] != 12) && (buffer[returnInteger] != 13)) && (buffer[returnInteger] != 10))
		returnInteger ++;
	return returnInteger;
}
int isThisCharacterHex (char thisCharacter)
{	int returnValue = 0;
	if (((thisCharacter >= 48) && (thisCharacter <= 57))
		|| ((thisCharacter >=65) && (thisCharacter <= 70)))
			{	returnValue = 1;}
	return returnValue;
}
int isThisCharacterSpace (char thisCharacter)
{	int returnValue = 0;
	if (thisCharacter == 32)
			{	returnValue = 1;}
	return returnValue;
}
//	  01 d0 00 00 00 e9 99
//	  0123456789ABCDEF
//                    1111
//	                  0123
// `3h == 19 dec

int doesThisStringInclued19goodCharacters (int startingHere, char buffer[1024])
{	int returnValue = 1;
	for (int plusThis = 0;plusThis<10; plusThis ++)
	{	if (isThisCharacterHex(buffer[startingHere+plusThis]) != 1)
			{	returnValue = returnValue *1;	}
		else
			{	returnValue = 0;	}
	}

}
int findPeriod (char buffer[1024])
{	int i;
	int returnValue = 0;
	
	while ((i<1024) && (returnValue == 0))
	{	if (buffer[i] == '.')
				returnValue = i;
		else
			i ++;
	}
	return returnValue;
}
struct configureSTRINGreturnTYPE
{	long int rotations;
	unsigned int latestWHEELevent;
};

struct configureSTRINGreturnTYPE configureSTRING(const char* originalSTRING ) //	char* velocitySTRING
{		struct configureSTRINGreturnTYPE returnVALUE;
		int originalLENGTH = strlen(originalSTRING);
		char* configureSTRING = (char*)malloc(originalLENGTH+1);
		char* velocitySTRING  = (char*)malloc(4+1);
		//velocitySTRING =  (char*)malloc(4+1);
		//int count = 7;
		long sumTOTAL = 0;
		int offset = 0;
		/* string =>   01 37 00 00 00 0d 93                             .7.....         
configureSTRING 37000000
string =>   01 38 00 00 00 9b 95                             .8.....         
configureSTRING 38000000
string =>   01 3a 00 00 00 dd 9a                             .:.....         
configureSTRING 3a000000
string =>   01 3b 00 00 00 93 9d                             .;.....         
configureSTRING 3b000000
string =>   01 3c 00 00 00 59 a0                             .<...Y.         
configureSTRING 3c000000
string =>   01 3e 00 00 00 13 a6                             .>.....         
configureSTRING 3e000000
                    1         2
            0123456789012345678901
string =>   01 3f 00 00 00 0b a9                             .?.....         
configureSTRING 3f000000
*/
		//printf("%s   ",originalSTRING);
		for (int i = 0; i<originalLENGTH; i++)
			{	switch (i)
				{	case 0 ... 2:
						break;
					case 3:
						configureSTRING[6] = originalSTRING[i+offset];
						break;
					case 4:
						configureSTRING[7] = originalSTRING[i+offset];
						break;
					case 5:
						break;
					case 6:
						configureSTRING[4] = originalSTRING[i+offset];
						break;
					case 7:
						configureSTRING[5] = originalSTRING[i+offset];
						break;
					case 8:
						break;
					case 9:
						configureSTRING[2] = originalSTRING[i+offset];
						break;
					case 10:
						configureSTRING[3] = originalSTRING[i+offset];
						break;
					case 11:
						break;
					case 12:
						configureSTRING[0] = originalSTRING[i+offset];
						break;
					case 13:
						configureSTRING[1] = originalSTRING[i+offset];
						break;
					case 14:
						break;
					case 15:
						velocitySTRING[2] = originalSTRING[i+offset];
						break;
					case 16:
						velocitySTRING[3] = originalSTRING[i+offset];
						break;	
					case 17:
						break;
					case 18:
						velocitySTRING[0] = originalSTRING[i+offset];
						break;
					case 19:
						velocitySTRING[1] = originalSTRING[i+offset];
						break; 
					default:
						break;
				}
			}
		long int decimalValue;
		decimalValue = strtol(configureSTRING, NULL, 16);
		//	printf("configureSTRING %s decimal %ld\n",configureSTRING,decimalValue);
		//	printf("velocitySTRING %s\n",velocitySTRING);
		returnVALUE.rotations = decimalValue;
		returnVALUE.latestWHEELevent = strtol(velocitySTRING, NULL, 16);
		return returnVALUE;
}

struct speedPARAMETERS
{	float gradient;
	float speedDELTA;
	float maximumSPEED;
	float wheelCIRCUMFERENCE;
	int latestTHROTTLEvalue;
};
struct speedPARAMETERS parametersINuse;
// below from google: "c programming" "remove substring from string"
char *strremove(char *str, const char *sub) 
{	size_t len = strlen(sub);
	if (len > 0) 
	    {	char *p = str;
		while ((p = strstr(p, sub)) != NULL) 
		    {	memmove(p, p + len, strlen(p + len) + 1);
		    }
	    }
    return str;
}

int main() 
{	char achar;
	int stringShown = 0; //was zero
    FILE *fp;
    FILE *parameterSTORAGEfile;
    char buffer[1024];
    char bufferTWO[1024];
    char bufferSegment[1024];
    char* velocitySTRING;
    velocitySTRING =  (char*)malloc(4+1);
    int firstWHEELeventRECORDED  = 0; //after the first wheel evebt this wii change to one
    unsigned int wheelEVENTprevious = 0;
    unsigned int wheelEVENTcurrent = 0;
    unsigned int deltaWHEELevent;
    int currentWHEELeventRECORDED = 0;
    struct configureSTRINGreturnTYPE returnedFROMfunction;
    int previousWHEELdataRECORDED = 0;
    double wheel_circumference =(2096); // mm add this to ehancedSPEEDparameters file
    long int rotationsREPORTED;
    long int rotationPREVIOUS;
    double currentSPEED;
    double miles_per_kilometer= (0.621371);
    double approximateSPEED;
    char fileSTRING[128];
    char MACaddressSTRING[128];
    char processSTRING[128];
    char scriptNAME[128];
    double distanceTRAVELLED;
    double speedSHOULDbe;
    int throttlePOSITION;
    int throttleHIGHlimit = 0; //was 32767
    int throttleLOWlimit = 0;
    
    //double miles_per_kilometer= (0.621371); //added
    long int throttleMAX = 32767;
    long int throttleRANGE = 32767*2;
    int lastTROTTLEposition = 32767;
    //long int wheel_event_previous; //added to store consecutive wheel events
    //long int wheel_event_current; //added to store consecutive wheel events	
    //double miles_per_kilometer= (0.621371); //added
    int joystick_node = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if (joystick_node<0)
	{	perror("open /dev/uninpu");
		return 1;	}
    ioctl(joystick_node, UI_SET_EVBIT, EV_ABS);
    setup_abs(joystick_node,ABS_X,-32767,32767);
    struct uinput_setup setup =
    {
		.name = "myBIKEcanFLY",
		.id  =
		{
			.bustype = BUS_USB,
			.vendor =  0x3,
			.product = 0x3,
			.version = 2,
		}
	};
	if (ioctl(joystick_node, UI_DEV_SETUP, &setup))
    {
      perror("UI_DEV_SETUP");
      return 1;
    }
  
	if (ioctl(joystick_node, UI_DEV_CREATE))
    {
      perror("UI_DEV_CREATE");
      return 1;
    }
    
    parameterSTORAGEfile = fopen("speed_parameters_version_zero.txt","r");
    //fscanf(parameterSTORAGEfile,"%s",fileSTRING); //#my sensor mac address
    for (int i = 0; i<1; i++)
	{	fgets(fileSTRING,128,parameterSTORAGEfile);	}
    fgets(MACaddressSTRING,18,parameterSTORAGEfile);
    //	D5:1A:DE:86:93:E9
    //	01234567890123456789
    //sprintf("/0",&MACaddressSTRING[17]);
    
    printf("MACaddres is %s\n", MACaddressSTRING);
    
    fgets(fileSTRING,128,parameterSTORAGEfile); //#my speed gradient {mph/mi}
    fgets(fileSTRING,128,parameterSTORAGEfile);
    fgets(fileSTRING,128,parameterSTORAGEfile);
    printf("fileSTRING>> %s<",fileSTRING);
    
    sscanf(fileSTRING,"%f\n",&parametersINuse.gradient);
    printf("speed gradient %f\n",parametersINuse.gradient);
    
    fgets(fileSTRING,128,parameterSTORAGEfile); //#my speed delta {mph}
    fgets(fileSTRING,128,parameterSTORAGEfile);
    sscanf(fileSTRING,"%f\n",&parametersINuse.speedDELTA);
    printf("speed delta    %f\n",parametersINuse.speedDELTA);
    
    fgets(fileSTRING,128,parameterSTORAGEfile); //#my maximum speed {mph}
    fgets(fileSTRING,128,parameterSTORAGEfile);
    sscanf(fileSTRING,"%f\n",&parametersINuse.maximumSPEED);
    printf("maximum        %f\n",parametersINuse.maximumSPEED);
    
    fgets(fileSTRING,128,parameterSTORAGEfile); //#latest throttle value
    fgets(fileSTRING,128,parameterSTORAGEfile); //3276
    sscanf(fileSTRING,"%d\n",&parametersINuse.latestTHROTTLEvalue);
    printf(fileSTRING,"lates throttle %d\n",parametersINuse.latestTHROTTLEvalue);
    fgets(fileSTRING,128,parameterSTORAGEfile); //#latest throttle value
    fgets(fileSTRING,128,parameterSTORAGEfile); //3276
    fgets(fileSTRING,128,parameterSTORAGEfile); //#my preferred unit of length {miles or km}
    fgets(fileSTRING,128,parameterSTORAGEfile); //mi
    fgets(fileSTRING,128,parameterSTORAGEfile); //#my wheel circumference in mm
    fgets(fileSTRING,128,parameterSTORAGEfile); //2096
    printf("ff %s",fileSTRING);
    sscanf(fileSTRING,"%f\n",&parametersINuse.wheelCIRCUMFERENCE);
    printf("wheel circumference        %f\n",parametersINuse.wheelCIRCUMFERENCE);
    
    fclose(parameterSTORAGEfile);
    //fp = popen("./expect_script_forty D5:1A:DE:86:93:E9", "r"); //add MAC address to ehancedSPEEDparameters file
    // usr/bin/gatttool -b D5:1A:DE:86:93:E9 -t random --char-write-req --handle=0x0025 --value=0100 --listen
    sprintf(scriptNAME,"gatttool -b %s -t random --char-write-req --handle=0x0025 --value=0100 --listen", MACaddressSTRING);
    printf("command to be sent:%s\n",scriptNAME);
    fp = popen(scriptNAME,"r");
    
    int error_ioctl =ioctl(joystick_node, UI_DEV_SETUP, &setup);
	
    
    memset(&ev,0,sizeof ev);
    ev[0].type = EV_ABS;
    ev[0].code = ABS_X; //was ABS_X; //returned to ABS_X;
    ev[0].value = 32767; // last_throttle_value; 
    //ev[1].type = EV_SYN;
    //ev[1].code = SYN_REPORT;
    //ev[1].value = 0;
    /*
    	//band aid below
	if (fabs(speed_should_be) <= 0.1) //then
		{speed_should_be= spd_parameters.last_throttle_value;\
			printf("Joystick forced to be zero\n");}
			
	//band aid above
	* */
    fp = popen(scriptNAME, "r"); //add MAC address to ehancedSPEEDparameters file
    if (fp == NULL) 
    {	perror("popen failed");
	return 1;
    }
    
    
    while (fgets(buffer, sizeof(buffer), fp) != NULL)
    {   int reportSegment = 0;
		char subString[] = "Notification handle = 0x0024 value: ";
		char *result;
		result = strstr(buffer, subString);
		//printf("%s",buffer);
		if (result != NULL)
		    {	char* whatREMAINS = strremove(buffer,subString);
			int stringLENGTH = strlen(buffer);
			if (stringLENGTH > 0 && buffer[stringLENGTH - 1] == '\n')
			{	buffer[stringLENGTH-2] = '\0'; // as -1
			}
			//printf("whatREMAINS<%s>\n",buffer);
			returnedFROMfunction = configureSTRING(buffer);
			printf("	distance %5lf ",returnedFROMfunction.rotations*parametersINuse.wheelCIRCUMFERENCE*miles_per_kilometer/1000.0/1000.0); //firstWHEELeventRECORDED
			if (firstWHEELeventRECORDED == 0)
			    {	firstWHEELeventRECORDED = 1;
				printf("\n");
				wheelEVENTprevious = returnedFROMfunction.latestWHEELevent;
				rotationPREVIOUS = returnedFROMfunction.rotations;
				distanceTRAVELLED = returnedFROMfunction.rotations*parametersINuse.wheelCIRCUMFERENCE/1000/1000*miles_per_kilometer;
				fflush(stdout);
			    }
			else
			    {	//printf("wheel event: %5ld \n",returnedFROMfunction.latestWHEELevent);	
				wheelEVENTcurrent = returnedFROMfunction.latestWHEELevent;
				if (wheelEVENTcurrent<wheelEVENTprevious)
				    deltaWHEELevent = wheelEVENTprevious - wheelEVENTcurrent;
				else
				    deltaWHEELevent = wheelEVENTcurrent-wheelEVENTprevious;	
				
				approximateSPEED = (returnedFROMfunction.rotations - rotationPREVIOUS)*parametersINuse.wheelCIRCUMFERENCE/1000/1000*3600*miles_per_kilometer;
				if (deltaWHEELevent != 0)
					currentSPEED = approximateSPEED*1024/deltaWHEELevent;
				else
					currentSPEED = 0;
					
				distanceTRAVELLED = returnedFROMfunction.rotations*parametersINuse.wheelCIRCUMFERENCE/1000/1000*miles_per_kilometer;
				//	speed_should_be = -exp(-distance_traveled/spd_parameters.speed_gradient)*spd_parameters.speed_delta+spd_parameters.top_speed; //was exp_todd why I don't think was working 
					speedSHOULDbe   = -exp(-distanceTRAVELLED/parametersINuse.gradient)*parametersINuse.speedDELTA+parametersINuse.maximumSPEED;
				printf("speed = %8f speedSHOULDbe = %8f",currentSPEED,speedSHOULDbe);
				rotationPREVIOUS = returnedFROMfunction.rotations;
				wheelEVENTprevious = returnedFROMfunction.latestWHEELevent;
				int joystickVALUE = floor(-currentSPEED/speedSHOULDbe*65526+32768);
				
				
				//speed_should_be =  floor(-current_speed/speed_should_be*65536+32768);
				//ev[0].value = (int)speed_should_be;
				/* 
					if (speed_should_be>throttleHIGHlimit)
						{	speed_should_be = throttleHIGHlimit;
							speed_should_be = fmin(speed_should_be , throttleHIGHlimit);
							throttleHIGHlimit=fmin(throttleHIGHlimit+2047,32767);	} //Could not idle
					else
							throttleHIGHlimit = 0; //to allow muliple attempts to idle throttle
		
					if (speed_should_be < -32767)
						speed_should_be = -32767; 
				//	band aid below
					if (fabs(speed_should_be) <= 0.1) //then
							{	speed_should_be= spd_parameters.last_throttle_value;\
								printf("Joystick forced to be zero\n");	}
				//band aid above*/
				
				if (joystickVALUE>throttleHIGHlimit)
					{	//joystickVALUE = throttleHIGHlimit;
						joystickVALUE = fmin(joystickVALUE,throttleHIGHlimit);
						throttleHIGHlimit = fmin(throttleHIGHlimit+2047,32766);
						printf("joystickVALUE = %d\n",(int)(joystickVALUE));
					}
				else
					{	throttleHIGHlimit = 0;
						printf("joystickVALUE = %d\n",(int)(joystickVALUE));}
						
				//	band aid below
				if (fabs(joystickVALUE) <= 0.1)
					{	joystickVALUE = parametersINuse.latestTHROTTLEvalue;	}
				//	band aid abov	
							
				  for (int updateTIMES = 0; updateTIMES <=499; updateTIMES++)
					{  
						ev[0].value = (int)joystickVALUE;
						write( joystick_node, &ev, sizeof ev);	} //was this line missing?? 
				fflush(stdout);
			    }
				
		    }		
    }	
    pclose(fp);
    return 0;
}
static void setup_abs(int fd, unsigned chan, int min, int max)
{
  if (ioctl(fd, UI_SET_ABSBIT, chan))
    perror("UI_SET_ABSBIT");
  
  struct uinput_abs_setup s =
    {
     .code = chan,
     .absinfo = { .minimum = min,  .maximum = max },
    };

  if (ioctl(fd, UI_ABS_SETUP, &s))
    perror("UI_ABS_SETUP");
}
