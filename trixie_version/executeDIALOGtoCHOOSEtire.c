// from googe:popen "c programming" "execute script"
#include <stdio.h>
#include <stdlib.h> // Required for general utilities
#include <time.h>
int main()
{	FILE *fp;
	char output_buffer[1024];
    const char *script_command = "~/forFLIGHT/dialog_to_choose_tire_size"; 
    char choice;
    struct timespec theTIMEnow;
	struct timespec theTIMEthen;
	double elapsedTIME;
	/*
	printf("wait two seconds and press c\n");
    clock_gettime(CLOCK_MONOTONIC, &theTIMEthen);
    do
	{	clock_gettime(CLOCK_MONOTONIC, &theTIMEnow);
		elapsedTIME = (theTIMEnow.tv_sec - theTIMEthen.tv_sec)+ (theTIMEnow.tv_nsec - theTIMEthen.tv_nsec) / 1000000000.0;
		scanf(" %c", &choice);
	}
	while ((elapsedTIME < 2) || (choice != 'c')); */
    fp = popen(script_command, "r");
    if (fp == NULL) 
        {	perror("Failed to run command");
			exit(EXIT_FAILURE);	}
    pclose(fp);
    
	
    printf("Program terminated. Goodbye!\n");
    return 0;
}
