# google search makefile multiple executables
#https://www.reddit.com/r/cprogramming/comments/t8tyrn/how_to_create_a_makefile_to_compile_two_programs/
CC = gcc
CFLAGS =
LDFLAGS=-lm -lrt

all: createAjoystickSOmyBIKEcanFLY write_tire_size_to_speed_parameter write_MAC_address_to_speed_parameter executeDIALOGtoCHOOSEtire

createAjoystickSOmyBIKEcanFLY.o: createAjoystickSOmyBIKEcanFLY.c
	$(CC) $(CFLAGS) -c $< -o $@

write_tire_size_to_speed_parameter.o: write_tire_size_to_speed_parameter.c
	$(CC) $(CFLAGS) -c $< -o $@

write_MAC_address_to_speed_parameter.o: write_MAC_address_to_speed_parameter.c
	$(CC) $(CFLAGS) -c $< -o $@

createAjoystickSOmyBIKEcanFLY: createAjoystickSOmyBIKEcanFLY.o
	$(CC) $^ -o $@  $(LDFLAGS)

write_tire_size_to_speed_parameter: write_tire_size_to_speed_parameter.o
	$(CC) $^ -o $@  $(LDFLAGS)

write_MAC_address_to_speed_parameter: write_MAC_address_to_speed_parameter.o
	$(CC) $^ -o $@  $(LDFLAGS)
	
executeDIALOGtoCHOOSEtire: executeDIALOGtoCHOOSEtire.o
	$(CC) $^ -o $@  $(LDFLAGS)
	
clean:
	rm -f *.o createAjoystickSOmyBIKEcanFLY write_tire_size_to_speed_parameter write_MAC_address_to_speed_parameter

#SRCS = write_tire_size_to_speed_parameter.c
#OBJS = $(SRCS:.c=.o)

# Executables to build
#TARGET = write_tire_size_to_speed_parameter

#write_tire_size_to_speed_parameter_SOURCES = write_tire_size_to_speed_parameter.c

#write_tire_size_to_speed_parameter_OBJS = $(patsubst %.c, %.o, $(write_tire_size_to_speed_parameter_SOURCES))

# Default target: builds all executables
#.PHONY: all
	
#all: $(TARGET) 
#$(TARGET): $(OBJS)
#	$(CC) $(OBJS) ($LDFLAGS) -o $(TARGET)
#write_tire_size_to_speed_parameter: $(write_tire_size_to_speed_parameter_OBJS)
#	$(CC) $(CFLAGS) -o $@ $(LDFLAGS)
#%.o: %.c
#	$(CC) $(CFLAGS) -c $< -o $@	
#	
#clean:
#	rm -f $(OBJS) $(TARGET)
	#rm -f $(write_tire_size_to_speed_parameter_OBJS) $(EXECUTABLE)
