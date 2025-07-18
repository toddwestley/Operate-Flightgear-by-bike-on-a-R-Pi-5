/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010  Nokia Corporation
 *  Copyright (C) 2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * /./home/todd/gatt_four/bluez-5.49/attrib/gatttool -b E5:97:EE:07:27:F2 -t random --char-write-req --handle=0x0025 --value=0100 --listen
 * from ~/gatt_four/bluez-5.49  make
 */
//https://github.com/GrantEdwards/uinput-joystick-demo/blob/master/uinput-demo.c
// https://github.com/GrantEdwards/uinput-joystick-demo/blob/master/uinput-demo.c
#ifdef HAVE_CONFIG_H

#include <config.h>
#endif

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include <glib.h>

#include "lib/bluetooth.h"
#include "lib/hci.h"
#include "lib/hci_lib.h"
#include "lib/sdp.h"
#include "lib/uuid.h"

#include "src/shared/util.h"
#include "att.h"
#include "btio/btio.h"
#include "gattrib.h"
#include "gatt.h"
#include "gatttool.h"
#include <stdio.h>    //added															found /usr/include/stdio.h
#include <string.h>   // addded															found /usr/include/string.h
//#include "/home/todd/ANT/gatt_one/bluez-5.18/attrib/trig_functions.h" // added		
//#include "/home/toddwestley/gatt_todd/trig_functions.h" // replaced above
#include "trig_functions.h"
//#include <math.h>
#include <linux/uinput.h>  // added                                                     found /usr/include/linux/uinput.h
//#include <fcntl.h>  // added and then deleted why??															found /usr/include/fcntl.h
#include <linux/fcntl.h> //correct added												found /usr/include/linux/fcntl.h
#include <unistd.h>  // added															found /usr/include/unistd.h
#include <errno.h> // added for error handling											founf /usr/include/errno.h
#include <stdlib.h> 	// added
#include <math.h>   	//ADDED
#include <linux/uinput.h> //added so uinput sould be -- sudo may still be requied
double wheel_circumference =(2096); // mm //added
double miles_per_kilometer= (0.621371); //added
int times_through = 0; // added to capture last wheel event
long int wheel_event_previous; //added to store consecutive wheel events
long int wheel_event_current; //added to store consecutive wheel events
double previous_rotations; //added to store rotations
double current_rotations; //added to store rotations
double distance_traveled; //added to store distance travelled
double current_speed; //added to store curremt speed
double delta_wheel_event; //added to store change in wheel event times
double approximate_speed; //added to store approximate speed
int quit_iteratiom = 0; // added to forse quit
int quit_time =4800;// was int quit_time = 380 thn 498;
int updateTIMES = 0;
int throttleHIGHlimit = 0;

struct speed_parameters { //added to store speed parameters
	float speed_gradient; //added to store speed parameters
	float speed_delta;    //added to store speed parameters
	float top_speed;      //added to store speed parameters
	int last_throttle_value;     //just added
}  spd_parameters;        //added to store speed parameters
struct speed_parameters param_one;  //added to store speed parameters
FILE *parameter_storage_file;       // added to get speed parameters
int joystick_node; // fd  on https://github.com/GrantEdwards/uinput-joystick-demo/blob/master/uinput-demo.c                //added integer so joystick node can be opened see https://unix.stackexchange.com/questions/241173/how-are-dev-linux-files-created REQUIRES sudo
//FILE *jostick_nodel;
//int joystick_node; // * added?? delet so only get text ouput
double speed_should_be;             // added
struct input_event ev[1];           // added inpu event structure WGY IS THIS TWO??
int maxthrrottleDRop = 4095;

static void setup_abs(int joystick_node, unsigned chan, int min, int max); //added
//FILE *rc; // from https://01.org/linuxgraphics/gfx-docs/drm/input/uinput.html
long int version; //added guess from above
static char *opt_src = NULL;
static char *opt_dst = NULL;
static char *opt_dst_type = NULL;
static char *opt_value = NULL;
static char *opt_sec_level = NULL;
static bt_uuid_t *opt_uuid = NULL;
static int opt_start = 0x0001;
static int opt_end = 0xffff;
static int opt_handle = -1;
static int opt_mtu = 0;
static int opt_psm = 0;
static gboolean opt_primary = FALSE;
static gboolean opt_characteristics = FALSE;
static gboolean opt_char_read = FALSE;
static gboolean opt_listen = FALSE;
static gboolean opt_char_desc = FALSE;
static gboolean opt_char_write = FALSE;
static gboolean opt_char_write_req = FALSE;
static gboolean opt_interactive = FALSE;
static GMainLoop *event_loop;
static gboolean got_error = FALSE;
static GSourceFunc operation;
struct uinput_setup setup_two; // added


struct characteristic_data {
	GAttrib *attrib;
	uint16_t start;
	uint16_t end;
};

static void events_handler(const uint8_t *pdu, uint16_t len, gpointer user_data)
{
	GAttrib *attrib = user_data;
	uint8_t *opdu;
	uint16_t handle, i, olen = 0;
	size_t plen;
    
	handle = get_le16(&pdu[1]);

	switch (pdu[0]) {
	case ATT_OP_HANDLE_NOTIFY:
		//g_print("Notification handle = 0x%04x value: ", handle);  //commented
		break;
	case ATT_OP_HANDLE_IND:
		g_print("Indication   handle = 0x%04x value: ", handle);
		break;
	default:
		g_print("Invalid opcode\n");
		return;
	}
    quit_iteratiom = quit_iteratiom+1;
    printf(" it=> %2i", quit_time-quit_iteratiom);
    printf("    rotations = ");     //added for debugging //was g_print
    printf("%i",pdu[4]+pdu[5]*256); //added for debugging
    printf("    event time = ");    //added for debugging
    printf("%6i",pdu[8]+pdu[9]*256); //added for debugging
	if (quit_iteratiom >= quit_time)
	{
		printf("QUIT NOW!\n");
		//quit_iteratiom =(float) 7/ (7-7);  //force divide y zero!!
		goto done;
		g_main_loop_quit(event_loop);
	}
	//addition begins here
	distance_traveled = (pdu[4]+pdu[5]*256)*wheel_circumference/1000/1000*miles_per_kilometer;
	if (times_through <= 0)
		{	
			//g_print("debug!!\n");
			previous_rotations = pdu[4]+pdu[5]*256;
			wheel_event_previous = pdu[8]+pdu[9]*256;
		}
	if (times_through>0)
		{	current_rotations = pdu[4]+pdu[5]*256;
			wheel_event_current = pdu[8]+pdu[9]*256;
			

			if (wheel_event_current > wheel_event_previous)
				{	delta_wheel_event = wheel_event_current - wheel_event_previous;
				}
			else
				{	delta_wheel_event = 65536+ wheel_event_current - wheel_event_previous;
				}

			approximate_speed = (current_rotations-previous_rotations)*wheel_circumference/1000/1000*3600*miles_per_kilometer;
			current_speed = approximate_speed* 1024/delta_wheel_event;
			speed_should_be = -exp(-distance_traveled/spd_parameters.speed_gradient)*spd_parameters.speed_delta+spd_parameters.top_speed; //was exp_todd why I don't think was working
			

			
			speed_should_be =  floor(-current_speed/speed_should_be*65536+32768);
		}
	times_through = times_through +1;	
	wheel_event_previous = wheel_event_current;
	previous_rotations = current_rotations;
	g_print(" dist = ");
	g_print("%f4",distance_traveled);
	g_print(" speed = ");
	g_print("%f4",current_speed);
	g_print(" sent to joystick = ");
	
	if (speed_should_be>throttleHIGHlimit)
		{speed_should_be = throttleHIGHlimit;
		 speed_should_be = fmin(speed_should_be , throttleHIGHlimit);
		 throttleHIGHlimit=fmin(throttleHIGHlimit+2047,32767);	} //Could not idle
	else
		throttleHIGHlimit = 0; //to allow muliple attempts to idle throttle
		
	if (speed_should_be < -32767)
		speed_should_be = -32767; 
	//band aid below
	if (fabs(speed_should_be) <= 0.1) //then
		{speed_should_be= spd_parameters.last_throttle_value;\
			printf("Joystick forced to be zero\n");}
			
	//band aid above
		
	g_print("%i",(int)speed_should_be); // should this be %i???  speed shoulb be is a double
	previous_rotations= current_rotations;
	memset(&ev,0,sizeof ev); //just add 2:39 2019_10_30
	ev[0].type = EV_ABS;
	ev[0].code = ABS_X; //was ev[0].code = ABS_THROTTLE; restored to ABS_X
	ev[0].value = (int)speed_should_be;
	//ev[1].type = EV_SYN;
	//ev[1].code = SYN_REPORT;
	//ev[1].value = 0;

    //ev[0].value = (int)(floor(speed_should_be)); // throttle set the value in the loop! // how do you convert mileage into speed mabe this b
    for (updateTIMES = 0; updateTIMES <=499; updateTIMES++)
		{   //=FLOOR($B$1-(($B$1-$B$2)*B4/499))
			// B1 --> THROTTLE WAS
			// n2 -> THRITTLR WILL BE
			//ev[0].value =(int)(spd_parameters.last_throttle_value+ (spd_parameters.last_throttle_value - speed_should_be)+updateTIMES/499);
			ev[0].value = (int)speed_should_be;
			write( joystick_node, &ev, sizeof ev);	} //was this line missing??
    /*
    parameter_storage_file = fopen("/home/toddwestley/Downloads/bluez-5.66/bluez-5.66/attrib/speed_parameters.txt","w");
    fprintf(parameter_storage_file,"%f\n",spd_parameters.speed_gradient);
    fprintf(parameter_storage_file,"%f\n",spd_parameters.speed_delta);
    fprintf(parameter_storage_file,"%f\n",spd_parameters.top_speed);
    fprintf(parameter_storage_file,"%i\n",(int)speed_should_be);
    fclose(parameter_storage_file);
    */
    
	g_print("\n");	
	if (pdu[0] == ATT_OP_HANDLE_NOTIFY)
		return;

	opdu = g_attrib_get_buffer(attrib, &plen);
	olen = enc_confirmation(opdu, plen);

	if (olen > 0)
		g_attrib_send(attrib, 0, opdu, olen, NULL, NULL, NULL);
	done:
		{	printf("/n/nEntered QUIT\n\n");
			g_main_loop_quit(event_loop);
			}	// I hope this forces quit
}

static gboolean listen_start(gpointer user_data)
{
	GAttrib *attrib = user_data;

	g_attrib_register(attrib, ATT_OP_HANDLE_NOTIFY, GATTRIB_ALL_HANDLES,
						events_handler, attrib, NULL);
	g_attrib_register(attrib, ATT_OP_HANDLE_IND, GATTRIB_ALL_HANDLES,
						events_handler, attrib, NULL);

	return FALSE;
}

static void connect_cb(GIOChannel *io, GError *err, gpointer user_data)
{
	GAttrib *attrib;
	uint16_t mtu;
	uint16_t cid;
	GError *gerr = NULL;

	if (err) {
		g_printerr("%s\n", err->message);
		got_error = TRUE;
		g_main_loop_quit(event_loop);
	}

	bt_io_get(io, &gerr, BT_IO_OPT_IMTU, &mtu,
				BT_IO_OPT_CID, &cid, BT_IO_OPT_INVALID);

	if (gerr) {
		g_printerr("Can't detect MTU, using default: %s",
								gerr->message);
		g_error_free(gerr);
		mtu = ATT_DEFAULT_LE_MTU;
	}

	if (cid == ATT_CID)
		mtu = ATT_DEFAULT_LE_MTU;

	attrib = g_attrib_new(io, mtu, false);

	if (opt_listen)
		g_idle_add(listen_start, attrib);

	operation(attrib);
}

static void primary_all_cb(uint8_t status, GSList *services, void *user_data)
{
	GSList *l;

	if (status) {
		g_printerr("Discover all primary services failed: %s\n",
							att_ecode2str(status));
		goto done;
	}

	for (l = services; l; l = l->next) {
		struct gatt_primary *prim = l->data;
		g_print("attr handle = 0x%04x, end grp handle = 0x%04x "
			"uuid: %s\n", prim->range.start, prim->range.end, prim->uuid);
	}

done:
	g_main_loop_quit(event_loop);
}

static void primary_by_uuid_cb(uint8_t status, GSList *ranges, void *user_data)
{
	GSList *l;

	if (status != 0) {
		g_printerr("Discover primary services by UUID failed: %s\n",
							att_ecode2str(status));
		goto done;
	}

	for (l = ranges; l; l = l->next) {
		struct att_range *range = l->data;
		g_print("Starting handle: %04x Ending handle: %04x\n",
						range->start, range->end);
	}

done:
	g_main_loop_quit(event_loop);
}

static gboolean primary(gpointer user_data)
{
	GAttrib *attrib = user_data;

	if (opt_uuid)
		gatt_discover_primary(attrib, opt_uuid, primary_by_uuid_cb,
									NULL);
	else
		gatt_discover_primary(attrib, NULL, primary_all_cb, NULL);

	return FALSE;
}

static void char_discovered_cb(uint8_t status, GSList *characteristics,
								void *user_data)
{
	GSList *l;

	if (status) {
		g_printerr("Discover all characteristics failed: %s\n",
							att_ecode2str(status));
		goto done;
	}

	for (l = characteristics; l; l = l->next) {
		struct gatt_char *chars = l->data;

		g_print("handle = 0x%04x, char properties = 0x%02x, char value "
			"handle = 0x%04x, uuid = %s\n", chars->handle,
			chars->properties, chars->value_handle, chars->uuid);
	}

done:
	g_main_loop_quit(event_loop);
}

static gboolean characteristics(gpointer user_data)
{
	GAttrib *attrib = user_data;

	gatt_discover_char(attrib, opt_start, opt_end, opt_uuid,
						char_discovered_cb, NULL);

	return FALSE;
}

static void char_read_cb(guint8 status, const guint8 *pdu, guint16 plen,
							gpointer user_data)
{
	uint8_t value[plen];
	ssize_t vlen;
	int i;

	if (status != 0) {
		g_printerr("Characteristic value/descriptor read failed: %s\n",
							att_ecode2str(status));
		goto done;
	}

	vlen = dec_read_resp(pdu, plen, value, sizeof(value));
	if (vlen < 0) {
		g_printerr("Protocol error\n");
		goto done;
	}
	g_print("Characteristic value/descriptor: ");
	for (i = 0; i < vlen; i++)
		g_print("%02x ", value[i]);
	g_print("\n");

done:
	if (!opt_listen)
		g_main_loop_quit(event_loop);
}

static void char_read_by_uuid_cb(guint8 status, const guint8 *pdu,
					guint16 plen, gpointer user_data)
{
	struct att_data_list *list;
	int i;

	if (status != 0) {
		g_printerr("Read characteristics by UUID failed: %s\n",
							att_ecode2str(status));
		goto done;
	}

	list = dec_read_by_type_resp(pdu, plen);
	if (list == NULL)
		goto done;

	for (i = 0; i < list->num; i++) {
		uint8_t *value = list->data[i];
		int j;

		g_print("handle: 0x%04x \t value: ", get_le16(value));
		value += 2;
		for (j = 0; j < list->len - 2; j++, value++)
			g_print("%02x ", *value);
		g_print("\n");
	}

	att_data_list_free(list);

done:
	g_main_loop_quit(event_loop);
}

static gboolean characteristics_read(gpointer user_data)
{
	GAttrib *attrib = user_data;

	if (opt_uuid != NULL) {

		gatt_read_char_by_uuid(attrib, opt_start, opt_end, opt_uuid,
						char_read_by_uuid_cb, NULL);

		return FALSE;
	}

	if (opt_handle <= 0) {
		g_printerr("A valid handle is required\n");
		g_main_loop_quit(event_loop);
		return FALSE;
	}

	gatt_read_char(attrib, opt_handle, char_read_cb, attrib);

	return FALSE;
}

static void mainloop_quit(gpointer user_data)
{
	uint8_t *value = user_data;

	g_free(value);
	g_main_loop_quit(event_loop);
}

static gboolean characteristics_write(gpointer user_data)
{
	GAttrib *attrib = user_data;
	uint8_t *value;
	size_t len;

	if (opt_handle <= 0) {
		g_printerr("A valid handle is required\n");
		goto error;
	}

	if (opt_value == NULL || opt_value[0] == '\0') {
		g_printerr("A value is required\n");
		goto error;
	}

	len = gatt_attr_data_from_string(opt_value, &value);
	if (len == 0) {
		g_printerr("Invalid value\n");
		goto error;
	}

	gatt_write_cmd(attrib, opt_handle, value, len, mainloop_quit, value);

	g_free(value);
	return FALSE;

error:
	g_main_loop_quit(event_loop);
	return FALSE;
}

static void char_write_req_cb(guint8 status, const guint8 *pdu, guint16 plen,
							gpointer user_data)
{
	if (status != 0) {
		g_printerr("Characteristic Write Request failed: "
						"%s\n", att_ecode2str(status));
		goto done;
	}

	if (!dec_write_resp(pdu, plen) && !dec_exec_write_resp(pdu, plen)) {
		g_printerr("Protocol error\n");
		goto done;
	}

	g_print("Characteristic value was wrIItten successfully\n hello world\n");

done:
	if (!opt_listen)
		g_main_loop_quit(event_loop);
}

static gboolean characteristics_write_req(gpointer user_data)
{
	GAttrib *attrib = user_data;
	uint8_t *value;
	size_t len;

	if (opt_handle <= 0) {
		g_printerr("A valid handle is required\n");
		goto error;
	}

	if (opt_value == NULL || opt_value[0] == '\0') {
		g_printerr("A value is required\n");
		goto error;
	}

	len = gatt_attr_data_from_string(opt_value, &value);
	if (len == 0) {
		g_printerr("Invalid value\n");
		goto error;
	}

	gatt_write_char(attrib, opt_handle, value, len, char_write_req_cb,
									NULL);

	g_free(value);
	return FALSE;

error:
	g_main_loop_quit(event_loop);
	return FALSE;
}

static void char_desc_cb(uint8_t status, GSList *descriptors, void *user_data)
{
	GSList *l;

	if (status) {
		g_printerr("Discover descriptors failed: %s\n",
							att_ecode2str(status));
		return;
	}

	for (l = descriptors; l; l = l->next) {
		struct gatt_desc *desc = l->data;

		g_print("handlestatic void setup_abs(int fd, unsigned chan, int min, int max); = 0x%04x, uuid = %s\n", desc->handle,
								desc->uuid);
	}

	if (!opt_listen)
		g_main_loop_quit(event_loop);
}

static gboolean characteristics_desc(gpointer user_data)
{
	GAttrib *attrib = user_data;

	gatt_discover_desc(attrib, opt_start, opt_end, NULL, char_desc_cb,
									NULL);

	return FALSE;
}

static gboolean parse_uuid(const char *key, const char *value,
				gpointer user_data, GError **error)
{
	if (!value)
		return FALSE;

	opt_uuid = g_try_malloc(sizeof(bt_uuid_t));
	if (opt_uuid == NULL)
		return FALSE;

	if (bt_string_to_uuid(opt_uuid, value) < 0)
		return FALSE;

	return TRUE;
}

static GOptionEntry primary_char_options[] = {
	{ "start", 's' , 0, G_OPTION_ARG_INT, &opt_start,
		"Starting handle(optional)", "0x0001" },
	{ "end", 'e' , 0, G_OPTION_ARG_INT, &opt_end,
		"Ending handle(optional)", "0xffff" },
	{ "uuid", 'u', G_OPTION_FLAG_OPTIONAL_ARG, G_OPTION_ARG_CALLBACK,
		parse_uuid, "UUID16 or UUID128(optional)", "0x1801"},
	{ NULL },
};

static GOptionEntry char_rw_options[] = {
	{ "handle", 'a' , 0, G_OPTION_ARG_INT, &opt_handle,
		"Read/Write characteristic by handle(required)", "0x0001" },
	{ "value", 'n' , 0, G_OPTION_ARG_STRING, &opt_value,
		"Write characteristic value (required for write operation)",
		"0x0001" },
	{NULL},
};

static GOptionEntry gatt_options[] = {
	{ "primary", 0, 0, G_OPTION_ARG_NONE, &opt_primary,
		"Primary Service Discovery", NULL },
	{ "characteristics", 0, 0, G_OPTION_ARG_NONE, &opt_characteristics,
		"Characteristics Discovery", NULL },
	{ "char-read", 0, 0, G_OPTION_ARG_NONE, &opt_char_read,
		"Characteristics Value/Descriptor Read", NULL },
	{ "char-write", 0, 0, G_OPTION_ARG_NONE, &opt_char_write,
		"Characteristics Value Write Without Response (Write Command)",
		NULL },
	{ "char-write-req", 0, 0, G_OPTION_ARG_NONE, &opt_char_write_req,
		"Characteristics Value Write (Write Request)", NULL },
	{ "char-desc", 0, 0, G_OPTION_ARG_NONE, &opt_char_desc,
		"Characteristics Descriptor Discovery", NULL },
	{ "listen", 0, 0, G_OPTION_ARG_NONE, &opt_listen,
		"Listen for notifications and indications", NULL },
	{ "interactive", 'I', G_OPTION_FLAG_IN_MAIN, G_OPTION_ARG_NONE,
		&opt_interactive, "Use interactive mode", NULL },
	{ NULL },
};

static GOptionEntry options[] = {
	{ "adapter", 'i', 0, G_OPTION_ARG_STRING, &opt_src,
		"Specify local adapter interface", "hciX" },
	{ "device", 'b', 0, G_OPTION_ARG_STRING, &opt_dst,
		"Specify remote Bluetooth address", "MAC" },
	{ "addr-type", 't', 0, G_OPTION_ARG_STRING, &opt_dst_type,
		"Set LE address type. Default: public", "[public | random]"},
	{ "mtu", 'm', 0, G_OPTION_ARG_INT, &opt_mtu,
		"Specify the MTU size", "MTU" },
	{ "psm", 'p', 0, G_OPTION_ARG_INT, &opt_psm,
		"Specify the PSM for GATT/ATT over BR/EDR", "PSM" },
	{ "sec-level", 'l', 0, G_OPTION_ARG_STRING, &opt_sec_level,
		"Set security level. Default: low", "[low | medium | high]"},
	{ NULL },
};
static void setup_abs(int joystick_node, unsigned chan, int min, int max) //was int
{
  if (ioctl(joystick_node , UI_SET_ABSBIT, chan))
    perror("UI_SET_ABSBIT");

  struct uinput_abs_setup s =
    {
     .code = chan,
     //.absinfo = { .minimum = min,  .maximum = max },
     .absinfo = { .minimum = -32767,  .maximum = 32767 },
    };

  if (ioctl(joystick_node , UI_ABS_SETUP, &s))
    perror("UI_ABS_SETUP");
} //added setup_abs

int main(int argc, char *argv[])
{   //added start
	long int dummy_int;
	joystick_node = open("/dev/uinput", O_WRONLY | O_NONBLOCK);  //was open...did I mean popen
	//parameter_storage_file = fopen("/home/todd/gatt_four/bluez-5.49/attrib/speed_parameters.txt","r");
  // parameter_storage_file = fopen("/home/toddwestley/bluez-5.49/attrib/speed_parameters.txt","r");
	 parameter_storage_file = fopen("/home/toddwestley/Downloads/bluez-5.66/bluez-5.66/attrib/speed_parameters.txt","r");
	dummy_int = fscanf(parameter_storage_file,"%f\n",&spd_parameters.speed_gradient);
	dummy_int = fscanf(parameter_storage_file,"%f\n",&spd_parameters.speed_delta);
	dummy_int = fscanf(parameter_storage_file,"%f\n",&spd_parameters.top_speed);
	dummy_int = fscanf(parameter_storage_file,"%d\n",&spd_parameters.last_throttle_value);  //was %I!
	printf("dummy_int =? %i\n", dummy_int);
	
	//need to write and retain the speed
	fclose(parameter_storage_file);
	printf("speed gradient     => %5.3f \n",spd_parameters.speed_gradient);
	printf("speed speed delta  => %5.3f \n",spd_parameters.speed_delta);
	printf("top speed          => %5.3f \n",spd_parameters.top_speed);
	printf("last joystick value=> %i\n", spd_parameters.last_throttle_value);
	
	//joystick_node = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	//joystick_node = fopen("/dev/uinput",  O_WRONLY | O_NONBLOCK ); //why 
	  //joystick_node = fopen("/dev/uinput",  O_WRONLY | O_NONBLOCK ); //why 
	ioctl(joystick_node, UI_SET_EVBIT, EV_ABS); // enable analog absolute position handling 
	setup_abs(joystick_node, ABS_X,  -32767, 32767); //commented out as it cause an operation error // was ABS_THRROTTTLE
    struct uinput_setup setup =
    {
		.name = "Todd Westley's Joystick",
		.id  =
		{
			.bustype = BUS_USB,
			.vendor =  0x3,
			.product = 0x3,
			.version = 2,
		}
	};

	int error_ioctl =ioctl(joystick_node, UI_DEV_SETUP, &setup);
//                   ioctl(fd           , UI_DEV_SETUP, &setup))
	if (ioctl(joystick_node, UI_DEV_CREATE))
//     (ioctl(fd           , UI_DEV_CREATE))	
	{
		perror("UI_DEV_SETUP");
		return 1;
    }
	//if (ioctl(joystick_node, UI_DEV_CREATE))
    //{
	//	perror("UI_DEV_CREATE");
	//	return 1;
    //}
    memset(&ev,0,sizeof ev);
    ev[0].type = EV_ABS;
    ev[0].code = ABS_X; //was ABS_X; //returned to ABS_X;
    ev[0].value = param_one.last_throttle_value; // last_throttle_value; 
    //ev[1].type = EV_SYN;
    //ev[1].code = SYN_REPORT;
    //ev[1].value = 0;
    write( joystick_node, &ev, sizeof ev); // added this line
	GOptionContext *context;
	GOptionGroup *gatt_group, *params_group, *char_rw_group;
	GError *gerr = NULL;
	GIOChannel *chan;

	opt_dst_type = g_strdup("public");
	opt_sec_level = g_strdup("low");

	context = g_option_context_new(NULL);
	g_option_context_add_main_entries(context, options, NULL);

	/* GATT commands */
	gatt_group = g_option_group_new("gatt", "GATT commands",
					"Show all GATT commands", NULL, NULL);
	g_option_context_add_group(context, gatt_group);
	g_option_group_add_entries(gatt_group, gatt_options);

	/* Primary Services and Characteristics arguments */
	params_group = g_option_group_new("params",
			"Primary Services/Characteristics arguments",
			"Show all Primary Services/Characteristics arguments",
			NULL, NULL);
	g_option_context_add_group(context, params_group);
	g_option_group_add_entries(params_group, primary_char_options);

	/* Characteristics value/descriptor read/write arguments */
	char_rw_group = g_option_group_new("char-read-write",
		"Characteristics Value/Descriptor Read/Write arguments",
		"Show all Characteristics Value/Descriptor Read/Write "
		"arguments",
		NULL, NULL);
	g_option_context_add_group(context, char_rw_group);
	g_option_group_add_entries(char_rw_group, char_rw_options);

	if (!g_option_context_parse(context, &argc, &argv, &gerr)) {
		g_printerr("%s\n", gerr->message);
		g_clear_error(&gerr);
	}

	if (opt_interactive) {
		interactive(opt_src, opt_dst, opt_dst_type, opt_psm);
		goto done;
	}

	if (opt_primary)
		operation = primary;
	else if (opt_characteristics)
		operation = characteristics;
	else if (opt_char_read)
		operation = characteristics_read;
	else if (opt_char_write)
		operation = characteristics_write;
	else if (opt_char_write_req)
		operation = characteristics_write_req;
	else if (opt_char_desc)
		operation = characteristics_desc;
	else {
		char *help = g_option_context_get_help(context, TRUE, NULL);
		g_print("%s\n", help);
		g_free(help);
		got_error = TRUE;
		goto done;
	}

	if (opt_dst == NULL) {
		g_print("Remote Bluetooth address required\n");
		got_error = TRUE;
		goto done;
	}

	chan = gatt_connect(opt_src, opt_dst, opt_dst_type, opt_sec_level,
					opt_psm, opt_mtu, connect_cb, &gerr);
	if (chan == NULL) {
		g_printerr("%s\n", gerr->message);
		g_clear_error(&gerr);
		got_error = TRUE;
		goto done;
	}

	event_loop = g_main_loop_new(NULL, FALSE);

	g_main_loop_run(event_loop);

	g_main_loop_unref(event_loop);

done:
	g_option_context_free(context);
	g_free(opt_src);
	g_free(opt_dst);
	g_free(opt_uuid);
	g_free(opt_sec_level);

	if (got_error)
		exit(EXIT_FAILURE);
	else
		exit(EXIT_SUCCESS);
}
