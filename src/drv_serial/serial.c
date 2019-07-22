// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2015, Joey Ferwerda.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Serial Driver */


#include <sys/stat.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "../openhmdi.h"
#include "string.h"


#define PORT "com5"


#ifndef WIN32
#include <termios.h>
#define BAUD_RATE B115200
#else // ! WIN32
#warning you are builing on/for windows, mind your step!
#include <windows.h>
#define BAUD_RATE 115200
#define O_NOCTTY 0
#define O_NONBLOCK 0
#endif // ! WIN32


typedef struct
{
	ohmd_device	 base;
	fusion		 sensor_fusion;
	int		 fd;
#ifdef BAUD_RATE
#ifndef WIN32
	struct termios	 settings;
#else // ! WIN32
	HANDLE		 fh;
	DCB		 settings;
#endif // ! WIN32
#endif // BAUD_RATE
	char		 buffer[256];
	int		 length;
} serial_priv;


static void update_device(ohmd_device* device)
{
	serial_priv	*priv = (serial_priv*)device;
	float		 w, x, y, z;
	int		 number;

	char		*next;


	if( priv->fd < 0 )
	{
		LOGE( "File descriptor for '%s' closed? %s", PORT, strerror(errno));
		return;
	}


	number= read( priv->fd, &priv->buffer[priv->length], sizeof(priv->buffer)-priv->length);
	if( number >= 0 )
	{
		priv->length+= number;
	}
	else if( errno != EAGAIN )
	{
		LOGE( "Can not read file '%s': %s", PORT, strerror(errno));
		return;
	}

	priv->buffer[priv->length]= '\0';

	next=strchr(priv->buffer,'\r');
	if( next == NULL )
		next=strchr(priv->buffer,'\n');
	if( next != NULL )
	{

		if( sscanf(priv->buffer,"%f,%f,%f,%f",&x,&y,&z,&w) == 4 )
		{
			priv->sensor_fusion.orient.w= w;
			priv->sensor_fusion.orient.x= x;
			priv->sensor_fusion.orient.y= y;
			priv->sensor_fusion.orient.z= z;
		}

//		next+= strspn( next, "\r\n");
		next+= strcspn( next, "-0.123456789");
		priv->length= strlen( next);
		for( number=0; number < priv->length; number++ )
			priv->buffer[number]= next[number];
//		memmove( &priv->buffer, &next, priv->length);
		priv->buffer[priv->length]= '\0';
	}
}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	serial_priv* priv = (serial_priv*)device;

	switch(type)
	{
		case OHMD_ROTATION_QUAT: {
				*(quatf*)out = priv->sensor_fusion.orient;
				break;
			}

		case OHMD_POSITION_VECTOR:
			out[0] = out[1] = out[2] = 0;
			break;

		case OHMD_DISTORTION_K:
			memset(out, 0, sizeof(float) * 6);
			break;

		default:
			ohmd_set_error(priv->base.ctx, "invalid type given to getf (%d)", type);
			return -1;
			break;
	}

	return 0;
}

static int setf(ohmd_device* device, ohmd_float_value type, const float* in)
{
	serial_priv* priv = (serial_priv*)device;

	switch(type){
		case OHMD_EXTERNAL_SENSOR_FUSION: {
				ofusion_update(&priv->sensor_fusion, *in, (vec3f*)(in + 1), (vec3f*)(in + 4), (vec3f*)(in + 7));
			}
			break;

		default:
			ohmd_set_error(priv->base.ctx, "invalid type given to setf (%d)", type);
			return -1;
			break;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	serial_priv* priv = (serial_priv*)device;


	LOGD("closing serial device");

#ifdef BAUD_RATE
#ifndef WIN32
	if( tcsetattr(priv->fd,TCSANOW,&priv->settings) < 0 )
	{
		LOGW( "Can not reset file attributes for '%s': %s", PORT, strerror(errno));
	}
#else // ! WIN32
	if( ! SetCommState(priv->fh,&priv->settings) )
	{
		LOGW( "Can not set serial attributes '%s': %s", PORT, strerror(errno));
	}
#endif // ! WIN32
#endif // BAUD_RATE

	if( priv->fd > 0 )
	{
		close( priv->fd);
		priv->fd=-1;
	}

	free(device);
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{

	serial_priv* priv = ohmd_alloc(driver->ctx, sizeof(serial_priv));
	if( ! priv )
		return NULL;

#ifndef WIN32

	priv->fd= open( desc->path, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_NDELAY);
	if( priv->fd < 0 )
	{
		LOGE( "Can not open file '%s': %s", desc->path, strerror(errno));
		free( priv);
		priv= NULL;
	}
	else
	{

// add flock() ?
// if( flock(priv->fd,LOCK_EX|LOCK_NB) < 0 ) perror();
#else // ! WIN32

	priv->fd= open( desc->path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if( priv->fd < 0 )
	{
		LOGE( "Can not open file '%s': %s", desc->path, strerror(errno));
		free( priv);
		priv= NULL;
	}
	else
	{

#endif // ! WIN32

#ifdef BAUD_RATE
#ifndef WIN32

		struct termios		 settings;


		if( tcgetattr(priv->fd,&priv->settings) < 0 )
		{
			LOGW( "Can not get serial attributes for '%s': %s", desc->path, strerror(errno));
		}
		else
		{

			settings= priv->settings;


			// convert break to null byte, no CR to NL translation,  no NL to CR translation, don't mark parity errors or breaks, no input parity check, don't strip high bit off, no XON/XOFF software flow control

			settings.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);


			// no CR to NL translation, no NL to CR-NL translation, no NL to CR translation, no column 0 CR suppression, no fill characters, no case mapping, no local output processing

			settings.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);
//			settings.c_oflag = 0;


			// echo off, echo newline off, canonical mode off, extended input processing off, signal chars off

			settings.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);


			// clear current char size mask, no parity checking, no output processing, one stop bit, force 8 bit input

			settings.c_cflag &= ~(CSIZE | CSTOPB | PARENB);
			settings.c_cflag |= CS8;

			// non-POSIX ways...
//			settings.c_cflag &= ~CBAUD;
//			settings.c_cflag |= BAUD_RATE;
//			settings.c_cflag &= ~CBAUD;
//			settings.c_cflag |= CBAUDEX | B0;

			if( cfsetispeed(&settings,BAUD_RATE) < 0 )
				LOGE( "Can not set incomming baudrate for '%s': %s", desc->path, strerror(errno));
			if( cfsetospeed( &settings, BAUD_RATE) < 0 )
				LOGW( "Can not set outgoing baudrate for '%s': %s", desc->path, strerror(errno));

			// block read() until this many bytes are received
			settings.c_cc[VMIN] = 0;
			// block read() until there are no bytes for this many tenths of a second
			settings.c_cc[VTIME] = 0;

			if( tcsetattr(priv->fd,TCSANOW,&settings) < 0 )
				LOGW( "Can not set serial attributes for '%s': %s", desc->path, strerror(errno));

			if( tcflush(priv->fd,TCOFLUSH) < 0 )
				LOGW( "Can not flush termios '%s': %s", desc->path, strerror(errno));
		}

#else // ! WIN32

		DCB			 settings;


		priv->fh= (HANDLE)_get_osfhandle( priv->fd);

		if( ! GetCommState(priv->fh,&priv->settings) )
		{
			LOGW( "Can not get serial attributes for '%s': %s", desc->path, strerror(errno));
		}
		else
		{

			settings= priv->settings;

			settings.Parity = NOPARITY;
			settings.BaudRate = BAUD_RATE;
			settings.ByteSize = 8;
			settings.StopBits = ONESTOPBIT;

			if( ! SetCommState(priv->fh,&settings) )
			{
			}
		}

#endif // ! WIN32
#endif // BAUD_RATE

		priv->length= 0;
		priv->buffer[priv->length]='\0';

		// Set default device settings
		ohmd_set_default_device_properties(&priv->base.properties);

		// Set device properties
		//TODO: Get information from external device using set_external_properties?
		//Using 'dummy' settings for now
		priv->base.properties.hsize = 0.149760f;
		priv->base.properties.vsize = 0.093600f;
		priv->base.properties.hres = 1280;
		priv->base.properties.vres = 800;
		priv->base.properties.lens_sep = 0.063500f;
		priv->base.properties.lens_vpos = 0.046800f;
		priv->base.properties.fov = DEG_TO_RAD(125.5144f);
		priv->base.properties.ratio = (1280.0f / 800.0f) / 2.0f;

		// calculate projection eye projection matrices from the device properties
		ohmd_calc_default_proj_matrices(&priv->base.properties);

		// set up device callbacks
		priv->base.update = update_device;
		priv->base.close = close_device;
		priv->base.getf = getf;
		priv->base.setf = setf;

		ofusion_init(&priv->sensor_fusion);

	}


	return (ohmd_device*)priv;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	ohmd_device_desc	*desc;


#ifndef WIN32

	struct stat		 sb;

	if( stat(PORT,&sb) >= 0 )
	{

		if( (sb.st_mode & S_IFMT) == S_IFCHR )
		{
			desc= &list->devices[list->num_devices++];

			strcpy( desc->driver, "OpenHMD Generic Serial Driver");
			strcpy( desc->vendor, "OpenHMD");
			strcpy( desc->product, "Serial Device");

			strcpy( desc->path, PORT);

			desc->device_class= OHMD_DEVICE_CLASS_HMD;
			desc->device_flags= OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

			desc->driver_ptr= driver;
		}

	}

#else // ! WIN32

	char lpTargetPath[256];

	if( QueryDosDevice( PORT, lpTargetPath, sizeof(lpTargetPath)) > 0 )
	{
		desc= &list->devices[list->num_devices++];

		strcpy( desc->driver, "OpenHMD Generic Serial Driver");
		strcpy( desc->vendor, "OpenHMD");
		strcpy( desc->product, "Serial Device");

		strcpy( desc->path, PORT);

		desc->device_class= OHMD_DEVICE_CLASS_HMD;
		desc->device_flags= OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

		desc->driver_ptr= driver;
	}

//	printf( "%s: %s\n", PORT, lpTargetPath);

#endif // ! WIN32

}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down serial driver");
	free(drv);
}

ohmd_driver* ohmd_create_serial_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));
	if( ! drv )
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;
	drv->ctx = ctx;

	return drv;
}

