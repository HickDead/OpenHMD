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
#include <termios.h>
#include <unistd.h>
#include "../openhmdi.h"
#include "string.h"


#define PORT "/dev/Relativty"

#define BAUD B115200


typedef struct
{
	ohmd_device	 base;
	fusion		 sensor_fusion;
	int		 fd;
	struct termios	 settings;
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
//		ohmd_set_error( priv->base.ctx, "File descriptor closed? '%s': %s.", PORT, strerror(errno));
		LOGE( "File descriptor closed? '" PORT "'.");
		return;
	}


	number= read( priv->fd, &priv->buffer[priv->length], sizeof(priv->buffer)-priv->length);
	if( number >= 0 )
	{
		priv->length+= number;
	}
	else if( errno != EAGAIN )
	{
//		ohmd_set_error( priv->base.ctx, "Can not read file '%s': %s.", PORT, strerror(errno));
		LOGE( "Can not read file '" PORT "'.");
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
//		for( number=0; number < priv->length; number++ )
//			priv->buffer[number]= next[number];
		memmove( &priv->buffer, &next, priv->length);
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

#ifdef BAUD
	if( tcsetattr(priv->fd,TCSANOW,&priv->settings) < 0 )
	{
//		ohmd_set_error( priv->base.ctx, "Can not reset file attributes '%s': %s.", PORT, strerror(errno));
		LOGW( "Can not reset file attributes '" PORT "'.");
	}
#endif // BAUD

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


	priv->fd= open( desc->path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	if( priv->fd < 0 )
	{
//		ohmd_set_error( priv->base.ctx, "Can not open file '%s': %s.", PORT, strerror(errno));
		LOGD( "Can not open file '" PORT "'.");
		free( priv);
		priv= NULL;
	}
	else
	{


#ifdef BAUD
		struct termios		 settings;


		if( tcgetattr(priv->fd,&priv->settings) < 0 )
		{
//			ohmd_set_error( priv->base.ctx, "Can not get file attributes '%s': %s.", PORT, strerror(errno));
			LOGD( "Can not get file attributes '" PORT "'.");
		}
		else
		{

			settings= priv->settings;

//			settings.c_cflag |= BAUD;
//			settings.c_cflag &= ~CBAUD;
//			settings.c_cflag |= CBAUDEX | B0;
			cfsetispeed( &settings, BAUD);
			cfsetospeed( &settings, BAUD);

			settings.c_iflag &= ~ICRNL;

			if( tcsetattr(priv->fd,TCSANOW,&settings) < 0 )
			{
//				ohmd_set_error( priv->base.ctx, "Can not set file attributes '%s': %s.", PORT, strerror(errno));
				LOGD( "Can not set file attributes '" PORT "'.");
			}

			if( tcflush(priv->fd,TCOFLUSH) < 0 )
			{
//				ohmd_set_error( priv->base.ctx, "Can not flush file attributes '%s': %s.", PORT, strerror(errno));
				LOGD( "Can not flush termios '" PORT "'.");
			}
		}
#endif // BAUD


		priv->length= 0;
		priv->buffer[priv->length]='\0';

		// Set default device properties
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
	struct stat		 sb;
	ohmd_device_desc	*desc;


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

