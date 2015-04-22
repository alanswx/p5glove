/*
 * $Id: p5glove.h,v 1.2 2003/03/26 18:39:30 gus Exp $
 *
 *  Copyright (c) 2003 Jason McMullan <jmcmullan@linuxcare.com>
 *  Modifications to the p5glove_data structure (c) 2009 Rajmil Fischman <r.a.fischman@keele.ac.uk>
 *
 *  USB P5 Data Glove support
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifdef __WIN32__
#include "win32_usb_hid.h"
#else
#include "mac_hid.h"
//#include <usb.h>
#endif

#ifndef P5GLOVE_H
#define P5GLOVE_H


/* Button bitmasks */
#define P5GLOVE_NO_BUTTON	0
#define P5GLOVE_BUTTON_A	1
#define P5GLOVE_BUTTON_B	2
#define P5GLOVE_BUTTON_C	4
#define P5GLOVE_BUTTON_D	8

/* IR Sensor Index */
#define P5GLOVE_IR_WRIST_TOP	0
#define P5GLOVE_IR_PINKY_R		1
#define P5GLOVE_IR_PINKY_L		2
#define P5GLOVE_IR_PALM			3
#define P5GLOVE_IR_INDEX		4
#define P5GLOVE_IR_THUMB_TOP	5
#define P5GLOVE_IR_WRIST_BOT	6
#define P5GLOVE_IR_THUMB_BOT	7

/* Finger Sensor Index */
#define P5GLOVE_INDEX	0
#define P5GLOVE_MIDDLE	1
#define P5GLOVE_RING	2
#define P5GLOVE_PINKY	3
#define P5GLOVE_THUMB	4

/************************************************************
 Corrections to warping due to curvature of the P5 tower
 See Keener, 2004, 'Unwarping the LED Positions (corrected)
 Definitions added by RF
 ************************************************************/
#define P5GLOVE_UPPER_V_BEND_ANGLE	-0.174532925 /* Upper sensor: -10 degrees = -0.174532925 radians */
#define P5GLOVE_LOWER_V_BEND_ANGLE	0.296705972 /* Upper sensor: +17 degrees = 0.296705972 radians */
#define P5GLOVE_UPPER_V_WARP_ANGLE	-0.136353847 /* Upper sensor: -10 degrees + 2.1875 degrees (tan) = -0.136353847 radians */
#define P5GLOVE_LOWER_V_WARP_ANGLE	0.296705972	 /* Lower sensor: +17 degrees (no tan correction) = 0.296705972 radians */
#define P5GLOVE_UPPER_H_WARP_ANGLE	-0.026179938 /* Lower sensor: -1.5 degrees (only tan correction) = -0.026179938 radians */

/************************************************************
 Glove dimensions to warping due to curvature of the P5 t	ower
 See Keener, 2004, 'Unwarping the LED Positions (corrected)
 Definitions added by RF
 ************************************************************/
#define P5GLOVE_HEAD_SEPARATION_M 0.20066		// m
#define P5GLOVE_HEAD_SEPARATION_CM 20.066		// cm
#define P5GLOVE_HEAD_SEPARATION_IN 7.9			// inches
#define P5GLOVE_HEAD_SEPARATION_P5 404.47998	// P5 glove units. 1 unit = 1/51.2 inch

/************************************************************
 Default Values
 Definitions added by RF
 ************************************************************/
#define P5GLOVE_MAX_SENSOR_VALUE	512.0	// maximum value output glove sensors (used to measure angles)
#define P5GLOVE_MAX_FINGER_VALUE	63.0	// maximum value output by the glove for finger bend
#define P5GLOVE_MIN_FINGER_VALUE	0.0		// minimum value output by the glove for finger bend

// Values in cm
#define P5GLOVE_X_CLIPPING_MIN_CM	-34.49
#define P5GLOVE_X_CLIPPING_MAX_CM	42.94
#define P5GLOVE_Y2_CLIPPING_MIN_CM	-40.41	// This is for Y2
#define P5GLOVE_Y2_CLIPPING_MAX_CM	49.89	// This is for Y2
#define P5GLOVE_Y1_CLIPPING_MIN_CM	-60.476	// This is for Y1
#define P5GLOVE_Y1_CLIPPING_MAX_CM	29.824	// This is for Y1
#define P5GLOVE_Z_CLIPPING_MIN_CM	37.71
#define P5GLOVE_Z_CLIPPING_MAX_CM	97.53

// Values in m
#define P5GLOVE_X_CLIPPING_MIN_M	-0.3449
#define P5GLOVE_X_CLIPPING_MAX_M	0.4294
#define P5GLOVE_Y2_CLIPPING_MIN_M	-0.4041		// This is for Y2
#define P5GLOVE_Y2_CLIPPING_MAX_M	0.4989		// This is for Y2
#define P5GLOVE_Y1_CLIPPING_MIN_M	-0.60476	// This is for Y1
#define P5GLOVE_Y1_CLIPPING_MAX_M	0.29824		// This is for Y1
#define P5GLOVE_Z_CLIPPING_MIN_M	0.3771
#define P5GLOVE_Z_CLIPPING_MAX_M	0.9753

// Rotation boundaries
#define  P5GLOVE_X_ROTATION_MIN		-3.1415926535898
#define  P5GLOVE_X_ROTATION_MAX		3.1415926535898
#define  P5GLOVE_Y_ROTATION_MIN		-3.1415926535898
#define  P5GLOVE_Y_ROTATION_MAX		3.1415926535898
#define  P5GLOVE_Z_ROTATION_MIN		-3.1415926535898
#define  P5GLOVE_Z_ROTATION_MAX		3.1415926535898

/************************************************************
 Other
 Definitions added by RF
 ************************************************************/
#ifndef M_PI
#define M_PI (3.1415926535898)
#endif

/********************************
 *	Type Definitions
	Added by RF
 ********************************/
// Structure for clipping cube
typedef struct	{
					double left;
					double right;
					double top;
					double bottom;
					double front;
					double rear;
				}	CUBERF;

// Structure for rotation boundaries
typedef struct	{
					double minX;
					double maxX;
					double minY;
					double maxY;
					double minZ;
					double maxZ;
				}	ROTATIONBOUNDSRF;

// Structure for finger bend boundaries
typedef struct	{
					double thumbMin;
					double thumbFactor;
					double indexMin;
					double indexFactor;
					double middleMin;
					double middleFactor;
					double ringMin;
					double ringFactor;
					double pinkyMin;
					double pinkyFactor;
				}	FINGERBOUNDSRF;

// Structure for storing vector coordinates
typedef struct	{
					double x;
					double y;
					double z;
				}	VECTOR_COORDS;

// Structure for storing rotations
typedef struct	{
					VECTOR_COORDS axis;
					double angle;
				}	ROTATION_DATA;

/********************************
 	P5 GLove Structures
	with modifications by RF
 ********************************/
/* Calibration data from Report 12 & 6 */
// Added by RF. Original in:
/*	
 * $Id: p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
 *
 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
 *
 *  USB P5 Data Glove support
 */
typedef struct p5glove_cal {
		int	id;	/* Device id */
		struct {
			double	v;	/* vertical angle in Radians */	
			double	h;	/* horizontal angle in Radians */
		} head[2];	// two sensor positions
					// head[0] is the bottom head (head 2 in Kenner)
					// head[1]: this is the top head (head 1 in Kenner)
		double	head_dist;	/* in centimeters */
		double  led[10][3];
		VECTOR_COORDS irZeroPos[10];// array containing the position of each sensor 
									// with respect to chosen coordinates for glove position
									// when the glove is not rotated (chosen Y axis is vertical)
									// this is equivalent to acoordinate system where the
									// origin is at the glove position coordinates (0,0,0). Added by RF
	} CALIBRATION_DATA;

typedef struct p5glove_ir {
		int visible;	/* Was the sensor visible? */
		int xUpperSensor,yLowerSensor,yUpperSensor;	// name modified by RF
		double yUpperSensorAngle, yLowerSensorAngle, xUpperSensorAngle; // sensor angles. Added by RF
		VECTOR_COORDS currentPos;	// current measured position of the sensor. Added by RF
	} IR_SENSOR;

struct p5glove_data {
	int buttons;	/* Button bitmask */
	int finger[5];	/* Finger clench values (0-63) */
	int brightest[4]; /* Contains the indexes of the brightest IR detectors:
				   brightest[0] is the index of the brightest detector
				   brightest[1] is the index of the second brightest detector
				   brightest[2] is the index of the third brightest detector
				   brightest[3] is the index of the fourth brightest detector
				   See Carl Kenner, 2004: 'Unwarping the LED positions (corrected)' 
				   Modified by RF 
				*/
	IR_SENSOR ir[8];	/* IR Sensors structure: values are in the interval [-511 - 511] */

	VECTOR_COORDS position;		// glove position
	VECTOR_COORDS positionN;	// glove position normalised to the interval [-1,1]
	ROTATION_DATA rotation;		// glove rotation
	VECTOR_COORDS orientation;	// glove orientation with respect to the Y axis (vertical)
	VECTOR_COORDS orientationN;	// glove orientation with respect to the Y axis (vertical)
};

typedef struct p5glove {
	unsigned char data[24],later[24];
	char name[128];
	CALIBRATION_DATA cal;
#ifdef __WIN32__
    USBHIDHandle *usb;
#else
	struct usb_dev_handle *usb;
	long long nextsamp;
#endif
} P5_GLOVE;

typedef P5_GLOVE *P5GlovePtr;

/********************************
 	Global variables
	Added by RF
 ********************************/
extern VECTOR_COORDS zeroPlaneOrientations[8][8][8];// array containing the coordinates of all the possible 512 
													// normal vectors of the planes formed by combinations of the sensors. 
													// The order in which the sensors are considered is important
													// because it affects the direction of the orientation vectors.
extern CUBERF clippingCube;		// clipping cube
extern double halfXClipping;	// half of X clipping range. Useful for various calculations
extern double halfYClipping;	// half of Y clipping range. Useful for various calculations
extern double halfZClipping;	// half of Z clipping range. Useful for various calculations

extern ROTATIONBOUNDSRF rotationBounds;		// Rotation bounds
extern double halfXRotation;				// half of X rotation range. Useful for various calculations
extern double halfYRotation;				// half of Y rotation range. Useful for various calculations
extern double halfZRotation;				// half of Z rotation range. Useful for various calculations

extern FINGERBOUNDSRF fingerBounds;	// Finger bend bounds
extern double halfThumbBend;		// half of thumb bend range range. Useful for various calculations	
extern double halfIndexBend;		// half of index bend range range. Useful for various calculations	
extern double halfMiddleBend;		// half of middle bend range range. Useful for various calculations	
extern double halfRingBend;			// half of ring bend range range. Useful for various calculations	
extern double halfPinkyBend;		// half of pinky bend range range. Useful for various calculations	
	
/* p5glove_open:
 * Open a handle to a P5 Glove. Returns NULL on error,
 * and sets errno appropriately.
 */
P5GlovePtr p5glove_open(void);

/* p5glove_close:
 * Close an open handle to a P5 Glove.
 */
void p5glove_close(P5GlovePtr glove);

/* p5glove_sample
 * Retrieve a sample from the P5
 * Returns 0 on succes, -1 on error, and sets 
 * errno to EAGAIN is called faster then the refresh frequency.
 */
int p5glove_sample(P5GlovePtr glove, struct p5glove_data *data);

/////////////////////////////////////////////////////////////////////
// Added functions from p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
/*
 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
 *
 *  USB P5 Data Glove support
 */
///////////////////////////////////////////////////////////////////////
void p5glove_begin_calibration(P5GlovePtr p5);
void p5glove_end_calibration(P5GlovePtr p5);
int p5glove_get_mouse_mode(P5GlovePtr p5);
void p5glove_mouse_mode_on(P5GlovePtr p5);
void p5glove_mouse_mode_off(P5GlovePtr p5);
//static int p5g_parse_report6(struct p5glove_data *info,int8_t *buff);
//static int p5g_parse_report12(struct p5glove_data *info,uint8_t *buff);
int p5glove_calibrate(P5GlovePtr p5);

#endif /* P5GLOVE_H */
