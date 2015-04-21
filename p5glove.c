/*
 * $Id: p5glove.c,v 1.2 2003/03/26 18:39:30 gus Exp $
 *
 *  Copyright (c) 2003 Jason McMullan <jmcmullan@linuxcare.com>
 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
 *  Some modifications to the P5GLove structure (c) 2009 Rajmil Fischman <r.a.fischman@keele.ac.uk>
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

#include <stdio.h>
#include <stdlib.h> /* for calloc */
#include <string.h> /* for memcpy */
#include <stdint.h> /* for integer types */
#include "hidapi.h"
#include <errno.h>
#include "p5glove.h"



static void process_sample(struct p5glove *p5, struct p5glove_data *info)
{
	unsigned char *data = p5->data, *later=p5->later;
	unsigned char tmp[24];
	int i;

	/* Decode data.
	 * Format (nibbles from LSB to MSB)
	 *                 11111111111111112222222222222222
	 * 0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF
	 * 01ddddddddBCCCCVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVxxx
	 *  0 1 2 3 4 5 6 7 8 9 A B C D E F 0 1 2 3 4 5 6 7
	 *                                  1 1 1 1 1 1 1 1
	 *	  (bytes from LSB to MSB)
	 *
	 * d - Packed 6-bit bend sensor data (Index, Middle, Ring, Pinky, Thumb)
	 * B - Button data. A=1, B=2, C=4
	 * C - IR sensor report index
	 * V - Packed 30 bit signed IR info (10 bits X, 10 bits Y, 10 bits Z), x4
	 */

#ifndef __WIN32__
	if (data[16] == 1) {	/* Hmm. Offset by 16. Fix it. */
		memcpy(tmp,later,24-16);
		memcpy(later,data+16,24-16);
		memcpy(tmp+24-16,data,16);
		data=tmp;
	} else if (data[8] == 1) { /* Hmm. Offset by 8. Fix it. */
		memcpy(tmp,later,24-8);
		memcpy(later,data+8,24-8);
		memcpy(tmp+24-8,data,8);
		data=tmp;
	}
#endif 

	// If this is not report 1, it is not a sample
	if (data[0] != 1) {
		return;
	}

	// get data for finger bends
	for (i=0; i < 5; i++) {
		int value;

		switch (i) {
			case 0: value = data[1] >> 2; break;
			case 1: value = ((data[1] & 0x3) << 4) | (data[2] >> 4); break;
			case 2: value = ((data[2] & 0xF) << 2) | (data[3] >> 6); break;
			case 3: value = (data[3] & 0x3F); break;
			case 4: value = data[4] >> 2; break;
			default: value = 0; break;
		}

		info->finger[i]=value;
	}

	// Get button data
	info->buttons=data[5]>>4;

	/* Clear visibility */
	for (i=0 ; i <8; i++)
		info->ir[i].visible=0;

	for (i=0; i < 4; i++) 
	{
		int axis;
		int value;

		axis = (data[5+((i+1)>>1)] >> ((i & 1) * 4)) & 0xf;
		if (axis > 7 )
		{
            /* axis == 15 probably means that the slot is unused */
//RF            if( axis != 15)
//RF			{
                /* at this stage we're not sure why the glove returns values > 7 here, but it does */
//RF                printf( "-------------------------------------------------------------------------------\n" );
//RF                printf( "warning: sensor slot %d > 7 (%d)\n", i, axis );
//RF                printf( "-------------------------------------------------------------------------------\n" );
//RF			}
            continue;
        }
		info->brightest[i] = axis;					/* save brightness information. Added by RF */

		switch (i) {
			case 0: value = ((data[0x7] & 0x0F)<<26) | (data[0x8]<<18) | (data[0x9]<<10) | (data[0xA]<<2) | (data[0x0B]>>6);
				break;
			case 1: value = ((data[0xB] & 0x3F)<<24) | (data[0xC]<<16) | (data[0xD] << 8) | data[0x0E];
				break;
			case 2: value = (data[0xF] << 22) | (data[0x10]<<14) | (data[0x11] << 6) | (data[0x12] >> 2);
				break;
			case 3: value = ((data[0x12] & 0x03)<<28) | (data[0x13] << 20) | (data[0x14] << 12) | (data[0x15] << 4) | (data[0x16] >> 4);
				break;
			default:
				/* Impossible! */
				return;
	}

#define SEX(value) ((((value) & 0x3FF) << (32-10)) >> (32-10))
		info->ir[axis].xUpperSensor=SEX(value);		/* RF */
		info->ir[axis].yLowerSensor=SEX(value>>10);	/* RF */
		info->ir[axis].yUpperSensor=SEX(value>>20);	/* RF */
		info->ir[axis].visible=1;
	}

	/* Remove any spurious data */
	for (i=0; i < 8; i++) {
		int j;

		if (!info->ir[i].visible)
			continue;
	
		// compare distances with every other sensor to see if there is a big leap
		for (j=0; j < 8; j++) {
			double dist,tmp;

			if (i == j)
				continue;
			if (!info->ir[j].visible)	// Don't bother with inactive sensors
				continue;

			tmp=info->ir[i].xUpperSensor-info->ir[j].xUpperSensor;	/* RF */
			dist = tmp*tmp;
			tmp=info->ir[i].yLowerSensor-info->ir[j].yLowerSensor;	/* RF */
			dist += tmp*tmp;
			tmp=info->ir[i].yUpperSensor-info->ir[j].yUpperSensor;	/* RF */
			dist += tmp*tmp;

			if (dist < (200.0*200.0))
				break;
		}

		if (j == 8)
			info->ir[i].visible=0;
	}
}

P5GlovePtr p5glove_open(void)
{
    struct p5glove *p5;
        if (hid_init())
                return NULL;
    hid_device *usb = hid_open(0x0d7f,0x100,NULL);
#if 0
    USBHIDHandle *usb = OpenUSBHID (
            0,                                                  /* 0th matching device */
            0x0d7f,                                             /* vendor id */
            0x0100,                                             /* product id */
            0,                                                  /* version number (not used) */
            SELECT_VENDOR_ID_FLAG | SELECT_PRODUCT_ID_FLAG );   /* selection flags */
#endif
    if( usb ){
        hid_set_nonblocking(usb, 1);

        p5 = calloc(1,sizeof(*p5));
        p5->usb = usb;

		return p5;
    }

	return NULL;
}

void p5glove_close(P5GlovePtr p5)
{
	if (p5->usb != NULL)
        hid_close(p5->usb);
	p5->usb=NULL;
	free(p5);
      hid_exit();
}

int p5glove_sample(P5GlovePtr p5, struct p5glove_data *info)
{
	int err;

    if (hid_read(p5->usb,p5->data, 24) ==24) {
    //if( ReadUSBHID( p5->usb, p5->data, 24 ) == 24 ){
        process_sample(p5, info);
        err = 0;
    }else{
        err = EACCES;
    }
    
	return err;
}

/////////////////////////////////////////////////////////////////////
// Added functions from p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
/*
 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
 *
 *  USB P5 Data Glove support
 */
 //	Some modifications by Rajmil Fischman, 2009
//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////
//	Auxiliary functions to unpack data
////////////////////////////////////////////
static uint32_t get_bits(uint8_t *data,int pos,int len)
{
	int index = (pos>>3);
	uint32_t value = 0;

	pos &= 0x7;
	if (pos != 0) {
		value |= (data[index] & (0xff >> pos)) << (len-(8-pos));
		len -= (8-pos);
		index++;
	}

	for (; len >= 8; len-=8) {
		value |= data[index] << (len-8);
		index++;
	}

	if (len > 0)
		value |= (data[index] & (0xff << (8-len))) >> (8-len);

	return value;
}

static int32_t get_bits_signed(uint8_t *data,int pos,int len)
{
	uint32_t value,mask=0;

	value = get_bits(data,pos,len);
	if (value & (1 << (len-1)))
		mask = 0xffffffff << (len - 32);
	return value | mask;
}

///////////////////////////////
// Calibration values
///////////////////////////////
//static int p5g_parse_report6(P5GlovePtr p5,int8_t *buff)			// RF
static int p5g_parse_report6(P5GlovePtr p5,unsigned char *buff)		// RF
{
	/* We are given 32nds of a degree, so: */
#define REPORT6_TO_RAD(x)	((x)/32.0*M_PI/180.0)
	p5->cal.head[0].v += REPORT6_TO_RAD(buff[1]);
	p5->cal.head[0].h += REPORT6_TO_RAD(buff[2]);
	p5->cal.head[1].v += REPORT6_TO_RAD(buff[3]);
	p5->cal.head[1].h += REPORT6_TO_RAD(buff[4]);

	/* The following is in 10ths of an inch. Convert to meters */
#define REPORT6_TO_METERS(x)	((x)/393.70039)
	p5->cal.head_dist = REPORT6_TO_METERS(buff[5]);

	return 0;
}

//static int p5g_parse_report12(P5GlovePtr p5,uint8_t *buff)		// RF
static int p5g_parse_report12(P5GlovePtr p5,unsigned char *buff)	// RF
{
	int led,axis;

	p5->cal.id = get_bits(buff,8,8);
	switch (p5->cal.id) {
		case 0:	// ID 0 is the vertical sensor value. Modified by RF
			p5->cal.head[0].v +=  P5GLOVE_LOWER_V_BEND_ANGLE;	// cal.head[0]: this is the bottom head (head 2 in Kenner)
			p5->cal.head[1].v += P5GLOVE_UPPER_V_BEND_ANGLE;	// cal.head[1]: this is the top head (head 1 in Kenner)
			break;
		default: // RF fprintf(stderr,"libp5glove: unknown glove type %d\n",p5->cal.id);
			 return -EINVAL;
	}
	/* The following is in 100ths of an inch. Convert to meters */
#define REPORT12_TO_METERS(x)	((x)/3937.0039)
	for (led = 0; led < 10; led++) {
		for (axis = 0; axis < 3; axis++) {
			int16_t val;
			val = get_bits(buff,16+(48*led)+(16*axis),16);
			p5->cal.led[led][axis]=REPORT12_TO_METERS(val);
			if (axis == 0)
				p5->cal.led[led][axis] *= -1.0;		// flip X value
		}
	}


//#ifdef DEBUG_CAL
//for (led=0; led < 12; led++) {
//	DPRINTF("%d %.4lf %.4lf %.4lf\n",led,
//			p5->cal.led[led][0],
//			p5->cal.led[led][1],
//			p5->cal.led[led][2]);
//}
//#endif

	return 0;
}

int p5glove_calibrate(P5GlovePtr p5)
{
	unsigned char report6_buff[6];		// RF
	unsigned char report12_buff[255];	// RF
	int err;

	memset(&p5->cal,0,sizeof(p5->cal));
	report12_buff[0]=12;
        err  = hid_get_feature_report(p5->usb, report12_buff, sizeof(report12_buff));
	//err=GetUSBHIDFeature(p5->usb,report12_buff,sizeof(report12_buff));

	if (err < 0) goto end;

	err=p5g_parse_report12(p5,report12_buff);
	if (err < 0) goto end;

	report6_buff[0]=6;
	//err=GetUSBHIDFeature(p5->usb,report6_buff,sizeof(report6_buff));
        err  = hid_get_feature_report(p5->usb, report6_buff, sizeof(report6_buff));
	if (err < 0) goto end;

	err=p5g_parse_report6(p5,report6_buff);


	// RF added the following code in order to be able to use the VECTOR_COORDS structure 
	int i = 0;
	for(i=0 ; i<8 ; i++)
	{
		p5->cal.irZeroPos[i].x = p5->cal.led[i][0];
		p5->cal.irZeroPos[i].y = p5->cal.led[i][1];;
		p5->cal.irZeroPos[i].z = p5->cal.led[i][2];
	}


//#ifdef DEBUG_CALIB
//DPRINTF("Cal results:\n");
//{ int i; for (i=0;i<2;i++) {
//DPRINTF("\tHead %d.v = %.2f deg\n",i,p5->cal.head[i].v * 180.0/M_PI);
//DPRINTF("\tHead %d.h = %.2f deg\n",i,p5->cal.head[i].h * 180.0/M_PI);} }
//DPRINTF("\tHead Distance = %.2f meters\n",p5->cal.head_dist);
//#endif

end:
	return err;
}

///////////////////////////////
// Glove Reported Features
///////////////////////////////
void p5glove_begin_calibration(P5GlovePtr p5)
{
//    char report[2] = { 0x01, 0x01 };			// RF
    unsigned char report[2] = { 0x01, 0x01 };	// RF
    //SetUSBHIDFeature( p5->usb, report, 2 );
    hid_send_feature_report( p5->usb, report, 2 );
}

void p5glove_end_calibration(P5GlovePtr p5)
{
//    char report[2] = { 0x01, 0x00 };			// RF
    unsigned char report[2] = { 0x01, 0x00 };	// RF
    //SetUSBHIDFeature( p5->usb, report, 2 );
    hid_send_feature_report( p5->usb, report, 2 );
}

int p5glove_get_mouse_mode(P5GlovePtr p5)
{
 //   char report[2] = { 0x05, 0x00 };			// RF
    unsigned char report[2] = { 0x05, 0x00 };	// RF
    //GetUSBHIDFeature( p5->usb, report, 2 );
    hid_get_feature_report(p5->usb, report, 2);
    return (report[1] == 0x01)? 1 : 0;
}

void p5glove_mouse_mode_on(P5GlovePtr p5)
{
//    char report[2] = { 0x05, 0x01 };			// RF
    unsigned char report[2] = { 0x05, 0x01 };	// RF
    //SetUSBHIDFeature( p5->usb, report, 2 );
    hid_send_feature_report( p5->usb, report, 2 );
}

void p5glove_mouse_mode_off(P5GlovePtr p5)
{
//    char report[2] = { 0x05, 0xFF };			// RF
    unsigned char report[2] = { 0x05, 0xFF };	// RF
    //SetUSBHIDFeature( p5->usb, report, 2 );
    hid_send_feature_report( p5->usb, report, 2 );
}
