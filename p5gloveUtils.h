/*

	P5gloveUtils.h

	Definitions used in utilities for the P5 GLove

	Copyright (c) 2009 Rajmil Fischman <r.a.fischman@keele.ac.uk>
	Contains unwarping matrix for the P5 glove, version 2 developed by 
	(c) 2004 Ross Bencina, <rossb@audiomulch.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

*/

#ifndef M_PI
#define M_PI (3.1415926535898)
#endif

/*****************
	DEFINITIONS
******************/
#define WORLD_SIZE 1200 //500
#define DEFAULT_NORMALISATION 1	// sets output mode for glove position: 0 = absolute coordinates. 1 = coordinates normalised to [-1,1] interval

/**************
	MACROS
***************/
#define	VECTOR_MAGNITUDE(vx,vy,vz)	sqrt(vx*vx + vy*vy + vz*vz)	// calculates vector magnitude
#define	RAD_TO_DEG(a)				(double)a*180.0/M_PI	// radians to degrees
#define	DEG_TO_RAD(a)				(double)a*M_PI/180.0	// degrees to radians
#define MILLISECONDS_TO_SECONDS(a)	(double)a/1000.0		// convert milliseconds to seconds

/**************************************************
	External variables included in P5gloveUtils.c
 **************************************************/
extern VECTOR_COORDS zeroPlaneOrientations[8][8][8];// array containing the coordinates of all the possible 512 
													// normal vectors of the planes formed by combinations of the sensors. 
													// The order in which the sensors are considered is important
													// because it affects the direction of the orientation vectors.
extern CUBERF clippingCube;		// clipping cube
extern double halfXClipping;	// half of X clipping range. Useful for various calculations
extern double halfYClipping;	// half of Y clipping range. Useful for various calculations
extern double halfZClipping;	// half of Z clipping range. Useful for various calculations


/**************************************************
	External functions included in P5gloveUtils.c
 **************************************************/
// Clipping cube
void setClippingCube(double left, double right, double top, double bottom, double front, double rear);

// Rotation bounds
void setRotationBounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);

// Finger bend bounds
void setFingerBounds(double thumbMin, double thumbMax, double indexMin, double indexMax, 
						double middleMin, double middleMax, double ringMin, double ringMax,
						double pinkyMin, double pinkyMax);

// Processing of P5 glove parameters
extern void calculateSensorPosition(P5GlovePtr p5, struct p5glove_ir *pIr);
extern int glovePositionOrientation(P5GlovePtr p5, struct p5glove_data  *pInfo);
extern void calculatePlanes(P5GlovePtr p5);

// Operations on vectors
extern void unit_vector(VECTOR_COORDS* pV, VECTOR_COORDS* pUV);
extern void normal_to_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, VECTOR_COORDS* pNormal);
extern void angle_of_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, double *pAngle);
// Planes
extern void normal_to_plane(VECTOR_COORDS* pP1, VECTOR_COORDS* pP2, VECTOR_COORDS* pP3, VECTOR_COORDS* pNormal);

// Rotations
extern void rotate_vector_X(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout);
extern void rotate_vector_Xcs(VECTOR_COORDS* pVin, double cx, double sx, VECTOR_COORDS* pVout);
extern void rotate_vector_Y(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout);
extern void rotate_vector_Ycs(VECTOR_COORDS* pVin, double cy, double sy, VECTOR_COORDS* pVout);
extern void rotate_vector_Z(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout);
extern void rotate_vector_Zcs(VECTOR_COORDS* pVin, double cz, double sz, VECTOR_COORDS* pVout);
extern void rotate_axis_origin(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout);
extern void rotate_axis_origin_alt(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout);
extern void rotate_axis(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, VECTOR_COORDS* pAxisOrigin, 
																		double angle, VECTOR_COORDS* pVout);
// unwarping matrix by Ross bencina
extern void p5_matrix( int *x_, int *y_, int *z_ );

// Functions taken fully or partially from p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
	/*
	 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
	 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
	 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
	 *
	 *  USB P5 Data Glove support
	 */
extern void p5glove_vec_mat(double vec[3],double mat[4][4],double res[3]);
extern void rotate_axis_origin_alt(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout);
