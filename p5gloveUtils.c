/*

	P5gloveUtils.c

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
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "hidapi.h"
#include "p5glove.h"
#include "P5gloveUtils.h"

/*****************************
 *	Variable Declarations
 *****************************/
VECTOR_COORDS zeroPlaneOrientations[8][8][8];	// array containing the coordinates of all the possible 512 
												// normal vectors of the planes formed by combinations of the sensors. 
												// The order in which the sensors are considered is important
												// because it affects the direction of the orientation vectors.
CUBERF clippingCube;	// clipping cube
double halfXClipping;	// half of X clipping range. Useful for various calculations
double halfYClipping;	// half of Y clipping range. Useful for various calculations
double halfZClipping;	// half of Z clipping range. Useful for various calculations

ROTATIONBOUNDSRF rotationBounds;	// Rotation bounds
double halfXRotation;				// half of X rotation range. Useful for various calculations
double halfYRotation;				// half of Y rotation range. Useful for various calculations
double halfZRotation;				// half of Z rotation range. Useful for various calculations

FINGERBOUNDSRF fingerBounds;	// Finger bend bounds
double halfThumbBend;			// half of thumb bend range range. Useful for various calculations	
double halfIndexBend;			// half of index bend range range. Useful for various calculations	
double halfMiddleBend;			// half of middle bend range range. Useful for various calculations	
double halfRingBend;			// half of ring bend range range. Useful for various calculations	
double halfPinkyBend;			// half of pinky bend range range. Useful for various calculations	

/****************************************
 ****************************************
 *	Processing of P5 glove parameters	*
 ****************************************
 ****************************************/

//////////////////////////////////////////////////////////////////////////////
//
//	void calculateSensorPosition(struct p5glove_ir *pIr)
//
//	Calculates position of sensor according to algorithms from 
//	Kenner, 2004, 'Unwarping the LED Positions (corrected)' 
//
//	Arguments
//	struct p5glove_ir *pIr		Pointer to infrared sensor data structure 
//
//	Returns
//	position of sensor saved in the data structure (via pIr)
//
//////////////////////////////////////////////////////////////////////////////
void calculateSensorPosition(P5GlovePtr p5, struct p5glove_ir *pIr)
{
	/****************************************************************************
		Calculate angles including corrections from Kenner, 2004, 
		'Unwarping the LED Positions (corrected)'

		Vertical angle from upper sensor
		Vertical angle from lower sensor
		Horizontal angle from upper sensor

		Algorithm: 	
		angle (radians) = arctan(value from glove / maximum value) + correction

		Added by RF
	 ****************************************************************************/
//	pIr->yUpperSensorAngle = atan((double)pIr->yUpperSensor/P5GLOVE_MAX_SENSOR_VALUE)  + P5GLOVE_UPPER_V_WARP_ANGLE;
//	pIr->yLowerSensorAngle = atan((double)pIr->yLowerSensor/P5GLOVE_MAX_SENSOR_VALUE) + P5GLOVE_LOWER_V_WARP_ANGLE;
//	pIr->xUpperSensorAngle = atan((double)pIr->xUpperSensor/P5GLOVE_MAX_SENSOR_VALUE) + P5GLOVE_UPPER_H_WARP_ANGLE;

	pIr->yUpperSensorAngle = atan((double)pIr->yUpperSensor/P5GLOVE_MAX_SENSOR_VALUE)  + p5->cal.head[1].v;
	pIr->yLowerSensorAngle = atan((double)pIr->yLowerSensor/P5GLOVE_MAX_SENSOR_VALUE) + p5->cal.head[0].v;
	pIr->xUpperSensorAngle = atan((double)pIr->xUpperSensor/P5GLOVE_MAX_SENSOR_VALUE) + p5->cal.head[1].h;

//if(count--==0)
//{
//printf("\nP5GLOVE_UPPER_V_WARP_ANGLE = %lf\tP5GLOVE_UPPER_H_WARP_ANGLE = %lf\tP5GLOVE_LOWER_V_WARP_ANGLE = %lf\n", 
//	   P5GLOVE_UPPER_V_WARP_ANGLE, P5GLOVE_UPPER_H_WARP_ANGLE, P5GLOVE_LOWER_V_WARP_ANGLE);
//printf("cal.head[1].v = %lf\tcal.head[1].h = %lf\tcal.head[0].v = %lf\n", 
//	   p5->cal.head[1].v, p5->cal.head[1].h, p5->cal.head[0].v);
//count--;
//}
 

	/***********************************************************************************
		Calculate Z coordinate from Kenner, 2004, 
		'Unwarping the LED Positions (corrected)'

		Algorithm: 	
		Z = HEAD separation / (-tan(head1 vertical angle) + tan(head2 vertical angle)

		Physical HEAD separation = 20.066 cm = 7.9 inch = 404.47998 P5 units

			1 P5 unit = 1/51.2 inch

		Added by RF
	 ***********************************************************************************/
	/* Avoid dividing by 0 */
	double denom = -tan(pIr->yUpperSensorAngle) + tan(pIr->yLowerSensorAngle);
	if( denom == 0.0 )
		pIr->currentPos.z = 0;
	else
		pIr->currentPos.z = (double)p5->cal.head_dist / denom;

	/***********************************************************************************
		Calculate Y coordinate from Kenner, 2004, 
		'Unwarping the LED Positions (corrected)'

		Algorithm Option 1: 	
		Y (from head 1) = Z tan(head1 vertical angle)
				
		Algorithm Option 2: 	
		Y (from head 2) = Z tan(head2 vertical angle)

		Added by RF
	 ***********************************************************************************/
	double Y1 = pIr->currentPos.z * tan(pIr->yUpperSensorAngle);
	pIr->currentPos.y = pIr->currentPos.z * tan(pIr->yLowerSensorAngle);


	/*************************************************************************************************************
		Calculate X coordinate from Kenner, 2004, 
		'Unwarping the LED Positions (corrected)'

		Algorithm: 	
		X = [Z tan(horizontal angle)] * cos(abs(head 1 vertical angle) - 10 degrees) / cos(head 1 vertical angle)
				
		Added by RF
	 *************************************************************************************************************/
						
	/* Avoid division by 0 */
	denom = cos(pIr->yUpperSensorAngle);
	if( denom == 0.0 )
		pIr->currentPos.x = 0;
	else 
		pIr->currentPos.x = -pIr->currentPos.z * tan(pIr->xUpperSensorAngle) * 
								cos(pIr->yUpperSensorAngle - P5GLOVE_UPPER_V_BEND_ANGLE) / denom;
//								cos(pIr->yUpperSensorAngle - p5->cal.head[0].v) / denom;
		/* the '-' sign is for consistency with a right handed coordinate system i.e.
		   left is negative and right is positive */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void glovePositionOrientation(P5GlovePtr p5, struct p5glove_data  *pInfo)
//
//	Calculates glove position and orientation from three brightest sensors
//
//	Arguments
//	P5GlovePtr p5					Pointer to glove structure
//	struct p5glove_data  *pInfo	Pointer to glove data structure with current values
//
//	Returns
//	1 if it was possible to calculate the position and orientation successfully
//	0 otherwise (normally because there are not enough active sensors to calculate the position and orientation)
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int glovePositionOrientation(P5GlovePtr p5, struct p5glove_data  *pInfo)
{
//	static first = 1;
//	static double lastAngle;
	int s1,s2,s3;		// indexes of the sensors used to calculate glove position and orientation
	int success = 0;	// 1 if successful, 0 otherwise

	/**************************************
		Find the three brightest sensors
	 **************************************/
	s1 = pInfo->brightest[0];
	s2 = pInfo->brightest[1];
	s3 = pInfo->brightest[2];

	/*********************************************************
		Process data only if we have three active sensors.
		Otherwise, it is nto possible to find a plane and
		its orientation.
	 *********************************************************/
	// only use sensor values between 0 and 7.
	// This check is necessary because the glove sends a sensor value of 15 when it goes out of range
	if(s1>=0 && s1<=7 && s2>=0 && s2<=7 && s3>=0 && s3<=7)			
	{																
		// do calculations only if there are three visible sensors
		if(pInfo->ir[s1].visible && pInfo->ir[s2].visible && pInfo->ir[s3].visible) 
		{
			/****************************************************************************
				Calculate positions using algorithms from Kenner, 2004, 
				'Unwarping the LED Positions (corrected)'
			****************************************************************************/
			calculateSensorPosition(p5, &(pInfo->ir[s1]));
			calculateSensorPosition(p5, &(pInfo->ir[s2]));
			calculateSensorPosition(p5, &(pInfo->ir[s3]));

			/*************************************************************
				Calculate normal to the plane formed by these positions
			*************************************************************/
			VECTOR_COORDS normal;
			normal_to_plane(&(pInfo->ir[s1].currentPos), &(pInfo->ir[s2].currentPos), &(pInfo->ir[s3].currentPos), &normal);

			/*****************************************************************************************
				Calculate axis and angle of rotation in order to bring the plane to the Zero position
			*****************************************************************************************/
			VECTOR_COORDS axis;
			double angle;

			normal_to_vectors(&normal, &(zeroPlaneOrientations[s1][s2][s3]), &axis);
			angle_of_vectors(&normal, &(zeroPlaneOrientations[s1][s2][s3]), &angle);

			/**************************************************************************************
				Rotate the Zero position to find the difference in positions of the other sensors
			**************************************************************************************/
			VECTOR_COORDS vOut;
			VECTOR_COORDS rotationPoint;

			// rotate zero position of the brightest sensor using the axis calculated above with origin at (0, 0, 0)
			rotate_axis_origin_alt(&(p5->cal.irZeroPos[s1]), &axis, angle, &vOut);

			// Subtract the difference from the reference position to get the displacement of the plane
			pInfo->position.x = pInfo->ir[s1].currentPos.x - vOut.x;
			pInfo->position.y = pInfo->ir[s1].currentPos.y - vOut.y;
			pInfo->position.z = pInfo->ir[s1].currentPos.z - vOut.z;

			/**************************************************
				Normalise values to interval [-1, 1] 
			***************************************************/
			pInfo->positionN.z = ((pInfo->position.z - clippingCube.front) / halfZClipping) - 1.0;
			if(pInfo->positionN.z<-1.0) pInfo->positionN.z = -1.0;
			else if(pInfo->positionN.z>1.0) pInfo->positionN.z = 1.0;

			pInfo->positionN.y = ((pInfo->position.y - clippingCube.bottom) / halfYClipping) - 1.0;
			if(pInfo->positionN.y<-1.0) pInfo->positionN.y = -1.0;
			else if(pInfo->positionN.y>1.0) pInfo->positionN.y = 1.0;

			pInfo->positionN.x = ((pInfo->position.x - clippingCube.left) / halfXClipping) - 1.0;
			if(pInfo->positionN.x<-1.0) pInfo->positionN.x = -1.0;
			else if(pInfo->positionN.x>1.0) pInfo->positionN.x = 1.0;

			/**************************************************************************************
				Find Glove Orientation by applying the rotation above to the Y Axis
			**************************************************************************************/
			// Initialise Y Axis (vertical)
			VECTOR_COORDS Xaxis, Yaxis, Zaxis;
			Xaxis.x = 1.0;	Xaxis.y = 0.0;	Xaxis.z = 0.0;
			Yaxis.x = 0.0;	Yaxis.y = 1.0;	Yaxis.z = 0.0;
			Zaxis.x = 0.0;	Zaxis.y = 0.0;	Zaxis.z = 1.0;

			// The angle may need correction depending on the orientation of the previous axis
			// because the function calculating the cosine is insensitive to the direction in which
			// the angle is measured.
//			if(!first)
//			{
				// if the difference between the previous and current angle is more than 90 degrees, flip the angle
//				if(fabs(angle-lastAngle)>M_PI/2.0)
//				{
//post("Angle Flip lastAngle = %.2lf thisAngle = %.2lf flippedAngle = %.2lf", RAD_TO_DEG(lastAngle), RAD_TO_DEG(angle), RAD_TO_DEG((M_PI - angle)));
//					angle = - angle;
//				}
//			}
//			else
//				first = 0;

			// remember angle
//			lastAngle = angle;

			//
			// rotate Y axis
			//
//			rotate_axis_origin_alt(&Yaxis, &axis, angle, &(pInfo->orientation));
			rotate_axis_origin_alt(&Yaxis, &axis, angle, &vOut);
			
			//
			// Find angles of rotation of vOut for every axis
			//
			// ---------------------------------------
			// Angle of rotation of the X axis (pitch)
			// ---------------------------------------
			// This is the angle between the vector projection on the plane YZ and the Y axis
			//
			VECTOR_COORDS vProj;
			vProj.x = 0;	vProj.y = vOut.y;	vProj.z = vOut.z;		// find projection
			angle_of_vectors(&vProj, &Yaxis, &(pInfo->orientation.x));	// find absolute angle
			if(vOut.z < 0)												// find sign (normal RH system)
				pInfo->orientation.x = -pInfo->orientation.x;
			//
			// -------------------------------------
			// Angle of rotation of the Y axis (yaw)
			// -------------------------------------
			// This is the angle between the vector projection on the plane ZX and the Z axis
			//
			vProj.x = vOut.x;	vProj.y = 0;	vProj.z = vOut.z;		// find projection
			angle_of_vectors(&vProj, &Zaxis, &(pInfo->orientation.y));	// find absolute angle
			if(vOut.x < 0)												// find sign
				pInfo->orientation.y = -pInfo->orientation.y;			
			//
			// --------------------------------------
			// Angle of rotation of the Z axis (roll)
			// --------------------------------------
			// This is the angle between the vector projection on the plane XY and the X axis
			vProj.x = vOut.x;	vProj.y = vOut.y;	vProj.z = 0;		// find projection
			angle_of_vectors(&vProj, &Xaxis, &(pInfo->orientation.z));	// find absolute angle
			if(vOut.y > 0)												// find sign
				pInfo->orientation.z = -pInfo->orientation.z;			// NOTE that the polarity of the rotation is reversed (LH system)

			
			/**************************************************
				Normalise values to interval [-1, 1] 
			***************************************************/
			pInfo->orientationN.z = ((pInfo->orientation.z - rotationBounds.minZ) / halfZRotation) - 1.0;
			if(pInfo->orientationN.z<-1.0) pInfo->orientationN.z = -1.0;
			else if(pInfo->orientationN.z>1.0) pInfo->orientationN.z = 1.0;

			pInfo->orientationN.y = ((pInfo->orientation.y - rotationBounds.minY) / halfYRotation) - 1.0;
			if(pInfo->orientationN.y<-1.0) pInfo->orientationN.y = -1.0;
			else if(pInfo->orientationN.y>1.0) pInfo->orientationN.y = 1.0;

			pInfo->orientationN.x = ((pInfo->orientation.x - rotationBounds.minX) / halfXRotation) - 1.0;
			if(pInfo->orientationN.x<-1.0) pInfo->orientationN.x = -1.0;
			else if(pInfo->orientationN.x>1.0) pInfo->orientationN.x = 1.0;

			// We've got a valid position and orientation!
			success = 1;
		}
	}
		
	return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void calculatePlanes(P5GlovePtr p5)
//
//	Calculates normals to planes formed by combinations of three sensors
//
//	Arguments
//	P5GlovePtr p5					Pointer to glove structure
//
//	Returns
//	Array of vectors storing the normals to the planes formed by each sensor combination (via Planes)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculatePlanes(P5GlovePtr p5)
{
	int i,j,k;

	for(i=0 ; i<8 ; i++)
		for(j=0 ; j<8 ; j++)
			for(k=0 ; k<8 ; k++)
			{
				// Calculate normal
				normal_to_plane(&(p5->cal.irZeroPos[i]), &(p5->cal.irZeroPos[j]), &(p5->cal.irZeroPos[k]), 
						&zeroPlaneOrientations[i][j][k]);

//				if(i==0 && j==1)
//				{
//					printf("Normal to plane between sensors %d %d %d:\tX=%6.2f\tY=%6.2f\tZ=%6.2f\n",i,j,k,
//						Planes[i][j][k].x, Planes[i][j][k].y,Planes[i][j][k].z);
//				}
			}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void setClippingCube(double left, double right, double top, double bottom, double front, double rear)
//
//	Sets 3D boundaries used to normalise glove coordinates. It also calculates half distances, which are
//	useful in setting intervals between -1 and 1.
//
//	Arguments
//	double left		left boundary
//	double right	right boundary
//	double top		top boundary
//	double bottom	bottom boundary
//	double front	boundary towards to the tower (front)
//	double rear		boundary towards the user (rear)
//
//	Returns
//	Nothing, but coordinates are stored in the global variable clippingCube
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setClippingCube(double left, double right, double top, double bottom, double front, double rear)
{
	clippingCube.left = left;
	clippingCube.right = right;
	clippingCube.top = top;
	clippingCube.bottom = bottom;
	clippingCube.front = front;
	clippingCube.rear = rear;


	/* calculate half of the clipping distances and avoid division by zero */
	halfXClipping = fabs(clippingCube.right - clippingCube.left) / 2.0;
	halfXClipping = (halfXClipping > 0 ? halfXClipping : 1.0);
	
	halfYClipping = fabs(clippingCube.top - clippingCube.bottom) / 2.0;
	halfYClipping = (halfYClipping > 0 ? halfYClipping : 1.0);
	
	halfZClipping = fabs(clippingCube.rear - clippingCube.front) / 2.0;
	halfZClipping = (halfZClipping > 0 ? halfZClipping : 1.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void setRotationBounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
//
//	Sets 3D boundaries used to normalise glove coordinates. It also calculates half distances, which are
//	useful in setting intervals between -1 and 1.
//
//	Arguments
//	double minX	lower X rotation boundary
//	double maxX	higher X rotation boundary
//	double minY	lower Y rotation boundary
//	double maxY	higher Y rotation boundary
//	double minZ	lower Z rotation boundary
//	double maxZ	higher Z rotation boundary
//
//	Returns
//	Nothing, but coordinates are stored in the global variable rotationBounds
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setRotationBounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
	rotationBounds.minX = minX;
	rotationBounds.maxX = maxX;
	rotationBounds.minY = minY;
	rotationBounds.maxY = maxY;
	rotationBounds.minZ = minZ;
	rotationBounds.maxZ = maxZ;


	/* calculate half of the clipping distances and avoid division by zero */
	halfXRotation = fabs(rotationBounds.maxX - rotationBounds.minX) / 2.0;
	halfXRotation = (halfXRotation > 0 ? halfXRotation : 1.0);
	
	halfYRotation = fabs(rotationBounds.maxY - rotationBounds.minY) / 2.0;
	halfYRotation = (halfYRotation > 0 ? halfYRotation : 1.0);

	halfZRotation = fabs(rotationBounds.maxZ - rotationBounds.minZ) / 2.0;
	halfZRotation = (halfZRotation > 0 ? halfZRotation : 1.0);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void setFingerBounds(double thumbMin, double thumbMax, double indexMin, double indexMax, 
//						double middleMin, double middleMax, double ringMin, double ringMax,
//						double pinkyMin, double pinkyMax);
//
//	Sets 3D boundaries used to normalise glove coordinates. It also calculates half distances, which are
//	useful in setting intervals between -1 and 1.
//
//	Arguments
//			double thumbMin	thumb minimum bend
//			double thumbMax thumn maximum bend
//			double indexMin index minimum bend
//			double indexMax index maximum bend
//			double middleMin middle minimum bend
//			double middleMax middle maximum bend
//			double ringMin ring minimum bend
//			double ringMax ring maximum bend
//			double pinkyMin pinky minimum bend
//			double pinkyMax pinky maximum bend
//
//	Returns
//	Nothing, but coordinates are stored in the global variable rotationBounds
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setFingerBounds(double thumbMin, double thumbMax, double indexMin, double indexMax, 
						double middleMin, double middleMax, double ringMin, double ringMax,
						double pinkyMin, double pinkyMax)
{
	/*
		Algorithm

		scaledValue = (value - min) * factor

		where
					   1.0	
			factor = -------
					 max-min
	*/
	double temp;

	fingerBounds.thumbMin = thumbMin;
	temp = fabs(thumbMax-thumbMin);
	temp = (temp > 0.0 ? temp : 1.0);
	fingerBounds.thumbFactor = 1.0 / temp;

	fingerBounds.indexMin = indexMin;
	temp = fabs(indexMax-indexMin);
	temp = (temp > 0.0 ? temp : 1.0);
	fingerBounds.indexFactor = 1.0 / temp;

	fingerBounds.middleMin = middleMin;
	temp = fabs(middleMax-middleMin);
	temp = (temp > 0.0 ? temp : 1.0);
	fingerBounds.middleFactor = 1.0 / temp;

	fingerBounds.ringMin = ringMin;
	temp = fabs(ringMax-ringMin);
	temp = (temp > 0.0 ? temp : 1.0);
	fingerBounds.ringFactor = 1.0 / temp;

	fingerBounds.pinkyMin = pinkyMin;
	temp = fabs(pinkyMax-pinkyMin);
	temp = (temp > 0.0 ? temp : 1.0);
	fingerBounds.pinkyFactor = 1.0 / temp;
}




/****************************
 ****************************
 *	Operations on vectors	*
 ****************************
 ****************************/
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	void unit_vector(VECTOR_COORDS* pV, VECTOR_COORDS* pUV)
//
//	Calculates unit vector in the direction of vector V
//
//	Arguments
//	VECTOR_COORDS* pV	Pointer to the vector
//	VECTOR_COORDS* pUV	Pointer to the unit vector
//
//	Returns
//	Unit vector (via pUV)
//
/////////////////////////////////////////////////////////////////////////////////////////////
void unit_vector(VECTOR_COORDS* pV, VECTOR_COORDS* pUV)
{
	double magnitude = VECTOR_MAGNITUDE(pV->x,pV->y,pV->z);

	// Assign vector to output
	pUV->x = pV->x;
	pUV->y = pV->y;
	pUV->z = pV->z;

	// normalise if vector is not (0,0,0)
	if(magnitude != 0.0)
	{
		pUV->x /= magnitude;
		pUV->y /= magnitude;
		pUV->z /= magnitude;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	void normal_to_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, VECTOR_COORDS* pNormal)
//
//	Finds the normal to two vectors
//
//	Arguments
//	VECTOR_COORDS* pV1		Pointers to the vectors 
//	VECTOR_COORDS* pV2
//	VECTOR_COORDS* pNormal	A pointer to the coordinates of the normal unit vector
//
//	Returns
//	Coordinates of the normal to the vectors (via VECTOR_COORDS* normal)
//
//	Comments
//	The order in which the vectors are sent determines the direction of the perpendicular
//	according to a right handed coordinate system: the normal unit vector is found 
//	by means of the cross-product V1 x V2.
//	(The cross product V2 x V1 would produce a vector pointing in the opposite direction).
//
/////////////////////////////////////////////////////////////////////////////////////////////
void normal_to_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, VECTOR_COORDS* pNormal)
{
	double magnitude;

	// Find the cross product V1 x V2
	pNormal->x = pV1->y*pV2->z - pV1->z*pV2->y;
	pNormal->y = pV1->z*pV2->x - pV1->x*pV2->z;
	pNormal->z = pV1->x*pV2->y - pV1->y*pV2->x;

	// Find magnitude and convert to unit vector (avoid division by zero)
	magnitude = sqrt(pNormal->x*pNormal->x + pNormal->y*pNormal->y + pNormal->z*pNormal->z);
	if(magnitude != 0.0)
	{
		pNormal->x /= magnitude;
		pNormal->y /= magnitude;
		pNormal->z /= magnitude;
	}
	else	// if the vector is zero, use unit vector int he Y direction
		pNormal->y = 1.0;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void angle_of_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, double *pAngle)
//
//	Finds the angle between two vectors
//
//	Arguments
//	VECTOR_COORDS* pV1		Pointers to the vectors 
//	VECTOR_COORDS* pV2
//	double *pAngle			Pointer to the value of the rotation angle
//
//	Returns
//	Angle between vectors (via pAngle)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void angle_of_vectors(VECTOR_COORDS* pV1, VECTOR_COORDS* pV2, double *pAngle)
{
	double denom = VECTOR_MAGNITUDE(pV1->x, pV1->y, pV1->z) * VECTOR_MAGNITUDE(pV2->x, pV2->y, pV2->z);

	/* Avoid division by zero */
	if(denom == 0)
		*pAngle = 0;
	else /* calculate the inverse cosine normalised dot product */
		*pAngle = acos((pV1->x*pV2->x + pV1->y*pV2->y + pV1->z*pV2->z)/denom);
}

/****************************
 ****************************
 *			Planes			*
 ****************************
 ****************************/
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void normal_to_plane(VECTOR_COORDS* pP1, VECTOR_COORDS* pP2, VECTOR_COORDS* pP3, VECTOR_COORDS* pNormal)
//
//	Finds the Normal for a plane containing three points
//
//	Arguments
//	VECTOR_COORDS* pP1		Pointers to three points forming a plane 
//	VECTOR_COORDS* pP2
//	VECTOR_COORDS* pP3
//	VECTOR_COORDS* pNormal	A pointer to the coordinates of the normal unit vector
//
//	Returns
//	Coordinates of normal to the plane (via pNormal)
//
//	Comments
//	The order in which the points are sent determines the direction of the normal vector
//	according to a right handed coordinate system in which the plane is formed by
//	vectors pointing out from P2 to P1 (V1 = P1-P2) and from P2 to P3 (V2 = P3-P2) 
//	and the normal unit vector is found by means of the cross-product V1 x V2.
//	(The cross product V2 x V1 would produce a vector pointing in the opposite direction).
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void normal_to_plane(VECTOR_COORDS* pP1, VECTOR_COORDS* pP2, VECTOR_COORDS* pP3, VECTOR_COORDS* pNormal)
{
	VECTOR_COORDS v1, v2;
//	double magnitude;

	// Construct the vectors
	v1.x = pP1->x - pP2->x;
	v1.y = pP1->y - pP2->y;
	v1.z = pP1->z - pP2->z;

	v2.x = pP3->x - pP2->x;
	v2.y = pP3->y - pP2->y;
	v2.z = pP3->z - pP2->z;

	// calculate normal between vectors
	normal_to_vectors(&v1, &v2, pNormal);
}



/************************************
 ************************************
 *	Implementation of Rotations		*
 ************************************
 ************************************/

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_X(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world X axis
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double angle			Rotation angle
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_X(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
{
	// calculate cosines and sine
	double cx = cos(angle);
	double sx = sin(angle);

	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an X rotation:
	//
	//		1	0	0
	//		0	cx	-sx
	//		0	sx	cx
	//
	////////////////////////////////////////////
	pVout->x = pVin->x;
	pVout->y = pVin->y * cx - pVin->z * sx;
	pVout->z = pVin->y * sx + pVin->z * cx;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_Xcs(VECTOR_COORDS* pVin, double cx, double sx, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world Y axis when the cosine and sine are given instead of the angle
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double cx				Cosine of rotation angle
//	double Sx				Sine of rotation angle
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_Xcs(VECTOR_COORDS* pVin, double cx, double sx, VECTOR_COORDS* pVout)
{
	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an X rotation:
	//
	//		1	0	0
	//		0	cx	-sx
	//		0	sx	cx
	//
	////////////////////////////////////////////
	pVout->x = pVin->x;
	pVout->y = pVin->y * cx - pVin->z * sx;
	pVout->z = pVin->y * sx + pVin->z * cx;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_Y(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world Y axis
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double angle			Rotation angle
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_Y(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
{
	// calculate cosines and sine
	double cy = cos(angle);
	double sy = sin(angle);

	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an Y rotation:
	//
	//		cy	0	sy
	//		0	1	0
	//		-sy	0	cy
	//
	////////////////////////////////////////////
	pVout->x = pVin->x * cy + pVin->z * sy;
	pVout->y = pVin->y;
	pVout->z = -pVin->x * sy + pVin->z * cy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_Ycs(VECTOR_COORDS* pVin, double cy, double sy, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world Y axis when the cosine and sine are given instead of the angle
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double cy				Cosine of rotation angle
//	double sy				Sine of rotation angle
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_Ycs(VECTOR_COORDS* pVin, double cy, double sy, VECTOR_COORDS* pVout)
{
	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an Y rotation:
	//
	//		cy	0	sy
	//		0	1	0
	//		-sy	0	cy
	//
	////////////////////////////////////////////
	pVout->x = pVin->x * cy + pVin->z * sy;
	pVout->y = pVin->y;
	pVout->z = -pVin->x * sy + pVin->z * cy;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_Z(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world Z axis
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double angle			Rotation angle
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_Z(VECTOR_COORDS* pVin, double angle, VECTOR_COORDS* pVout)
{
	// calculate cosines and sine
	double cz = cos(angle);
	double sz = sin(angle);

	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an Z rotation:
	//
	//		cz	-sz	0
	//		sz	cz	0
	//		0	0	1
	//
	////////////////////////////////////////////
	pVout->x = pVin->x * cz - pVin->y * sz;
	pVout->y = pVin->x * sz + pVin->y * cz;
	pVout->z = pVin->z;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_vector_Zcs(VECTOR_COORDS* pVin, double cz, double sz, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to the world Z axis when the cosine and sine are given instead of the angle
//
//	Arguments
//	VECTOR_COORDS* pVin		Pointer to the vector to be rotated 
//	double cz				Cosine of the angle of rotation
//	double sz				Sine of the angle of rotation
//	VECTOR_COORDS* pVout	Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_vector_Zcs(VECTOR_COORDS* pVin, double cz, double sz, VECTOR_COORDS* pVout)
{
	////////////////////////////////////////////
	//	Apply the following rotation matrix
	//	for an Z rotation:
	//
	//		cz	-sz	0
	//		sz	cz	0
	//		0	0	1
	//
	////////////////////////////////////////////
	pVout->x = pVin->x * cz - pVin->y * sz;
	pVout->y = pVin->x * sz + pVin->y * cz;
	pVout->z = pVin->z;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_axis_origin(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to an arbitrary axis passing through the origin: (0, 0, 0)
//
//	Arguments
//	VECTOR_COORDS* pVin			Pointer to the vector to be rotated 
//	VECTOR_COORDS* pAxisVector	Vector pointing in the direction of the axis
//	double angle				Rotation angle
//	VECTOR_COORDS* pVout		Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
//	Comments
//	This function uses an algorithm described by Glenn Murray (c) 2005, 
//	http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/ (accessed 27 March 2009):
//
//	(1) rotate space about the Z axis so that the rotation axis lies in the XZ plane
//	(2) rotate space about the Y axis so that the rotation axis lies along the Z axis
//	(3) perform the desired rotation by 'angle' about the Z axis
//	(4) apply the inverse of step (2)
//	(5) apply the inverse of step (1)
//
//	NOTE: This method diverges if the axis is along the Z axis, in which casem a direct Z rotation may be used.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_axis_origin(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout)
{
//static count = 100;
//count--;


//if(count ==0)
//{
//	printf("\nrotate_axis_origin: Input: (%6.3lf , %6.3lf, %6.3lf)\n", pVin->x, pVin->y, pVin->z);
//	printf("rotate_axis_origin: Axis: (%6.3lf , %6.3lf, %6.3lf)\n", pAxisVector->x, pAxisVector->y, pAxisVector->z);
//}
	//
	//	(0) Use simple method if the axis is along the Z direction
	//
	if(pAxisVector->x == 0 && pAxisVector->y == 0)
	{
		// If the axis is zero, do not rotate
		if(pAxisVector->z == 0)
		{
			pVout->x = pVin->x;
			pVout->y = pVin->y;
			pVout->z = pVin->z;

			return;
		}
		else
		{
			rotate_vector_Z(pVin, -angle, pVout);
//if(count ==0)
//printf("Rotated vector by 'angle' about desired Axis: (%6.3lf , %6.3lf, %6.3lf))\n\n", pVout->x, pVout->y, pVout->z);
			return;
		}
	}


	//
	//	(1) rotate space about the Z axis so that the rotation axis lies in the XZ plane
	//
	
	// Calculate cosine and sine of the Axis Vector azimuth angle
	double cz, sz;
	VECTOR_COORDS vOut1;
	double xy = sqrt(pAxisVector->x * pAxisVector->x + pAxisVector->y * pAxisVector->y);

	if(xy == 0)
	{
		cz = 1;
		sz = 0;
	}
	else
	{
		cz = pAxisVector->x / xy;
		sz = -pAxisVector->y / xy; // rotation is in the negative direction
	}

	// Rotate Axis Vector about the Z axis
	rotate_vector_Zcs(pVin, cz, sz, &vOut1);

	//
	//	(2) rotate space about the Y axis so that the rotation axis lies along the Z axis
	//

	// Calculate cosine and sine of the Axis Vector elevation angle
	double cy, sy;
	double denom = VECTOR_MAGNITUDE(pAxisVector->x,pAxisVector->y,pAxisVector->z);
	VECTOR_COORDS vOut2;

	if(denom ==0)
	{
		cy = 1;
		sy = 0;
	}
	else
	{
		cy = pAxisVector->z / denom;
		sy = -xy / denom;	// rotation is in the negative direction
	}

	// Rotate vOut1 about the Y axis
	rotate_vector_Ycs(&vOut1, cy, sy, &vOut2);

	//
	//	(3) perform the desired rotation by 'angle' about the Z axis
	//
	VECTOR_COORDS vOut3;
	rotate_vector_Z(&vOut2, -angle, &vOut3);

	//
	//	(4) apply the inverse of step (2)
	//

	// Rotate vOut3 about the Y axis using the negative of the angle (negative sine)
	rotate_vector_Ycs(&vOut3, cy, -sy, &vOut2);

	//
	//	(5) apply the inverse of step (1)
	//
	// Rotate vOut2 about the Z axis using the negative of the angle (negative sine)
	rotate_vector_Zcs(&vOut2, cz, -sz, pVout);


//if(count ==0)
//printf("Rotated vector by 'angle' about desired Axis: (%6.3lf , %6.3lf, %6.3lf))\n\n", pVout->x, pVout->y, pVout->z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_axis_origin_alt(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout)
//
//	Alternative vector rotation with respect to an arbitrary axis passing through the origin: (0, 0, 0)
//	Taken from p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
	/*
	 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
	 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
	 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
	 *
	 *  USB P5 Data Glove support
	 */
//////////////////////////////////////////////////////////////////////
//	Arguments
//	VECTOR_COORDS* pVin			Pointer to the vector to be rotated 
//	VECTOR_COORDS* pAxisVector	Vector pointing in the direction of the axis.
//	double angle				Rotation angle
//	VECTOR_COORDS* pVout		Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
//	Comments
//	This function uses an algorithm described by Glenn Murray (c) 2005, 
//	http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/ (accessed 27 March 2009):
//
//	(1) rotate space about the Z axis so that the rotation axis lies in the XZ plane
//	(2) rotate space about the Y axis so that the rotation axis lies along the Z axis
//	(3) perform the desired rotation by 'angle' about the Z axis
//	(4) apply the inverse of step (2)
//	(5) apply the inverse of step (1)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_axis_origin_alt(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, double angle, VECTOR_COORDS* pVout)
{
	double c, s, t, x, y, z, rot_matrix[4][4];
	double inVec[3], outVec[3];

//static count = 100;
//count--;

	// convert Axis to unit vector
	VECTOR_COORDS uv;
	unit_vector(pAxisVector, &uv);

	inVec[0] = pVin->x;
	inVec[1] = pVin->y;
	inVec[2] = pVin->z;

//if(count ==0)
//{
//printf("\nrotate_axis_origin_alt: Input: (%6.3lf , %6.3lf, %6.3lf)\n", pVin->x, pVin->y, pVin->z);
//printf("rotate_axis_origin_alt: Axis: (%6.3lf , %6.3lf, %6.3lf)\n", pAxisVector->x, pAxisVector->y, pAxisVector->z);
//printf("rotate_axis_origin_alt: Axis UNIT VECTOR: (%6.3lf , %6.3lf, %6.3lf)\n", uv.x, uv.y, uv.z);
//}
	c=cos(angle);
	s=sin(angle);
	t=1.0-c;

	/* Calculate transformation matrix, in column major order */
	x = uv.x;
	y = uv.y;
	z = uv.z;

	/* Column 0 */
	rot_matrix[0][0]=t*x*x+c;
	rot_matrix[0][1]=t*x*y-s*z;
	rot_matrix[0][2]=t*x*z+s*y;
	rot_matrix[0][3]=0;

	/* Column 1 */
	rot_matrix[1][0]=t*x*y+s*z;
	rot_matrix[1][1]=t*y*y+c;
	rot_matrix[1][2]=t*y*z-s*x;
	rot_matrix[1][3]=0;

	/* Column 2 */
	rot_matrix[2][0]=t*x*z-s*y;
	rot_matrix[2][1]=t*y*z+s*x;
	rot_matrix[2][2]=t*z*z+c;
	rot_matrix[2][3]=0;

	/* Column 3 */
	rot_matrix[3][0]=0;
	rot_matrix[3][1]=0;
	rot_matrix[3][2]=0;
	rot_matrix[3][3]=1;

	/* Now, apply the rotation angle to the position */
	p5glove_vec_mat(inVec,rot_matrix,outVec);

	pVout->x = outVec[0];
	pVout->y = outVec[1];
	pVout->z = outVec[2];

//if(count ==0)
//printf("Rotated vector by 'angle' about desired Axis: (%6.3lf , %6.3lf, %6.3lf))\n\n", pVout->x, pVout->y, pVout->z);
}

//	Taken from p5glove.c 67 2005-02-14 16:30:02Z jmcmullan $
	/*
	 *  Copyright (c) 2003 Jason McMullan <ezrec@hotmail.com>
	 *  Windows patch (c) 2004 Ross Bencina <rossb@audiomulch.com>
	 *  MacOSX patch (c) 2004 Tim Kreger <tkreger@bigpond.net.au>
	 *
	 *  USB P5 Data Glove support
	 */
/* Optimized for our use. We know the last row and last column are all zeros,
 * except for [3][3]=1.
 */
void p5glove_vec_mat(double vec[3],double mat[4][4],double res[3])
{
	int r,c;

	for (r=0; r < 3; r++) {
		res[r]=0.0;
		for (c=0; c < 3; c++)
			res[r] += vec[c]*mat[c][r];
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	void rotate_axis(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, VECTOR_COORDS* pAxisOrigin, double angle, VECTOR_COORDS* pVout)
//
//	Rotates vector with respect to an arbitrary axis passing through an arbitrary point
//
//	Arguments
//	VECTOR_COORDS* pVin			Pointer to the vector to be rotated 
//	VECTOR_COORDS* pAxisVector	Vector pointing in the direction of the axis
//	VECTOR_COORDS* pAxisOrigin	Point through which the axis goes through (origin of the vector)
//	double angle				Rotation angle
//	VECTOR_COORDS* pVout		Pointer to the resulting vector after rotation
//
//	Returns
//	Pointer to the resulting vector after rotation (via pVout)
//
//	Comments
//	This function uses an algorithm described by Glenn Murray (c) 2005, 
//	http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/ (accessed 27 March 2009):
//
//	(1) translate space so that the rotation axis passes through the origin
//	(2) rotate space about the Z axis so that the rotation axis lies in the XZ plane
//	(3) rotate space about the Y axis so that the rotation axis lies along the Z axis
//	(4) perform the desired rotation by 'angle' about the Z axis
//	(5) apply the inverse of step (3)
//	(6) apply the inverse of step (2)
//	(7) apply the inverse of step (1)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_axis(VECTOR_COORDS* pVin, VECTOR_COORDS* pAxisVector, VECTOR_COORDS* pAxisOrigin, double angle, VECTOR_COORDS* pVout)
{
	//
	//	(1) translate space so that the rotation axis passes through the origin
	//
	VECTOR_COORDS uvIn, uvOut;
	
	uvIn.x = pAxisVector->x - pAxisOrigin->x;
	uvIn.y = pAxisVector->y - pAxisOrigin->y;
	uvIn.z = pAxisVector->z - pAxisOrigin->z;


	//
	//	Perform steps (2) to (6)
	//
	rotate_axis_origin(pVin, &uvIn, angle, &uvOut);

	
	//
	//	(7) apply the inverse of step (1)
	//
	pVout->x = uvOut.x + pAxisOrigin->x;
	pVout->y = uvOut.y + pAxisOrigin->y;
	pVout->z = uvOut.z + pAxisOrigin->z;
}

// unwarping matrix for the P5 glove, version 2
// Ross Bencina <rossb@audiomulch.com> 26th February, 2004
//
// this version is about as good as it gets without separately
// modelling the Z axis distortion that the glove seems to 
// introduce when merging signals from the two cameras.
//
// massage the coordinates into something relatively usable.
// usage example: p5_matrix( &info.ir[i].x, &info.ir[i].y, &info.ir[i].z );
void p5_matrix( int *x_, int *y_, int *z_ )
{
    // rotation coefficients
    static const float st_yz = 0.6730125; // sin( M_PI * .235 );
    static const float ct_yz = 0.739631; // cos( M_PI * .235 );
    // unit cube scaler
    static const float s = 1. / 512.;

    float x = *x_;
    float y = *y_;
    float z = *z_;

    // scale to unit cube
    x *= s; y *= s; z *= s;

    // rotate  y, z
    {
        float yy = y, zz = z;
        y = yy * ct_yz + zz * st_yz;
        z = zz * ct_yz - yy * st_yz;
    }

    y += 0.19;       // translate y
    z += 3.312;     // translate z


	// fisheye correction
	float zfocal = z - 0.8;
	if( zfocal > 0. )
	    z *= zfocal / sqrt( y*y + zfocal*zfocal ); // alternate form of: cos( atan( y / zfocal) )
	
	// distance correct z
	z = pow(fabs(z),8.4) * 0.0003385305;
	
	// perspective correct x
	x *= (31.5 + pow(z,1.7)) * 0.063096;
	
	//perspective correct y
	y *= (4.5 + z) * 0.239;
	
	// scale z
	z *= 0.478;

 // x, y ~(-2.5, 2.5), z ~(0, 5)
 // remap x, y and z to +/-512
    *x_ = x * 200.;
    *y_ = y * 200.;
    *z_ = (z * 200) - 512;
}
