/*

	P5dumpRF.c

	Tracks P5 glove data module. 
	Copyright (c) 2009 Rajmil Fischman <r.a.fischman@keele.ac.uk>

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




#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h> /* exit */
#include <string.h> /* memset */
#include "hidapi.h"
#include "p5glove.h"
#include "P5gloveUtils.h"


/*****************************

 *	Function Declarations

 *****************************/

int calibrateAxes(P5GlovePtr p5, struct p5glove_data *pInfo);




/* 
	P5 data dump.
 */
int main(int argc, char **argv)
{
	P5GlovePtr glove;
	struct p5glove_data info;
	int sample;
	int i,j,k,err;


	/*******************

	  Initialise glove 

	********************/

	glove=p5glove_open();
	if (glove == NULL) {
		fprintf(stderr, "%s: Can't open glove interface\n", argv[0]);
		return 1;
	}

	/******************
	  Disable Mouse 
	*******************/
	p5glove_mouse_mode_off(glove);


	/*****************************************************
		Get glove calibration values, including
		zero sensor positions
	******************************************************/
	printf("Calbrating glove...\n");

	p5glove_calibrate(glove);



	// Print the zero positions

	printf("Calibrated zero position of the sensors:\n"); 

	for(i=0 ; i<8 ; i++)

	{

		printf("\t%2d\t%7.4f\t%7.4f\t%7.4f\n", i, 

			glove->cal.irZeroPos[i].x, glove->cal.irZeroPos[i].y, glove->cal.irZeroPos[i].z);

	}

	printf("\n"); 


	
	/*******************************************************************************
		Calculate the orientations of the normal vectors of the planes formed by
		each group of three sensors
	********************************************************************************/
	printf("Calculating plane normals...\n");
	calculatePlanes(glove);



	

	/*******************************************************************************

		Calibrate user reach

	********************************************************************************/



	if(calibrateAxes(glove, &info) == P5GLOVE_BUTTON_B)

		printf("User defined reach\n");

	else	// Initialise clipping volume with defaults

		printf("Default reach\n");



	printf("clipping distances\n");

	printf("left=%5.2f right=%5.2f top=%5.2f bottom=%5.2f front=%5.2f rear=%5.2f\n", 

		clippingCube.left,clippingCube.right,clippingCube.top, clippingCube.bottom, clippingCube.front, clippingCube.rear);

	printf("halfXClipping=%5.2f halfYClipping=%5.2f halfZClipping=%5.2f\n",

		halfXClipping, halfYClipping, halfZClipping);



	printf("\n");



	/****************
	 * BEGIN DUMP
	 ****************/
	printf("GLOVE DATA\n");
	memset(&info,0,sizeof(info));
	for (sample=0; ; ) 
	{
		// Exit if button C is pressed

		if(info.buttons & P5GLOVE_BUTTON_C)

			break;



		// get glove sample and check for errors

		err=p5glove_sample(glove,&info);
		if (err < 0 && errno == EAGAIN)
			continue;
		else if (err < 0) // serious error, get out!

		{

			perror("Glove Failure");

			exit(1);

		}



		// Increase sample count

		sample++;
		


		/*****************************************************************
			Find glove position and orientation

			These will be stored respectively in:



			VECTOR_COORDS info.position (coordinates position in meters)

			VECTOR_COORDS info.positionN (position normalised to [-1,1])

			VECTOR_COORDS info.orientation

		 

		******************************************************************/
		glovePositionOrientation(glove, &info);



		// Print glove position and orientation

		printf("\rsample: %d ABS POSITION:(%6.3lf, %6.3lf, %6.3lf) NORM POSITION:(%6.3lf, %6.3lf, %6.3lf) ORIENTATION:(%6.3lf, %6.3lf, %6.3lf)",

					sample, info.position.x,info.position.y,info.position.z,

					info.positionN.x,info.positionN.y,info.positionN.z,

					info.orientation.x,info.orientation.y,info.orientation.z);

		//Print pressed buttons

		if(info.buttons & P5GLOVE_BUTTON_A)

			printf(" A");

		else

			printf("  ");

		if(info.buttons & P5GLOVE_BUTTON_B)

			printf(" B");

		else

			printf("  ");

		if(info.buttons & P5GLOVE_BUTTON_C)

			printf(" C");

		else

			printf("  ");





//printf("\rIR%1d:(%6.3lf, %6.3lf, %6.3lf) IR%1d:(%6.3lf, %6.3lf, %6.3lf) IR%1d:(%6.3lf, %6.3lf, %6.3lf) Axis:(%6.3lf, %6.3lf, %6.3lf) Angle = %6.3lf",

//					i,info.ir[i].currentPos.x,info.ir[i].currentPos.y,info.ir[i].currentPos.z,

//					j,info.ir[j].currentPos.x,info.ir[j].currentPos.y,info.ir[j].currentPos.z,

//					k,info.ir[k].currentPos.x,info.ir[k].currentPos.y,info.ir[k].currentPos.z,

//					axis.x, axis.y, axis.z, angle);

				



		/* Buttons */

/*		printf("%c%c%c ",

			(info.buttons & P5GLOVE_BUTTON_A) ? 'A' : '.',

			(info.buttons & P5GLOVE_BUTTON_B) ? 'B' : '.',

			(info.buttons & P5GLOVE_BUTTON_C) ? 'C' : '.');

*/

		/* Fingers */

/*		printf("%2d,%2d,%2d,%2d,%2d ",

			info.finger[P5GLOVE_THUMB],

			info.finger[P5GLOVE_INDEX],

			info.finger[P5GLOVE_MIDDLE],

			info.finger[P5GLOVE_RING],

			info.finger[P5GLOVE_PINKY]);

*/

	

	}

	p5glove_close(glove);

    return 0;
}



//

//	int calibrateAxes(P5GlovePtr p5, struct p5glove_data *pInfo)

//	

//	Calibrates distances of axes according to users position

//

//	Button A begins/restarts calibration

//	Button B saves values

//	Button C exits calibration

//

//	Returns:

//			P5GLOVE_BUTTON_A	if A was pressed

//			P5GLOVE_BUTTON_B	if B was pressed

//			P5GLOVE_BUTTON_C	if C was pressed

//			P5GLOVE_NO_BUTTON	if no button was pressed

//

int calibrateAxes(P5GlovePtr p5, struct p5glove_data *pInfo)

{

	double left, right, top, bottom, rear, front;

	int first = 1;

	int err;

	

	



	/********************************

		Set default boundaries

	*********************************/



	setClippingCube(P5GLOVE_X_CLIPPING_MIN_M, P5GLOVE_X_CLIPPING_MAX_M, 

					P5GLOVE_Y2_CLIPPING_MAX_M, P5GLOVE_Y2_CLIPPING_MIN_M, 

					P5GLOVE_Z_CLIPPING_MIN_M, P5GLOVE_Z_CLIPPING_MAX_M);



	// Print user message

	printf("\n--------------------------------------------------\n");

	printf("\tDistance Calibration\n\n");

	printf("\tUse P5 glove BUTTONS at follows:\n");

	printf("\t\t'A'\tto start calibration\n");	

	printf("\t\t'B'\tto save each calibration value\n");

	printf("\t\t'C'\tto skip or end calibration\n");

	printf("--------------------------------------------------\n\n");



	/********************************

		wait for a button press

	*********************************/

	

	// filter out button B

	while(pInfo->buttons & P5GLOVE_BUTTON_B)

	{

		// get glove sample and check for errors

		err=p5glove_sample(p5,pInfo);

		if (err < 0 && errno == EAGAIN)

			continue;

		else if (err < 0) // serious error, get out!

			return errno;

	}



	// exit if button C was pressed

	if(pInfo->buttons & P5GLOVE_BUTTON_C)

	{

		// flush out button C

		while(pInfo->buttons & P5GLOVE_BUTTON_C)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}



		return P5GLOVE_BUTTON_C;

	}



	//

	// Carry out loop until calibration is completed or the user requests to exit

	//

	while(1)

	{

		if(first)

			first = 0;

		else

		{

			printf("\n--------------------------------------------------\n");

			printf("\tDistance Calibration\n\n");

			printf("\tUse P5 glove BUTTONS at follows:\n");

			printf("\t\t'A'\tto start calibration\n");	

			printf("\t\t'B'\tto save each calibration value\n");

			printf("\t\t'C'\tto skip or end calibration\n");

			printf("--------------------------------------------------\n\n");

		}



		///////////////////////////

		// CALIBRATE LEFT EDGE

		//

		printf("\rMove to   LEFT edge and press 'B'.   Left position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to   LEFT edge and press 'B'.   Left position = %6.3f: ", pInfo->position.x);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			left = pInfo->position.x;



		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////

		// CALIBRATE RIGHT EDGE

		//

		printf("\rMove to  RIGHT edge and press 'B'.  Right position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to  RIGHT edge and press 'B'.  Right position = %6.3f: ", pInfo->position.x);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			right = pInfo->position.x;



		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////

		// CALIBRATE TOP EDGE

		//

		printf("\rMove to    TOP edge and press 'B'.    Top position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to    TOP edge and press 'B'.    Top position = %6.3f: ", pInfo->position.y);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			top = pInfo->position.y;



		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////

		// CALIBRATE BOTTOM EDGE

		//

		printf("\rMove to BOTTOM edge and press 'B'. BOTTOM position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to BOTTOM edge and press 'B'. BOTTOM position = %6.3f: ", pInfo->position.y);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			bottom = pInfo->position.y;



		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////

		// CALIBRATE REAR EDGE

		//

		printf("\rMove to REAR edge (away from tower) and press 'B'.   REAR position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to REAR edge (away from tower) and press 'B'.   REAR position = %6.3f: ", pInfo->position.z);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			rear = pInfo->position.z;

		

		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////

		// CALIBRATE FRONT EDGE

		//

		printf("\rMove to FRONT edge (towards the tower) and press 'B'.  FRONT position");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			/*****************************************************************

				Find glove position and orientation

				These will be stored respectively in:



				VECTOR_COORDS info.position (coordinates position in meters)

				VECTOR_COORDS info.positionN (position normalised to [-1,1])

				VECTOR_COORDS info.orientation

			******************************************************************/

			glovePositionOrientation(p5, pInfo);

		

			// print current position

			printf("\rMove to FRONT edge (towards the tower) and press 'B'.  FRONT position = %6.3f: ", pInfo->position.z);



		}



		// act according to button press

		if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

		{

			// flush out button D

			while(pInfo->buttons & P5GLOVE_BUTTON_D)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

		{

			// flush out button C

			while(pInfo->buttons & P5GLOVE_BUTTON_C)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}



			return P5GLOVE_BUTTON_C;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

		{

			// flush out button A

			while(pInfo->buttons & P5GLOVE_BUTTON_A)

			{

				// get glove sample and check for errors

				err=p5glove_sample(p5,pInfo);

				if (err < 0 && errno == EAGAIN)

					continue;

				else if (err < 0) // serious error, get out!

					return errno;

			}

			continue;

		}

		else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			front = pInfo->position.z;



		// flush button B presses

		while(pInfo->buttons & P5GLOVE_BUTTON_B)

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

				return errno;

		}

		printf("\n");



		////////////////////////////////////////

		// Act on users' decision

		//

		printf("Calibration values: LEFT = %6.3f  RIGHT = %6.3f  TOP = %6.3f  BOTTOM = %6.3f  REAR =%6.3f  FRONT = %6.3f\n",

				left, right, top, bottom, rear, front);

		printf("Save Values?\n");

		printf("\t\t'A'\tto restart calibration\n");	

		printf("\t\t'B'\tto SAVE values\n");

		printf("\t\t'C'\tto exit without saving\n");



		// wait until the user acts

		for( pInfo->buttons = P5GLOVE_NO_BUTTON; pInfo->buttons == P5GLOVE_NO_BUTTON; )

		{

			// get glove sample and check for errors

			err=p5glove_sample(p5,pInfo);

			if (err < 0 && errno == EAGAIN)

				continue;

			else if (err < 0) // serious error, get out!

			{

				perror("Glove Failure");

				exit(1);

			}	



			// act according to button press

			if(pInfo->buttons & P5GLOVE_BUTTON_D)		// do nothing

			{

				// flush out button D

				while(pInfo->buttons & P5GLOVE_BUTTON_D)

				{

					// get glove sample and check for errors

					err=p5glove_sample(p5,pInfo);

					if (err < 0 && errno == EAGAIN)

						continue;

					else if (err < 0) // serious error, get out!

						return errno;

				}

			}

			else if(pInfo->buttons & P5GLOVE_BUTTON_C)		// exit

			{

				// flush out button C

				while(pInfo->buttons & P5GLOVE_BUTTON_C)

				{

					// get glove sample and check for errors

					err=p5glove_sample(p5,pInfo);

					if (err < 0 && errno == EAGAIN)

						continue;

					else if (err < 0) // serious error, get out!

						return errno;

				}



				return P5GLOVE_BUTTON_C;

			}

			else if(pInfo->buttons & P5GLOVE_BUTTON_A)	// restart the process

			{

				// flush out button A

				while(pInfo->buttons & P5GLOVE_BUTTON_A)

				{

					// get glove sample and check for errors

					err=p5glove_sample(p5,pInfo);

					if (err < 0 && errno == EAGAIN)

						continue;

					else if (err < 0) // serious error, get out!

						return errno;

				}

				continue;

			}

			else if(pInfo->buttons & P5GLOVE_BUTTON_B)	// save value and continue

			{

				// set clipping cube

				setClippingCube(left, right, top, bottom, front, rear); 



				// flush button B presses

				while(pInfo->buttons & P5GLOVE_BUTTON_B)

				{

					// get glove sample and check for errors

					err=p5glove_sample(p5,pInfo);

					if (err < 0 && errno == EAGAIN)

						continue;

					else if (err < 0) // serious error, get out!

						return errno;

				}



				return P5GLOVE_BUTTON_B;

			}



		}

		printf("\n");

	}



	return P5GLOVE_NO_BUTTON;

}



