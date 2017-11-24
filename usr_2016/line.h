/***************************************************************************

    file                 : line.h
    created              : Aug 31, 2007
    copyright            : (C) 2007 John Isham
    email                : isham.john@gmail.com
    version              : $Id: driver.h,v 1.12 2006/03/06 22:43:50 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _LINE_H_
#define _LINE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
//#include <portability.h>

#include "linalg.h"

#define TRACK_DATA "track.dat"
#define CLOTH_DATA "cloth.dat"

#define LINE0_DATA  "line0.dat"
#define LINE1_DATA  "line1.dat"
#define LINE2_DATA  "line2.dat"
#define LINE3_DATA  "line3.dat"
#define LINE4_DATA  "line4.dat"
#define LINE5_DATA  "line5.dat"
#define LINE6_DATA  "line6.dat"
#define LINE7_DATA  "line7.dat"
#define LINE8_DATA  "line8.dat"
#define LINE9_DATA  "line9.dat"

class Line {
	public:
		Line();
		~Line();

		void setCar( tCarElt *car ) { Line::my_car = car; }

		void InitTrack(tTrack* track, tSituation *p);

		void GetRaceLineData(tSituation *s, v2d *target, 
				     float *speed, float *avspeed, 
				     float *raceoffset, float *lookahead, 
				     float *racesteer);

		vec2f getTargetPoint( int line,
				      float fromstart, 
				      float offset );

		float lineRadius(int line, float fromstart);

	private:
		tCarElt *my_car;
		float my_car_width;

		void initCloth(void);

#define DT 0.001f
#define T_MAX (sqrt( 2.0 * PI ))
#define T_SIZE  6284    //6284

		float cloth_s[T_SIZE], cloth_c[T_SIZE];

		float cloth_x(float t);
		float cloth_y(float t);

		void scanTrack(tTrack* t, float car_width);
		void refineLine(tTrack* t, int pass);

		void customizeLine(tTrack* t, int pass);

		// Utility functions.
		void dumpTrack(tTrack* t);
		void dumpTrackLeft(FILE *tfile, tTrack* t);
		void dumpTrackRight(FILE *tfile, tTrack* t);


		void dumpCloth(char *filename);

		typedef struct {
  
		  int type;
		  tTrackSeg *startseg, *endseg;
		  int ttype;
		  tTrackSeg *t_start, *t_end;
		  
		  float length;
		  float arc;
		  float radius;
		  float fromstart;

		  vec2f startpt, endpt;
		  vec2f t_startpt, t_endpt;

		  /* first apex */
		  vec2f apex;
		  vec2f path_center;
		  float path_radius;

		  /* clothoid params */
		  float apex1_t, entry_t, entry_a, entry_s, run_in, start_rot, sign_ex, sign_ey;
		  float apex2_t, exit_t, exit_a, exit_s, run_out, end_rot, sign_xx, sign_xy;

		  /* second apex */
		  vec2f apex2, midpoint;
		  vec2f path2_center;
		  float path2_radius;

		} lineseg_t;

/* 60 is NOT enought with Spring track :too much Seg --> Overflow!
//
// need MAX_LINE_SEGS  180  <-- SO bad doesnt works!
//  */
#define MAX_LINE_SEGS  60 //
#define MAX_LINE_PASS   4

		lineseg_t lseg[MAX_LINE_PASS][MAX_LINE_SEGS];
		int num_lseg[MAX_LINE_PASS];
		tTrack *mytrack;

		void addSegStraight(int index,
				    tTrackSeg *start,
				    tTrackSeg *end,
				    float len );

		void addSegKink(int index,
				tTrackSeg *start,
				tTrackSeg *end,
				float arc );

		void addSegCR(int index, 
			      tTrackSeg *start,
			      tTrackSeg *end,
			      float arc,
			      float radius);

		void addSegCRDouble(int index,
				    tTrackSeg *start,
				    tTrackSeg *end,
				    float arc,
				    float radius);

		void addSegUnknown(int index,
				   tTrackSeg *start,
				   tTrackSeg *end);

#if 0
		bool isKink( float radius, float width, float arc );
#endif
		/* find inside */
		vec2f findApex(tTrackSeg *start,
			       tTrackSeg *end,
			       float arc );

		vec2f findOutside(tTrackSeg *start,
				  tTrackSeg *end,
				  float arc );

		void circle3point( vec2f point1,
				   vec2f point2,
				   vec2f point3,
				   vec2f *center,
				   float *radius);


		/* ack, talk about creep.  This started off simple 
		 * and has become a beast.  This should really
		 * become a struct of params that is passed around,
		 * whenever I get time to clean it up.
		 */
		void findClothoid(vec2f start, 
				  vec2f apex,
				  float angle,
				  float rotation,
				  float *sign_x,
				  float *sign_y,
				  float *apex_t,
				  float *entry_t,
				  float *a,
				  float *len,
				  float *run_in);


		void dumpLine(char *filename, int index);
};

#endif // _LINE_H_

