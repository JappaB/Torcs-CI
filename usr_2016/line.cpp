/***************************************************************************

    file                 : line.cpp
    created              : Aug 31, 2007
    copyright            : (C) 2007 John Isham
    email                : isham.john@gmail.com
    version              : $Id: driver.cpp,v 1.16 2006/04/27 22:32:29 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   Note:  According to the torcs racing board endurance racing rules for *
 *   The 2008 season, you shouldn't be copying this code anyway.           *
 *                                                                         *
 ***************************************************************************/
#include "line.h"

#define SCAN_VERBOSE 0
#define SCAN_DEBUG   0

#define DUMP_TRACK   0
#define DUMP_LINE0   0
#define DUMP_LINE1   0
#define DUMP_CLOTH_TAILS 0

#define HACKS 0

#define TARG_DEBUG   0

#define WIDTH_FACT 1.0

Line::Line()
{
  Line::initCloth();
}

Line::~Line()
{
//#
}

#define RAD_TO_DEG ( 180.0 / PI )

#define TURN_NULL 0
#define TURN_CR   1
#define TURN_IR   2
#define TURN_DR   3
#define TURN_S    4
#define TURN_X    5

#define SEG_UNK  0
#define SEG_STR  1
#define SEG_KINK 3
#define SEG_CL   6
#define SEG_CLEN 7
#define SEG_CLEX 8

/*============================================================================*/

void Line::InitTrack(tTrack* track, tSituation *p)
{
  int ref_line;

 /* //////////////////////////////////////////////////////////////////////// */

  /* FIXME:  There should be a robust way to read this in at init time 
   car is not always defined at inittrack time, but should be at initrace time */

  //float car_width = Line::my_car->_dimension_y;

#if SCAN_VERBOSE
  #warning "hymie_2016: using car_width of 1.94 to avoid segfault"
#endif
  float car_width = 1.94;

 /* //////////////////////////////////////////////////////////////////////// */

  ref_line = 0;

  Line::scanTrack(track, car_width); /* 0 */

  Line::refineLine(track, ref_line); /* 0->1 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 1->2 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 2->3 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 3->4 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 4->5 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 5->6 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 6->7 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 7->8 */
  ref_line++;

  Line::refineLine(track, ref_line); /* 8->9 */
  ref_line++;

  //Line::customizeLine(track, ref_line);
}


/**********************************************************/

// Called for every track change or new race.
void Line::scanTrack(tTrack* t, float car_width )
{
  tTrack *track;

  my_car_width = car_width;
  track = t;
  mytrack = t;

#if SCAN_VERBOSE
  char *typestr[] = { (char *)"NULL", (char *)"R", (char *)"L", (char *)"S" };
  char *turnstr[] = { (char *)"NULL", (char *)"cr", (char *)"iR", (char *)"DR", (char *)"S", (char *)"X" };
#else
#if SCAN_DEBUG
  char *typestr[] = { (char *)"NULL", (char *)"R", (char *)"L", (char *)"S" };
#endif
#endif

  tTrackSeg *currentseg, *startseg = track->seg;
  currentseg = startseg;

  int turn_type = 0;
  int prev_turn = 0;
  int next_turn = 0;
  float width = 0.0;
  tTrackSeg *start, *end;

  num_lseg[0] = 0;
  num_lseg[1] = 0;

#if SCAN_VERBOSE
  printf("=================================\n");
  printf("Scanning Track\n");
#endif

  /* front straight ?? */
  if ( currentseg->type == TR_STR )
    {
      while ( currentseg->prev->type == TR_STR )
	currentseg = currentseg->prev;

      startseg = currentseg;
    }
  else
    {
      while ( currentseg->type != TR_STR )
	currentseg = currentseg->next;

      startseg = currentseg;
    }

  do 
    {
	 
      if (currentseg->type == TR_STR) 
	{
	  prev_turn = currentseg->prev->type;
	  
	  float str_len = 0.0;

	  start = currentseg;

	  while ( currentseg->type == TR_STR )
	    {
	      str_len += currentseg->length;
	      width = currentseg->width;

#if SCAN_DEBUG
	      printf("%4d %-12s %d(%s) len:%7.2f W:%5.2f\n",
		     currentseg->id,
		     currentseg->name,
		     currentseg->type, typestr[currentseg->type], 
		     str_len,
		     width);
#endif

	      currentseg = currentseg->next;
	    }
	  
	  end = currentseg->prev;
	  next_turn = currentseg->type;


#if SCAN_VERBOSE
	  printf("END OF STR: len:%7.2f W:%5.2f prev: %d(%s) next: %d(%s)\n",
		 str_len,
		 width,
		 prev_turn, typestr[prev_turn],
		 next_turn, typestr[next_turn]);

	  if ( prev_turn != next_turn )
	    {
	      printf("Z STRAIGHT\n");
	    }
#endif

	  addSegStraight( 0, start, end, str_len );
	} 
      else /* turn */
	{
	  prev_turn = currentseg->prev->type;

	  float rad_min, rad_max;
	  int transitions = 0;
	  float turn_arc = 0.0;
	  int lastsegtype = currentseg->type;
	  float dr = 0.0;
	  float last_dr = 0.0;
	  int force_transition = 0;

	  start = currentseg;

	  if ( prev_turn == TR_STR )
	    {
#if SCAN_VERBOSE
	  printf("TURN ENTRY\n");
#endif
	    }

	  rad_min = currentseg->radius;
	  rad_max = rad_min;

	  if ( currentseg->next->type == lastsegtype )
	    last_dr = currentseg->next->radius - currentseg->radius;
	  else
	    last_dr = 0.0;

	  while ( (currentseg->type == lastsegtype) &&
		  ( ! force_transition )
		  )
	    {
	      force_transition = 0;
	      turn_arc += currentseg->arc;
	      width = currentseg->width;

	      if ( currentseg->radius < rad_min )
		rad_min = currentseg->radius;

	      if ( currentseg->radius > rad_max )
		rad_max = currentseg->radius;

	      if ( currentseg->next->type == lastsegtype )
		{
		  dr = currentseg->next->radius - currentseg->radius;

		  if ( fabs(dr) < 1.0 )
		    turn_type = TURN_CR;

		  else if ( dr < 0.0 )
		    turn_type = TURN_DR;

		  else
		    turn_type= TURN_IR;
		}
	      if ( dr > currentseg->radius )
		{
#if SCAN_VERBOSE
		  printf("FORCING TRANSITION\n");
#endif
		  force_transition = 1;
		}
#if HACKS
	      if ( currentseg->id == 368 )
		{
#if SCAN_VERBOSE
		  printf("Hack: forcing transition at seg:%d\n",currentseg->id);
#endif
		  force_transition = 1;
		}
#endif
#if 0
	      //if ( currentseg->length > (2.0 * currentseg->width) )
	      if ( currentseg->next->length > (2.0 * currentseg->next->width) )
		{
#if SCAN_VERBOSE
		  printf("Possible straight/extension?? FORCING TRANSITION\n");
#endif
		  force_transition = 1;
		}
#endif


#if SCAN_DEBUG
	      printf("%4d %-12s %d(%s) W:%5.2f L:%6.2f R:%6.2f (%6.2f-%6.2f) dr:%5.2f(%6.2f) tr:%d t:%2s A:%6.2f\n",
		     currentseg->id,
		     currentseg->name,
		     currentseg->type, 
		     typestr[currentseg->type],
		     currentseg->width,
		     currentseg->length,
		     currentseg->radius,
		     rad_min, 
		     rad_max,
		     dr, last_dr,
		     transitions,
		     turnstr[turn_type],
		     turn_arc * RAD_TO_DEG );
#endif
	      if ( (fabs(dr) > 1.0) &&
		   (fabs(dr - last_dr) > 1.0 ) )
		{
#if SCAN_VERBOSE
		  printf("RADIUS CHANGE\n");
#endif
		  transitions++;
		} 
		   
	      last_dr = dr;
	      currentseg = currentseg->next;
	    }

	  end = currentseg->prev;

	  next_turn = currentseg->type;

	  if ( (transitions == 0) && (turn_type == TURN_CR))
	    {
#if SCAN_VERBOSE
	      printf("CR TURN; NO TRANSITIONS\n");
#endif

	      /* small kink?? */
	      float rad, width;
	      float kink;
	      float kink_ang;

	      width = currentseg->prev->width;
	      rad   = currentseg->prev->radius;

	      kink = ( (rad - width/2.0) / (rad + width/2.0));

	      kink_ang = 2.0 * acos ( kink );
#if SCAN_DEBUG
	      printf("CR: Kink angle:%6.2f turn_arc:%6.2f\n",
		     kink_ang*RAD_TO_DEG,
		     turn_arc*RAD_TO_DEG);
#endif

	      if ( (turn_arc <= (85 / RAD_TO_DEG)) && (turn_arc < kink_ang) )
		{

#if SCAN_VERBOSE
		      printf("SMALL KINK\n");
#endif
		      addSegKink( 0, start, end, turn_arc);
		}
	      else if ( turn_arc < 2.0 * kink_ang )
		{
#if SCAN_VERBOSE
		      printf("CR SINGLE APEX TURN\n");
#endif
		      addSegCR( 0, start, end, turn_arc, currentseg->prev->radius);
		}
	      else if ( turn_arc < 3.0 * kink_ang )
		{
#if SCAN_VERBOSE
		      printf("CR DOUBLE APEX TURN\n");
#endif
		      //addSegCRDouble( start, end, turn_arc, currentseg->prev->radius);
		      addSegCR( 0, start, end, turn_arc, currentseg->prev->radius);
		}
	      else
		{ 
#if SCAN_VERBOSE
		  printf("CR: kink: unknown/triple apex or higher\n");
#endif
		  //addSegUnknown( 0, start, end);
		  /* HACK, treat as single apex CR turn */
		  addSegCR( 0, start, end, turn_arc, end->radius);
		}

	    }
	  else if ( (transitions == 0) && (turn_type == TURN_DR))
	    {
#if SCAN_VERBOSE
	      printf("DR TURN; NO TRANSITIONS\n");
#endif
	      //addSegUnknown( 0, start, end);
	      /* HACK, treat as single apex CR turn */
	      addSegCR( 0, start, end, turn_arc, end->radius);
	    }
	  else if ( (transitions == 0) && (turn_type == TURN_IR))
	    {
#if SCAN_VERBOSE
	      printf("IR SINGLE APEX TURN\n");
#endif
	      addSegCR( 0, start, end, turn_arc, start->radius);
	    }
	  else if (transitions == 1)
	    {
#if SCAN_VERBOSE
	      printf("SINGLE TRANSITION TURN, type:%d(%s)\n",
		     turn_type, turnstr[turn_type]);
#endif
	      //addSegUnknown( start, end);
	      /* HACK, treat as single apex CR turn */
	      addSegCR( 0, start, end, turn_arc, currentseg->prev->radius);
	    }
	  else if (transitions == 2)
	    {
#if SCAN_VERBOSE
	      printf("DOUBLE TRANSITION TURN, type:%d(%s)\n",
		     turn_type, turnstr[turn_type]);
#endif
	      //addSegUnknown( start, end);
	      /* HACK, treat as single apex CR turn */
	      addSegCR( 0, start, end, turn_arc, currentseg->prev->radius);
	    }
	  else
	    {
#if SCAN_VERBOSE
	      printf("UNKNOWN TURN TYPE: trans:%d type:%d(%s)\n",
		     transitions,
		     turn_type, turnstr[turn_type]);
#endif
	      //addSegUnknown( 0, start, end);
	      /* HACK, treat as single apex CR turn */
	      addSegCR( 0, start, end, turn_arc, currentseg->prev->radius);
	    }

#if SCAN_VERBOSE
	  printf("END OF %s TURN, apex:%5.2f trans:%d\n",
		 turnstr[turn_type],
		 turn_arc / 2.0 * RAD_TO_DEG,
		 transitions);
#endif

#if SCAN_VERBOSE
	  if ( next_turn == TR_STR )
	    {
	      printf("TURN EXIT\n");
	    }
	  else
	    {
	      printf("S-TURN\n");
	    }
#endif
	}
    } 
  while (currentseg != startseg);	

#if SCAN_VERBOSE
  printf("Done\n");
  printf("=================================\n");
#endif  

/* ============================================*/
#if DUMP_TRACK
  dumpTrack(track);
#endif

#if DUMP_LINE0
  dumpLine((char *)LINE0_DATA, 0);
#endif
/* ============================================*/
}

/**********************************************************/
void Line::refineLine(tTrack* t, int pass_in )
{
  int cur, prev, next, prev2, next2;
#if 1
  int pass;
#endif

#if SCAN_VERBOSE
  char *segstr[] = { (char *)"NULL", (char *)"STR ", (char *)"ZSTR", (char *)"KINK", (char *)" CR ", (char *)"CR2 ", (char *)" CL ", (char *)"SSTR" };
  char *dirstr[] = { (char *)"X", (char *)"R", (char *)"L", (char *)"S" };

  float run_in, run_out;
#endif
#if SCAN_DEBUG
  char *typestr[] = { (char *)"NULL", (char *)"R", (char *)"L", (char *)"S" };
#endif

#if 0
  cur = num_lseg[pass] - 1;
  prev = cur - 1;
  next = 0; // (cur + 1) % num_lseg[0]
#else
  cur = 0;
#endif

#if 1
  pass = pass_in;

  if ( pass_in > 2)
    {
      pass = 2; /* lame attempt to conserve memory */

      //printf("pass_in:%d pass:%d\n",pass_in, pass);
#if 1
      memcpy( lseg[pass], lseg[pass+1], 
	      MAX_LINE_SEGS * sizeof(lineseg_t));
#else
      int index;
      for ( index = 0; index < num_lseg[pass+1] ; index++ )
	{
	  lseg[pass][index].type           = lseg[pass+1][index].type;
	  lseg[pass][index].startseg       = lseg[pass+1][index].startseg;
	  lseg[pass][index].endseg         = lseg[pass+1][index].endseg;
	  lseg[pass][index].ttype          = lseg[pass+1][index].ttype;
	  lseg[pass][index].t_start        = lseg[pass+1][index].t_start;
	  lseg[pass][index].t_end          = lseg[pass+1][index].t_end;
	  lseg[pass][index].length         = lseg[pass+1][index].length;
	  lseg[pass][index].arc            = lseg[pass+1][index].arc;
	  lseg[pass][index].radius         = lseg[pass+1][index].radius;
	  lseg[pass][index].fromstart      = lseg[pass+1][index].fromstart;
	  lseg[pass][index].startpt.x      = lseg[pass+1][index].startpt.x;
	  lseg[pass][index].startpt.y      = lseg[pass+1][index].startpt.y;
	  lseg[pass][index].endpt.x        = lseg[pass+1][index].endpt.x;
	  lseg[pass][index].endpt.y        = lseg[pass+1][index].endpt.y;
	  lseg[pass][index].t_startpt.x    = lseg[pass+1][index].t_startpt.x;
	  lseg[pass][index].t_startpt.y    = lseg[pass+1][index].t_startpt.y;
	  lseg[pass][index].t_endpt.x      = lseg[pass+1][index].t_endpt.x;
	  lseg[pass][index].t_endpt.y      = lseg[pass+1][index].t_endpt.y;
	  lseg[pass][index].apex.x         = lseg[pass+1][index].apex.x;
	  lseg[pass][index].apex.y         = lseg[pass+1][index].apex.y;
	  lseg[pass][index].path_center.x  = lseg[pass+1][index].path_center.x;
	  lseg[pass][index].path_center.y  = lseg[pass+1][index].path_center.y;
	  lseg[pass][index].path_radius    = lseg[pass+1][index].path_radius;
	  lseg[pass][index].apex1_t        = lseg[pass+1][index].apex1_t;
	  lseg[pass][index].entry_t        = lseg[pass+1][index].entry_t;
	  lseg[pass][index].entry_a        = lseg[pass+1][index].entry_a;
	  lseg[pass][index].entry_s        = lseg[pass+1][index].entry_s;
	  lseg[pass][index].run_in         = lseg[pass+1][index].run_in;
	  if ( lseg[pass+1][index].run_in > 0.0 )
	    printf("pass_in:%d pass:%d index:%d run_in:%f\n",
		   pass_in, pass, index, lseg[pass+1][index].run_in);
	  lseg[pass][index].start_rot      = lseg[pass+1][index].start_rot;
	  lseg[pass][index].sign_ex        = lseg[pass+1][index].sign_ex;
	  lseg[pass][index].sign_ey        = lseg[pass+1][index].sign_ey;
	  lseg[pass][index].apex2_t        = lseg[pass+1][index].apex2_t;
	  lseg[pass][index].exit_t         = lseg[pass+1][index].exit_t;
	  lseg[pass][index].exit_a         = lseg[pass+1][index].exit_a;
	  lseg[pass][index].exit_s         = lseg[pass+1][index].exit_s;
	  lseg[pass][index].run_out        = lseg[pass+1][index].run_out;
	  lseg[pass][index].end_rot        = lseg[pass+1][index].end_rot;
	  lseg[pass][index].sign_xx        = lseg[pass+1][index].sign_xx;
	  lseg[pass][index].sign_xy        = lseg[pass+1][index].sign_xy;
	  lseg[pass][index].apex2.x        = lseg[pass+1][index].apex2.x;
	  lseg[pass][index].apex2.y        = lseg[pass+1][index].apex2.y;
	  lseg[pass][index].midpoint.x     = lseg[pass+1][index].midpoint.x;
	  lseg[pass][index].midpoint.y     = lseg[pass+1][index].midpoint.y;
	  lseg[pass][index].path2_center.x = lseg[pass+1][index].path2_center.x;
	  lseg[pass][index].path2_center.y = lseg[pass+1][index].path2_center.y;
	  lseg[pass][index].path2_radius   = lseg[pass+1][index].path2_radius;
	}
#endif
      num_lseg[pass] = num_lseg[pass+1];
    }

  memset( lseg[pass+1], 0, MAX_LINE_SEGS * sizeof(lineseg_t));
#endif
  num_lseg[pass+1] = 0;

#if SCAN_VERBOSE
  printf("#==========================\n");
  printf("#refine pass:%d\n",pass);
#endif

  while ( cur < num_lseg[pass] )
    {
      prev = ( cur - 1);
      if ( prev < 0 ) prev += num_lseg[pass];

      next = ( cur + 1);
      if ( next >= num_lseg[pass]) next -= num_lseg[pass];

      prev2 = ( cur - 2);
      if ( prev2 < 0 ) prev2 += num_lseg[pass];

      next2 = ( cur + 2);
      if ( next2 >= num_lseg[pass]) next2 -= num_lseg[pass];

#if SCAN_VERBOSE
      printf("\nseg:%3d %3d %3d p2:%s(%s) p:%s(%s) c:%s(%s) n:%s(%s) n2:%s(%s)\n\n",
	     cur,
	     lseg[pass][cur].startseg->id,
	     lseg[pass][cur].endseg->id,
	     segstr[lseg[pass][prev2].type],
	     dirstr[lseg[pass][prev2].ttype],
	     segstr[lseg[pass][prev].type],
	     dirstr[lseg[pass][prev].ttype],
	     segstr[lseg[pass][cur].type],
	     dirstr[lseg[pass][cur].ttype],
	     segstr[lseg[pass][next].type],
	     dirstr[lseg[pass][next].ttype],
	     segstr[lseg[pass][next2].type],
	     dirstr[lseg[pass][next2].ttype]);
#endif

      switch (lseg[pass][cur].type)
	{
	case SEG_STR:
#if SCAN_VERBOSE
	  if (lseg[pass][prev].type == SEG_KINK)
	    {
	      printf("Entry: annex kink?\n");
	      if (lseg[pass][prev2].type == SEG_STR)
		{
		  printf("Entry: fit turn to kink?\n");
		}
	    }

	  if (lseg[pass][next].type == SEG_KINK)
	    {
	      printf("Exit: annex kink?\n");
	      if (lseg[pass][next2].type == SEG_STR)
		{
		  printf("Exit: fit turn to kink?\n");
		}
	    }
#endif

	  addSegStraight( (pass+1), 
			  lseg[pass][cur].startseg, 
			  lseg[pass][cur].endseg, 
			  lseg[pass][cur].length );

#if 1
	  /* fit to prev/next line */
	  //printf("cur:%d fitting to adjacent (%d:%d)\n",cur,prev,next);
	  lseg[pass+1][cur].startpt.x = lseg[pass][prev].endpt.x;
	  lseg[pass+1][cur].startpt.y = lseg[pass][prev].endpt.y;

	  lseg[pass+1][cur].endpt.x = lseg[pass][next].startpt.x;
	  lseg[pass+1][cur].endpt.y = lseg[pass][next].startpt.y;
#endif
	  break;

	case SEG_KINK:
	  if ( (lseg[pass][prev].type == SEG_STR) &&
	       (lseg[pass][next].type == SEG_STR) 
	       )
	    {
#if SCAN_VERBOSE
	      printf("fit turn to kink?\n");
#endif
#if 0
	      addSegCR( (pass+1), 
			lseg[pass][cur].startseg, 
			lseg[pass][cur].endseg, 
			lseg[pass][cur].arc, 
			lseg[pass][cur].radius);
#endif
	      addSegCR( (pass+1), 
			lseg[pass][cur].startseg->prev, 
			lseg[pass][cur].endseg->next, 
			lseg[pass][cur].arc, 
			lseg[pass][cur].radius);
	    }
	  else
	    {
	      addSegStraight( (pass+1), 
			      lseg[pass][cur].startseg, 
			      lseg[pass][cur].endseg, 
			      lseg[pass][cur].length );
	    }
#if 1
	  /* fit to prev/next line */
	  lseg[pass+1][cur].startpt.x = lseg[pass][prev].endpt.x;
	  lseg[pass+1][cur].startpt.y = lseg[pass][prev].endpt.y;

	  lseg[pass+1][cur].endpt.x = lseg[pass][next].startpt.x;
	  lseg[pass+1][cur].endpt.y = lseg[pass][next].startpt.y;
#endif
	  break;

	case SEG_CL:
	  tTrackSeg *entryseg, *exitseg;

	  entryseg = lseg[pass][cur].startseg;
	  exitseg  = lseg[pass][cur].endseg;

#if SCAN_VERBOSE
	  printf("refine turn:\n");

	  run_in = cloth_x(lseg[pass][cur].entry_t) * lseg[pass][cur].entry_a;
	  printf("Entry: t_a:%f t_e:%f a:%f run-in:%f\n",
		 lseg[pass][cur].apex1_t,
		 lseg[pass][cur].entry_t,
		 lseg[pass][cur].entry_a,
		 run_in);

	  if (lseg[pass][prev].type == SEG_KINK)
	    {
	      if (lseg[pass][prev2].type == SEG_CL)
		{
		  if ( lseg[pass][prev2].ttype == lseg[pass][cur].ttype )
		    printf("Enter: Bridge straight (O-O)\n");
		  else
		    printf("Enter: split with next turn (M-M/S)\n");
		}
	      else if (lseg[pass][prev2].type == SEG_KINK)
		printf("Entry: Multi-kink: need more intelligence\n");
	      else if (lseg[pass][prev2].type == SEG_STR)
		printf("Entry: annex kink exit, cl entry from straight\n");

	    }
	  else if (lseg[pass][prev].type == SEG_STR)
	    {
	      printf("Entry straight: len:%f\n",lseg[pass][prev].length);

	      if ( lseg[pass][prev].length >= run_in )
		printf("Enter: optimize outside-greedy (O)\n");
	      else
		printf("Enter: optimize outside-possible split (O)\n");

	    }
	  else if ( lseg[pass][prev].type == SEG_CL) 
	    {
	      if ( lseg[pass][prev].ttype == lseg[pass][cur].ttype )
		printf("Enter: NoOpt:Turn split?\n");
	      else
		printf("Enter: S-turn NoOpt:(M-M/S)\n");
	    }

	  run_out = cloth_x(lseg[pass][cur].exit_t) * lseg[pass][cur].exit_a;

	  printf("Exit: t_a:%f t_e:%f a:%f run-out:%f\n",
		 lseg[pass][cur].apex2_t,
		 lseg[pass][cur].exit_t,
		 lseg[pass][cur].exit_a,
		 run_out);

	  if (lseg[pass][next].type == SEG_KINK)
	    {
	      if (lseg[pass][next2].type == SEG_CL)
		{
		  if ( lseg[pass][next2].ttype == lseg[pass][cur].ttype )
		    printf("Exit: annex kink apex, split with next turn\n");
		  else
		    printf("Exit: split with next turn (M-M/S)\n");
		}
	      else if (lseg[pass][next2].type == SEG_KINK)
		printf("exit: Multi-kink: need more intelligence\n");
	      else if (lseg[pass][next2].type == SEG_STR)
		printf("exit: annex kink exit, cl entry from straight\n");
	    }
	  else if (lseg[pass][next].type == SEG_STR)
	    {
	      printf("Exit straight: len:%f\n",lseg[pass][prev].length);

	      if ( lseg[pass][next].length >= run_out )
		printf("Exit: optimize outside-greedy (O)\n");
	      else
		printf("Exit: optimize outside-possible split (O)\n");
	    }
	  else if ( lseg[pass][next].type == SEG_CL) 
	    {
	      if ( lseg[pass][next].ttype == lseg[pass][cur].ttype )
		printf("Enter: NoOpt:Turn split?\n");
	      else
		printf("Enter: S-turn NoOpt:(M-M/S)\n");
	    }
#endif

#if SCAN_DEBUG
	  printf("cur:%d prev_type:%s run_in:%5.2f\n",
		 cur,
		 typestr[entryseg->prev->type],
		 lseg[pass][cur].run_in);
#endif
	  /* extend entry into straight */
	  //while ( (entryseg->prev->type == TR_STR) && (lseg[pass][cur].run_in < 0.0) )
	  if ( (entryseg->prev->type == TR_STR) && (lseg[pass][cur].run_in < 0.0)
#if HACKS
	       && ( (cur != 10) || ( pass < 2))
#endif
	       )
	    {
#if SCAN_DEBUG
	      printf("Adding entry seg:%d len:%5.2f to lseg:%d (pass %d) run-in:%5.2f\n",
		     entryseg->prev->id,
		     entryseg->prev->length,
		     cur,
		     pass,
		     lseg[pass][cur].run_in);
#endif
	      entryseg = entryseg->prev;
	      //lseg[pass+1][prev].endseg = lseg[pass+1][prev].endseg->prev;
	      //lseg[pass+1][prev].length -= entryseg->length;
	      lseg[pass][cur].run_in += entryseg->length;
	    }

#if SCAN_DEBUG
	  printf("cur:%d next_type:%s run_out:%5.2f\n",
		 cur,
		 typestr[entryseg->next->type],
		 lseg[pass][cur].run_out);
#endif
	  /* extend exit into straight */
	  //while ( (exitseg->next->type == TR_STR) && (lseg[pass][cur].run_out < 0.0) )
	  if ( (exitseg->next->type == TR_STR) && (lseg[pass][cur].run_out < 0.0)
#if HACKS
	       && ( cur != 4)
	       && ( (cur != 8) || ( pass < 2))
#endif
	       )
	    {
#if SCAN_DEBUG
	      printf("Adding exit seg:%d len:%5.2f to lseg:%d (pass %d) run-out:%5.2f\n",
		     exitseg->next->id,
		     exitseg->next->length,
		     cur,
		     pass,
		     lseg[pass][cur].run_out);
#endif
	      exitseg = exitseg->next;
	      //lseg[pass][next].startseg = lseg[pass][next].startseg->next;
	      //lseg[pass][next].length -= exitseg->length;
	      lseg[pass][cur].run_out += exitseg->length;
	    }
#if HACKS
	  /* yecch! */
	  if ( 0 == pass ) {
	    /* double apex and smooth turn 1 on e-track-1 */
	    if ( 1 == cur ) exitseg  =  exitseg->next->next->next->next;
	    if ( 2 == cur ) entryseg = entryseg->next->next->next->next;
	    //if ( 4 == cur ) exitseg  = exitseg->prev;
	    if ( 19 == cur ) exitseg  =  exitseg->next->next->next->next->next;
	    if ( 20 == cur ) entryseg = entryseg->next->next->next->next->next;
	  }
#endif


	  addSegCR( (pass+1), 
		    entryseg, 
		    exitseg, 
		    lseg[pass][cur].arc, 
		    lseg[pass][cur].radius);


	  break;


	default:
#if SCAN_VERBOSE
	  printf("%s: *** UNHANDLED LINESEG TYPE %d ***\n",__FUNCTION__,lseg[pass][cur].type);
#endif
	  break;
	}
      
      cur++;
      //cur--;
    }

#if SCAN_VERBOSE
  printf("#refine pass:%d done\n",pass);
  printf("#==========================\n");
#endif

#if DUMP_LINE1
  switch (pass)
    {
    case 0:
      dumpLine((char *)LINE1_DATA, 1);
      break;

    case 1:
      dumpLine((char *)LINE2_DATA, 2);
      break;

    case 2:
      dumpLine((char *)LINE3_DATA, 2);
      break;

    case 3:
      dumpLine((char *)LINE4_DATA, 2);
      break;

    case 4:
      dumpLine((char *)LINE5_DATA, 2);
      break;

    case 5:
      dumpLine((char *)LINE6_DATA, 2);
      break;

    case 6:
      dumpLine((char *)LINE7_DATA, 2);
      break;

    case 7:
      dumpLine((char *)LINE8_DATA, 2);
      break;

    case 8:
      dumpLine((char *)LINE9_DATA, 2);
      break;

    default:
      printf("Unhandled dumpLine pass number %d\n",pass);
      dumpLine((char *)LINE9_DATA, 2);
      break;
    }
#endif

  return;
}

/**********************************************************/

void Line::customizeLine(tTrack* t, int pass )
{
  int cur;

#if SCAN_VERBOSE
  char *segstr[] = { (char *)"NULL", (char *)"STR ", (char *)"ZSTR", (char *)"KINK", (char *)" CR ", (char *)"CR2 ", (char *)" CL ", (char *)"SSTR" };
  char *dirstr[] = { (char *)"X", (char *)"R", (char *)"L", (char *)"S" };
#endif

#if SCAN_VERBOSE
  printf("#==========================\n");
  printf("#customize pass:%d\n",pass);
#endif

  cur = 0;

  while ( cur < num_lseg[pass] )
    {
#if SCAN_VERBOSE
      printf("Seg:%d start:%d end:%d type:%s dir:%s \n",
	     cur,
	     lseg[pass][cur].startseg->id,
	     lseg[pass][cur].endseg->id,
	     segstr[lseg[pass][cur].type],
	     dirstr[lseg[pass][cur].ttype]);
#endif
      cur++;
    }

#if SCAN_VERBOSE
  printf("#customize pass:%d done\n",pass);
  printf("#==========================\n");
#endif

#if DUMP_LINE1
  switch (pass)
    {
    case 1:
      dumpLine((char *)LINE1_DATA, 1);
      break;

    case 2:
      dumpLine((char *)LINE2_DATA, 2);
      break;

    default:
      printf("Unhandled customizeLine pass number %d\n",pass);
      dumpLine((char *)LINE2_DATA, 2);
      break;
    }
#endif

  return;
}

/**********************************************************/

#define RADFACT 1.0  // RADFACT 2.0
#define ALTRADFACT 1.0  // ALTRADFACT 1.4

float Line::lineRadius( int line, float fromstart )
{
  /* code common with gettargetpoint */
  int lineseg = 0;


  if ( (fromstart <  lseg[line][1].fromstart) ||
       ((lseg[line][0].startseg->id > lseg[line][0].endseg->id) &&             
	(fromstart >= lseg[line][0].fromstart) ) )
    {
      lineseg = 0;
    }
  else
    {
      lineseg = 1;

      while ( (lineseg < num_lseg[line]) &&
	      (fromstart > lseg[line][lineseg].fromstart) )
	{
	  lineseg++;
	}
      /* now past desired lineseg by 1 */
      if ( lineseg > 1) lineseg--;
    }



  if ( (lseg[line][lineseg].type == SEG_STR) ||
       (lseg[line][lineseg].type == SEG_KINK)  )
    {
      return FLT_MAX;
    } 
  else if (lseg[line][lineseg].type == SEG_CL)  
    {
      float len_entry, len_exit;
      float t_targ, t_start, t_end;
      float a;
      float radius;
      float run_in;

      run_in = lseg[line][lineseg].run_in;

      if ( run_in < 0.0 ) run_in = 0.0;

      len_entry = lseg[line][lineseg].entry_a * 
	( lseg[line][lineseg].apex1_t - lseg[line][lineseg].entry_t);

      len_exit  = lseg[line][lineseg].exit_a  * 
	( lseg[line][lineseg].apex2_t - lseg[line][lineseg].exit_t);


      if ( (fromstart - lseg[line][lineseg].fromstart) < run_in )
	return FLT_MAX;

      else if ( (fromstart - lseg[line][lineseg].fromstart) < (len_entry + run_in) )
	{
	  /* all in entry */
	  a = lseg[line][lineseg].entry_a;

	  t_start = lseg[line][lineseg].entry_t;
	  t_targ = ( (fromstart - lseg[line][lineseg].fromstart - run_in) + a * t_start) / a;

	  radius = a/t_targ;
#if SCAN_DEBUG
	  printf("rad: seg:%d len:%f run_in:%f entry:%f a:%f t:%f rad:%f\n",
		 lineseg,
		 (fromstart - lseg[line][lineseg].fromstart),
		 run_in,
		 len_entry,
		 a,
		 t_targ,
		 radius);
#endif
 

	  if ( radius < 0.0 )
	    return FLT_MAX;
	  else
	    {
#if 0
#if HACKS
	      if ( (lineseg == 3) || (lineseg == 5) || (lineseg == 7) || (lineseg == 13) || (lineseg == 27) || (lineseg == 37) )
		return radius;
	      else
#endif
#endif
		return (RADFACT * radius);
	    }
	}
      else if ( (fromstart - lseg[line][lineseg].fromstart) < (len_entry + len_exit + run_in) )
	{
	  /* all in exit */
	  a = lseg[line][lineseg].exit_a;

	  t_end   = lseg[line][lineseg].exit_t;
	  t_start = lseg[line][lineseg].apex2_t;;

	  //t_targ = ( (a * t_start) - (fromstart - lseg[line][lineseg].fromstart)) / a;
	  t_targ = ( (a * t_start) - (fromstart - lseg[line][lineseg].fromstart - len_entry - run_in )) / a;

	  radius = a/t_targ;
#if SCAN_DEBUG
	  printf("rad: seg:%d len:%f exit:%f a:%f t:%f rad:%f\n",
		 lineseg,
		 fromstart,
		 len_entry+len_exit,
		 a,
		 t_targ,
		 radius);
#endif

	  if ( radius < 0.0 )
	    return FLT_MAX;
	  else
	    {
		return (RADFACT * radius);
	    }
	}
      else if ( (fromstart - lseg[line][lineseg].fromstart) > (len_entry + len_exit + run_in) )
	return FLT_MAX;
      else 
	{
	  /* at apex */
	  float radius1, radius2;

	  radius1 = lseg[line][lineseg].entry_a / lseg[line][lineseg].apex1_t;
	  radius2 = lseg[line][lineseg].exit_a  / lseg[line][lineseg].apex2_t;

	  radius = MIN( radius1, radius2);

#if SCAN_DEBUG
	  printf("rad: seg:%d fromstart: %fapex: a1:%f t1:%f r1:%f a2:%f t2:%f r2:%f rad:%f\n",
		 lineseg, fromstart,
		 lseg[line][lineseg].entry_a,
		 lseg[line][lineseg].apex1_t,
		 radius1,
		 lseg[line][lineseg].exit_a,
		 lseg[line][lineseg].apex2_t,
		 radius2,
		 radius);
#endif

	  if ( radius < 0.0 )
	    return FLT_MAX;
	  else
	    return (RADFACT * radius);
	}
    }
  else
    printf("%s: *** UNHANDLED LINESEG TYPE %d (line %d) (fromstart %f) ***\n",
	   __FUNCTION__,
	   lseg[line][lineseg].type,
	   line,
	   fromstart);

  return FLT_MAX;

} 

/**********************************************************/

void Line::GetRaceLineData(tSituation *s, v2d *target, float *speed, float *avspeed, 
		     float *raceoffset, float *lookahead, float *racesteer)
{
  /* 
   * target->x , target->y - target point out
   * *racesteer - steering out
   */
#if 0
  tTrackSeg *seg;
  float lgfromstart;

  seg = car->_trkPos.seg;
  lgfromstart = seg->lgfromstart + seg->length - getDistToSegEnd();

#endif

}

/**********************************************************/

vec2f Line::getTargetPoint(int line,
			   float fromstart, 
			   float offset )
{
  /* This is FAR from ideal */
  /* Really should trace down the raceline */

  
  int lineseg = 0;

#if 0
  if ( fabs(offset) > 1.0 )
    printf("gettarg: offset:%f\n",offset);
#endif

  while ( fromstart >  mytrack->length )
    fromstart -= mytrack->length;

  while ( fromstart < 0.0 )
    fromstart += mytrack->length;


  /*we're looking for an absolute distance into the track, but it is
  possible that the first segment wraps the start-finish line */

#if 0
  printf("gettarg: fromstart: %f num_lseg:%d %d:%f %d:%f\n",
	 fromstart,
	 num_lseg[line],
	 lseg[line][0].startseg->id,
	 lseg[line][0].fromstart,
	 lseg[line][1].startseg->id,
	 lseg[line][1].fromstart);
#endif

  /* find lineseg that contains seg */ 
  if ( (fromstart <  lseg[line][1].fromstart) ||
       ((lseg[line][0].startseg->id > lseg[line][0].endseg->id) &&             
	(fromstart >= lseg[line][0].fromstart) ) )
    {
      lineseg = 0;
    }
  else
    {

      lineseg = 1;

      while ( (lineseg < num_lseg[line]) &&
	      (fromstart > lseg[line][lineseg].fromstart) )
	{
#if 0
	      printf("gettarg: seg:%d fromstart:%f\n",lineseg,lseg[line][lineseg].fromstart);
#endif
	      lineseg++;
	}
      /* now past desired lineseg by 1 */
      if (lineseg > 1) lineseg--;
    }
#if 0
  printf("gettarg: done:seg:%d fromstart:%f\n",lineseg,lseg[line][lineseg].fromstart);
#endif

#if 0
  if ( (fromstart < 400) || (fromstart > 3200) )
    printf("gettarg: lineseg:%d fromstart:%f\n",lineseg,fromstart);
#endif


  /* everything above here should be a utility function */

  vec2f s,t;

  s.x = lseg[line][lineseg].startpt.x;
  s.y = lseg[line][lineseg].startpt.y;;

  if ( (lseg[line][lineseg].type == SEG_STR) ||
       (lseg[line][lineseg].type == SEG_KINK)    )
    {
      vec2f d, n;


      d.x = (lseg[line][lineseg].endpt.x - lseg[line][lineseg].startpt.x)/lseg[line][lineseg].length;
      d.y = (lseg[line][lineseg].endpt.y - lseg[line][lineseg].startpt.y)/lseg[line][lineseg].length;
      d.normalize();

#if 0
      n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
      n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
      n.normalize();
#endif
      //n.x = d.x;
      //n.y = d.y;
      //n = n.rotate(s, -1.0 / 2.0 * PI );
      //n = n.rotate(s, (0.5 * PI) );
      n.x = (lseg[line][lineseg].endseg->vertex[TR_EL].x - lseg[line][lineseg].endseg->vertex[TR_ER].x)/
	lseg[line][lineseg].endseg->length;
      n.y = (lseg[line][lineseg].endseg->vertex[TR_EL].y - lseg[line][lineseg].endseg->vertex[TR_ER].y)/
	lseg[line][lineseg].endseg->length;
      n.normalize();

#if 0
      if ( fabs(offset) > 1.0 )
	printf("gettarg: offset:%f n.x:%f n.y:%f\n",offset, n.x, n.y);
#endif

      if ( fromstart < lseg[line][lineseg].fromstart )
	fromstart += mytrack->length;

      t = s 
	+ d*(fromstart - lseg[line][lineseg].fromstart) 
	+ offset*n;

#if 0
      if ( (fromstart < 400) || (fromstart > 3200) )
	printf("S: fromstart:%f x:%f y:%f\n",fromstart,t.x,t.y);
#endif

      return t;
    } 
  else if (lseg[line][lineseg].type == SEG_CL)  
    {
      float len_entry, len_exit;
      float run_in;
      float t_targ, t_start, t_end;
      float x_orig, y_orig;
      float sign_x, sign_y;
      float a;
      float rot;

      vec2f origin, radial, shift, t;

      run_in = lseg[line][lineseg].run_in;

      if ( run_in < 0.0 ) run_in = 0.0;
#if 0
      else if ( run_in > 0.0)
	printf("gettarg: CL: fromstart:%f run_in:%f\n",fromstart,run_in);
#endif

      len_entry = lseg[line][lineseg].entry_a * 
	( lseg[line][lineseg].apex1_t - lseg[line][lineseg].entry_t);

      len_exit  = lseg[line][lineseg].exit_a  * 
	( lseg[line][lineseg].apex2_t - lseg[line][lineseg].exit_t);

      origin.x = 0.0;
      origin.y = 0.0;

#if 0
      if ( (fromstart < 400) || (fromstart > 3200) )
	printf("gettarg: CL:fromstart:%f run_in:%f\n",fromstart, run_in);
#endif

      if ( (fromstart - lseg[line][lineseg].fromstart) < run_in )
	{
	  /* treat as linear section at beginning */

	  vec2f d, n;

	  d.x = (lseg[line][lineseg].t_startpt.x - lseg[line][lineseg].startpt.x) / run_in;
	  d.y = (lseg[line][lineseg].t_startpt.y - lseg[line][lineseg].startpt.y) / run_in;
	  d.normalize();

	  n.x = (lseg[line][lineseg].startseg->vertex[TR_EL].x - lseg[line][lineseg].startseg->vertex[TR_ER].x) / lseg[line][lineseg].startseg->width;
	  n.y = (lseg[line][lineseg].startseg->vertex[TR_EL].y - lseg[line][lineseg].startseg->vertex[TR_ER].y) / lseg[line][lineseg].startseg->width;
	  n.normalize();


	  if ( fromstart < lseg[line][lineseg].fromstart )
	    fromstart += mytrack->length;

	  t = s 
	    + d*(fromstart - lseg[line][lineseg].fromstart) 
	    + offset*n;

#if 0
	  if ( (fromstart < 400) || (fromstart > 3200) )
	    printf("run-in: x:%f y:%f\n",t.x,t.y);
#endif
	    
	  return t;
	} /* len < run_in */
      else if ( (fromstart - lseg[line][lineseg].fromstart) < (len_entry + run_in) )
	{
	  a = lseg[line][lineseg].entry_a;

	  t_start = lseg[line][lineseg].entry_t;
	  t_targ = ( (fromstart - lseg[line][lineseg].fromstart - run_in) + a * t_start) / a;

#if 0
	  printf("targ: entry: len:%f len_en:%f len_ex:%f t_st:%f t_targ:%f\n",
		 (fromstart - lseg[line][lineseg].fromstart), len_entry, len_exit,
		 t_start, t_targ);
#endif

	  x_orig = cloth_x( t_start ) * a;
	  y_orig = cloth_y( t_start ) * a;

	  if ( lseg[line][lineseg].run_in > 0.0 )
	    x_orig -= lseg[line][lineseg].run_in;
	  
	  shift.x = lseg[line][lineseg].startpt.x;
	  shift.y = lseg[line][lineseg].startpt.y;

	  rot =  -1.0 * lseg[line][lineseg].start_rot;
	  
	  sign_x = lseg[line][lineseg].sign_ex;
	  sign_y = lseg[line][lineseg].sign_ey;

	  radial.x = cloth_x(t_targ) * a - x_orig;
	  radial.y = cloth_y(t_targ) * a - y_orig;

	  radial.x *= sign_x;
	  radial.y *= sign_y;

	  radial = radial.rotate( origin, rot) ;

	  radial = radial + shift;

	  t.x = radial.x;
	  t.y = radial.y;

	  /* FIXME - no offset! */
#if 0
	  if ( (fromstart < 400) || (fromstart > 3200) )
	    printf("entry: x:%f y:%f\n",t.x,t.y);
#endif

	  return t;
	} /* len < run_in + len_entry */
      else 
	{
	  if ( (fromstart - lseg[line][lineseg].fromstart) > (len_entry + len_exit + run_in) ) 
	    {
#if SCAN_DEBUG
	    printf("%s: length too large! ent:%f exit:%f sum:%f len:%f\n",
		   __FUNCTION__, 
		   len_entry, len_exit, 
		   len_entry + len_exit, 
		   (fromstart - lseg[line][lineseg].fromstart));
#endif	 
	    float new_from_start;
	    int nextseg;

	    nextseg = lineseg + 1;
	    if ( nextseg == num_lseg[line] )
	      nextseg = 0;

	    new_from_start =  fromstart - len_entry - len_exit - run_in
	      - lseg[line][lineseg].fromstart
	      + lseg[line][nextseg].fromstart;

#if SCAN_DEBUG
	    printf("%s: fromstart: %f corrected fromstart:%f\n",
		   __FUNCTION__, 
		   fromstart,
		   new_from_start);
#endif  
#if 0
	    if ( (fromstart < 400) || (fromstart > 3200) )
	      printf("run_out:\n");
#endif

	    return getTargetPoint(line, new_from_start, offset); 

	    } /* len > len_entry + len_exit + run_in */



	  a = lseg[line][lineseg].exit_a;

	  t_end   = lseg[line][lineseg].exit_t;
	  t_start = lseg[line][lineseg].apex2_t;;


	  t_targ = ( (a * t_start) - ((fromstart - lseg[line][lineseg].fromstart) - len_entry - run_in)) / a;

	  x_orig = cloth_x( t_end ) * a;
	  y_orig = cloth_y( t_end ) * a;
	  
	  if ( lseg[line][lineseg].run_out > 0.0 )
	    x_orig -= lseg[line][lineseg].run_out;
	  
	  shift.x = lseg[line][lineseg].endpt.x;
	  shift.y = lseg[line][lineseg].endpt.y;

	  rot = -1.0 * lseg[line][lineseg].end_rot;
	  NORM_PI_PI(rot);

	  sign_x = lseg[line][lineseg].sign_xx;
	  sign_y = lseg[line][lineseg].sign_xy;

	  radial.x = cloth_x(t_targ) * a - x_orig;
	  radial.y = cloth_y(t_targ) * a - y_orig;

	  radial.x *= sign_x;
	  radial.y *= sign_y;

	  radial = radial.rotate( origin, rot) ;

	  radial = radial + shift;

#if 0
	  printf("targ: exit: len:%f len_en:%f len_ex:%f t_st:%f t_end:%f t_targ:%f x:%g y:%f\n",
		 (fromstart - lseg[line][lineseg].fromstart), len_entry, len_exit,
		 t_start, t_end, t_targ,
		 radial.x, radial.y);
#endif


	  /* FIXME - no offset! */

	  t.x = radial.x;
	  t.y = radial.y;
#if 0
	  if ( (fromstart < 400) || (fromstart > 3200) )
	    printf("exit: x:%f y:%f\n",t.x,t.y);
#endif

	  return t;

	}
#if 0
      if ( (fromstart < 400) || (fromstart > 3200) )
	printf("E: OOPS!\n");
#endif

      return t;
    }
  else
    printf("%s: *** UNHANDLED LINESEG TYPE %d***\n",__FUNCTION__,lseg[line][lineseg].type);

  return lseg[line][lineseg].endpt;
}
/**********************************************************/


void Line::addSegStraight(int index,
			  tTrackSeg *start,
			  tTrackSeg *end,
			  float len )
{
  int prev_turn, next_turn;

  prev_turn = start->prev->type;
  next_turn = end->next->type;

  lseg[index][num_lseg[index]].type     = SEG_STR;
  lseg[index][num_lseg[index]].ttype    = TR_STR;

  lseg[index][num_lseg[index]].startseg = start;
  lseg[index][num_lseg[index]].endseg   = end;

  lseg[index][num_lseg[index]].fromstart= start->lgfromstart;

#if SCAN_DEBUG
  printf("AddSegStraight: start:%d end:%d fromstart:%f\n",
	 start->id,
	 end->id,
	 lseg[index][num_lseg[index]].fromstart);
#endif

  
  /* FIXME - does not account for car width,
   * turn in and braking zones
   * short Z-straights where straight line unsuitable
   */
  vec2f s,n;
  float car_width = my_car_width;

  if ( prev_turn == TR_RGT )
    {
      s.x = start->vertex[TR_SL].x;
      s.y = start->vertex[TR_SL].y;

      n.x = start->vertex[TR_SL].x - start->vertex[TR_SR].x;
      n.y = start->vertex[TR_SL].y - start->vertex[TR_SR].y;
      n.normalize();
    }
  else
    {
      s.x = start->vertex[TR_SR].x;
      s.y = start->vertex[TR_SR].y;

      n.x = start->vertex[TR_SR].x - start->vertex[TR_SL].x;
      n.y = start->vertex[TR_SR].y - start->vertex[TR_SL].y;
      n.normalize();
    }
  s = s - n * WIDTH_FACT * car_width;

  lseg[index][num_lseg[index]].startpt = s;
  //lseg[index][num_lseg[index]].startpt.x = s.x;
  //lseg[index][num_lseg[index]].startpt.y = s.y;

  if ( next_turn == TR_RGT )
    {
      s.x = end->vertex[TR_EL].x;
      s.y = end->vertex[TR_EL].y;

      n.x = end->vertex[TR_EL].x - end->vertex[TR_ER].x;
      n.y = end->vertex[TR_EL].y - end->vertex[TR_ER].y;
      n.normalize();
    }
  else
    {
      s.x = end->vertex[TR_ER].x;
      s.y = end->vertex[TR_ER].y;

      n.x = end->vertex[TR_ER].x - end->vertex[TR_EL].x;
      n.y = end->vertex[TR_ER].y - end->vertex[TR_EL].y;
      n.normalize();
    }
  s = s - n * WIDTH_FACT * car_width;

  lseg[index][num_lseg[index]].endpt = s;
  //lseg[index][num_lseg[index]].endpt.x = s.x;
  //lseg[index][num_lseg[index]].endpt.y = s.y;

  //lseg[index][num_lseg[index]].length   = len;
  lseg[index][num_lseg[index]].length = DIST ( lseg[index][num_lseg[index]].startpt.x,
					lseg[index][num_lseg[index]].startpt.y,
					lseg[index][num_lseg[index]].endpt.x,
					lseg[index][num_lseg[index]].endpt.y);

  num_lseg[index]++;

  if ( num_lseg[index] >= MAX_LINE_SEGS)
    printf("hymie_2016: %s: num_lseg[index] overflow\n",__FUNCTION__);
}

/**********************************************************/

void Line::addSegKink(int index,
		      tTrackSeg *start,
		      tTrackSeg *end,
		      float arc )
{
  int prev_turn, next_turn;

  prev_turn = start->prev->type;
  next_turn = end->next->type;

  lseg[index][num_lseg[index]].type     = SEG_KINK;
  lseg[index][num_lseg[index]].ttype    = start->type;


  lseg[index][num_lseg[index]].startseg = start;
  lseg[index][num_lseg[index]].endseg   = end;
  lseg[index][num_lseg[index]].arc      = arc;

  lseg[index][num_lseg[index]].fromstart= start->lgfromstart;

#if SCAN_DEBUG
  printf("AddSegKink: start:%d end:%d fromstart:%f\n",
	 start->id,
	 end->id,
	 lseg[index][num_lseg[index]].fromstart);
#endif

  
  /* FIXME - does not account for car width,
   * turn in and braking zones
   * short Z-straights where straight line unsuitable
   */
#if 0
  /* middle chord; not ideal but simple and good */
  lseg[index][num_lseg[index]].startpt.x = 
    (start->vertex[TR_SL].x + start->vertex[TR_SR].x) / 2.0;
  lseg[index][num_lseg[index]].startpt.y = 
    (start->vertex[TR_SL].y + start->vertex[TR_SR].y) / 2.0;

  lseg[index][num_lseg[index]].endpt.x = 
    (end->vertex[TR_EL].x + end->vertex[TR_ER].x) / 2.0;
  lseg[index][num_lseg[index]].endpt.y = 
    (end->vertex[TR_EL].y + end->vertex[TR_ER].y) / 2.0;
#endif

  /* outer chord  */
  if ( start->type == TR_RGT )
    {
      lseg[index][num_lseg[index]].startpt.x = start->vertex[TR_SL].x;
      lseg[index][num_lseg[index]].startpt.y = start->vertex[TR_SL].y;

      lseg[index][num_lseg[index]].endpt.x = end->vertex[TR_EL].x;
      lseg[index][num_lseg[index]].endpt.y = end->vertex[TR_EL].y;
    }
  else
    {
      lseg[index][num_lseg[index]].startpt.x = start->vertex[TR_SR].x;
      lseg[index][num_lseg[index]].startpt.y = start->vertex[TR_SR].y;

      lseg[index][num_lseg[index]].endpt.x = end->vertex[TR_ER].x;
      lseg[index][num_lseg[index]].endpt.y = end->vertex[TR_ER].y;
    }


  /* find offset from outer chord to apex */
  vec2f apex, midpoint, offset;

  apex = findApex( start, end, arc/2.0);

  lseg[index][num_lseg[index]].apex.x = apex.x;
  lseg[index][num_lseg[index]].apex.y = apex.y;

  midpoint.x = (lseg[index][num_lseg[index]].startpt.x + lseg[index][num_lseg[index]].endpt.x) / 2.0;
  midpoint.y = (lseg[index][num_lseg[index]].startpt.y + lseg[index][num_lseg[index]].endpt.y) / 2.0;

  offset = midpoint - apex;

  /* use this offset to correct start/end points */
  lseg[index][num_lseg[index]].startpt -= offset;  
  lseg[index][num_lseg[index]].endpt   -= offset;

  //lseg[index][num_lseg[index]].length   = len;
  lseg[index][num_lseg[index]].length = DIST ( lseg[index][num_lseg[index]].startpt.x,
					lseg[index][num_lseg[index]].startpt.y,
					lseg[index][num_lseg[index]].endpt.x,
					lseg[index][num_lseg[index]].endpt.y);


  num_lseg[index]++;

  if ( num_lseg[index] >= MAX_LINE_SEGS)
    printf("hymie_2016: %s: num_lseg[index] overflow\n",__FUNCTION__);
}

/**********************************************************/

void Line::addSegCR(int index,
		    tTrackSeg *start,
		    tTrackSeg *end,
		    float arc,
		    float radius)
{
  int prev_turn, next_turn;

  tTrackSeg *t_start, *t_end;

  tTrackSeg *tmpseg;
  float tmparc;

  vec2f t_startpt, t_endpt;

  prev_turn = start->prev->type;
  next_turn = end->next->type;

  lseg[index][num_lseg[index]].type     = SEG_CL;


  /* allow extrension of turn into straight segments */
  t_start = start;
  while ( t_start->type == TR_STR )
    t_start = t_start->next;

  t_end = end;
  while ( t_end->type == TR_STR )
    t_end = t_end->prev;

  lseg[index][num_lseg[index]].t_start = t_start;
  lseg[index][num_lseg[index]].t_end   = t_end;

  lseg[index][num_lseg[index]].ttype    = t_start->type;

  /* re-compute arc in case start/end has been changed */
  tmpseg = start;
  tmparc = 0.0;
  
  while ( tmpseg != end )
    {
      tmparc += tmpseg->arc;
      tmpseg = tmpseg->next;
    }
  /* add last seg */
  tmparc += tmpseg->arc;

  lseg[index][num_lseg[index]].arc      = tmparc;

  lseg[index][num_lseg[index]].startseg = start;
  lseg[index][num_lseg[index]].endseg   = end;

  lseg[index][num_lseg[index]].fromstart= start->lgfromstart;

#if SCAN_DEBUG
  printf("AddSegCR: start:%d end:%d fromstart:%f t_start:%d t_end:%d arc:%f rad:%f\n",
	 start->id,
	 end->id,
	 lseg[index][num_lseg[index]].fromstart,
	 t_start->id,
	 t_end->id,
	 lseg[index][num_lseg[index]].arc,
	 radius);
#endif

  
  /* FIXME - does not account for car width,
   * turn in and braking zones
   * direction of preceding/following turns
   */
  if ( (prev_turn == TR_STR) || (prev_turn == lseg[index][num_lseg[index]].ttype) )
    {
#if SCAN_DEBUG
      printf("Using outside edge for entry\n");
#endif    
      /* outer edge on entry */
      if ( t_start->type == TR_RGT )
	{
	  lseg[index][num_lseg[index]].startpt.x = start->vertex[TR_SL].x;
	  lseg[index][num_lseg[index]].startpt.y = start->vertex[TR_SL].y;

	  t_startpt.x = t_start->vertex[TR_SL].x;
	  t_startpt.y = t_start->vertex[TR_SL].y;
	}
      else
	{
	  lseg[index][num_lseg[index]].startpt.x = start->vertex[TR_SR].x;
	  lseg[index][num_lseg[index]].startpt.y = start->vertex[TR_SR].y;

	  t_startpt.x = t_start->vertex[TR_SR].x;
	  t_startpt.y = t_start->vertex[TR_SR].y;
	}
    }
  else 
    {
#if SCAN_DEBUG
      printf("Using middle for entry\n");
#endif    
      /* middle on entry */
      lseg[index][num_lseg[index]].startpt.x = (start->vertex[TR_SL].x + start->vertex[TR_SR].x) / 2.0;
      lseg[index][num_lseg[index]].startpt.y = (start->vertex[TR_SL].y + start->vertex[TR_SR].y) / 2.0;;

      t_startpt.x = (t_start->vertex[TR_SL].x + t_start->vertex[TR_SR].x) / 2.0;
      t_startpt.y = (t_start->vertex[TR_SL].y + t_start->vertex[TR_SR].y) / 2.0;;
    }
  lseg[index][num_lseg[index]].t_startpt.x = t_startpt.x;
  lseg[index][num_lseg[index]].t_startpt.y = t_startpt.y;


#if HACKS
  if ( (num_lseg[index] == 6) )
    {
#if SCAN_DEBUG
      printf("Forcing entry to middle seg:%d\n",num_lseg[index]);
#endif
      /* middle on entry */
      lseg[index][num_lseg[index]].startpt.x = (start->vertex[TR_SL].x + start->vertex[TR_SR].x) / 2.0;
      lseg[index][num_lseg[index]].startpt.y = (start->vertex[TR_SL].y + start->vertex[TR_SR].y) / 2.0;;

      t_startpt.x = (t_start->vertex[TR_SL].x + t_start->vertex[TR_SR].x) / 2.0;
      t_startpt.y = (t_start->vertex[TR_SL].y + t_start->vertex[TR_SR].y) / 2.0;;
    }

  if ( (num_lseg[index] == 2) || (num_lseg[index] == 10) || (num_lseg[index] == 18) )
    {
#if SCAN_DEBUG
      printf("Forcing entry to center seg:%d\n",num_lseg[index]);
#endif
      /* middle on entry */
      lseg[index][num_lseg[index]].startpt.x = 
	(start->vertex[TR_SL].x + start->vertex[TR_SR].x +
	 start->vertex[TR_EL].x + start->vertex[TR_ER].x) / 4.0;
      lseg[index][num_lseg[index]].startpt.y = 
	(start->vertex[TR_SL].y + start->vertex[TR_SR].y +
	 start->vertex[TR_EL].y + start->vertex[TR_ER].y) / 4.0;;

      t_startpt.x = (t_start->vertex[TR_SL].x + t_start->vertex[TR_SR].x) / 2.0;
      t_startpt.y = (t_start->vertex[TR_SL].y + t_start->vertex[TR_SR].y) / 2.0;;
    }
#endif



  if ( (next_turn == TR_STR) || (next_turn == lseg[index][num_lseg[index]].ttype) )
    {
#if SCAN_DEBUG
      printf("Using outside edge for exit\n");
#endif    
      /* outer edge on entry */
      if ( t_end->type == TR_RGT )
	{
	  lseg[index][num_lseg[index]].endpt.x = end->vertex[TR_EL].x;
	  lseg[index][num_lseg[index]].endpt.y = end->vertex[TR_EL].y;

	  t_endpt.x = t_end->vertex[TR_EL].x;
	  t_endpt.y = t_end->vertex[TR_EL].y;
	}
      else
	{
	  lseg[index][num_lseg[index]].endpt.x = end->vertex[TR_ER].x;
	  lseg[index][num_lseg[index]].endpt.y = end->vertex[TR_ER].y;

	  t_endpt.x = t_end->vertex[TR_ER].x;
	  t_endpt.y = t_end->vertex[TR_ER].y;
	}
    }
  else 
    {
#if SCAN_DEBUG
      printf("Using middle for exit\n");
#endif    
      /* middle on exit */
      lseg[index][num_lseg[index]].endpt.x = (end->vertex[TR_EL].x + end->vertex[TR_ER].x) / 2.0;
      lseg[index][num_lseg[index]].endpt.y = (end->vertex[TR_EL].y + end->vertex[TR_ER].y) / 2.0;;

      t_endpt.x = (t_end->vertex[TR_EL].x + t_end->vertex[TR_ER].x) / 2.0;
      t_endpt.y = (t_end->vertex[TR_EL].y + t_end->vertex[TR_ER].y) / 2.0;;
    }

#if HACKS
  if ( (num_lseg[index] == 1) || (num_lseg[index] == 4) )
    {
#if SCAN_DEBUG
      printf("Forcing exit to middle seg:%d\n",num_lseg[index]);
#endif
      lseg[index][num_lseg[index]].endpt.x = (end->vertex[TR_EL].x + end->vertex[TR_ER].x) / 2.0;
      lseg[index][num_lseg[index]].endpt.y = (end->vertex[TR_EL].y + end->vertex[TR_ER].y) / 2.0;;

      t_endpt.x = (t_end->vertex[TR_EL].x + t_end->vertex[TR_ER].x) / 2.0;
      t_endpt.y = (t_end->vertex[TR_EL].y + t_end->vertex[TR_ER].y) / 2.0;;
    }
  if ( (num_lseg[index] == 8) || (num_lseg[index] == 16) )
    {
#if SCAN_DEBUG
      printf("Forcing exit to center seg:%d\n",num_lseg[index]);
#endif
      lseg[index][num_lseg[index]].endpt.x = 
	(end->vertex[TR_EL].x + end->vertex[TR_ER].x +
	 end->vertex[TR_SL].x + end->vertex[TR_SR].x) / 4.0;
      lseg[index][num_lseg[index]].endpt.y = 
	(end->vertex[TR_EL].y + end->vertex[TR_ER].y +
	 end->vertex[TR_SL].y + end->vertex[TR_SR].y) / 4.0;;

      t_endpt.x = (t_end->vertex[TR_EL].x + t_end->vertex[TR_ER].x) / 2.0;
      t_endpt.y = (t_end->vertex[TR_EL].y + t_end->vertex[TR_ER].y) / 2.0;;
    }
#endif


  /* find offset from outer chord to apex */
  vec2f apex;

  apex = findApex( t_start, t_end, lseg[index][num_lseg[index]].arc/2.0);

  lseg[index][num_lseg[index]].apex.x = apex.x;
  lseg[index][num_lseg[index]].apex.y = apex.y;


  float start_rot, end_rot;

  t3Dd radial;

  if ( t_start->type == TR_RGT )
    {
      radial.x = t_startpt.x - lseg[index][num_lseg[index]].t_start->center.x;
      radial.y = t_startpt.y - lseg[index][num_lseg[index]].t_start->center.y;

#if SCAN_DEBUG
      printf("cl: start: %f %f cent: %f %f rad: %f %f\n",
	     t_startpt.x,
	     t_startpt.y,
	     lseg[index][num_lseg[index]].t_start->center.x,
	     lseg[index][num_lseg[index]].t_start->center.y,
	     radial.x,
	     radial.y);
#endif

      //start_rot = atan2f( radial.y, radial.x) - ( PI / 2.0);
      start_rot = (PI / 2.0) - atan2f( radial.y, radial.x);
      NORM_PI_PI(start_rot);

#if SCAN_DEBUG
      printf("cl: start_rot: %f\n",
	     start_rot * RAD_TO_DEG);
#endif

      radial.x = t_endpt.x - lseg[index][num_lseg[index]].t_end->center.x;
      radial.y = t_endpt.y - lseg[index][num_lseg[index]].t_end->center.y;

#if SCAN_DEBUG
      printf("cl: end: %f %f cent: %f %f rad: %f %f\n",
	     t_endpt.x,
	     t_endpt.y,
	     lseg[index][num_lseg[index]].t_end->center.x,
	     lseg[index][num_lseg[index]].t_end->center.y,
	     radial.x,
	     radial.y);
#endif


      //end_rot = atan2f( radial.y, radial.x) - ( PI / 2.0);;
      end_rot = (PI / 2.0 ) - atan2f( radial.y, radial.x);
      NORM_PI_PI(end_rot);

#if SCAN_DEBUG
      printf("cl: end_rot: %f\n",
	     end_rot * RAD_TO_DEG);
#endif
    }
  else /* TR_LFT */
    {
      radial.x = t_startpt.x - lseg[index][num_lseg[index]].t_start->center.x;
      radial.y = t_startpt.y - lseg[index][num_lseg[index]].t_start->center.y;

#if SCAN_DEBUG
      printf("cl: start: %f %f cent: %f %f rad: %f %f\n",
	     t_startpt.x,
	     t_startpt.y,
	     lseg[index][num_lseg[index]].t_start->center.x,
	     lseg[index][num_lseg[index]].t_start->center.y,
	     radial.x,
	     radial.y);
#endif

      start_rot = -1.0 * (atan2f( radial.y, radial.x) + ( PI / 2.0));
      NORM_PI_PI(start_rot);

#if SCAN_DEBUG
      printf("cl: start_rot: %f\n",
	     start_rot * RAD_TO_DEG);
#endif

      radial.x = t_endpt.x - lseg[index][num_lseg[index]].t_end->center.x;
      radial.y = t_endpt.y - lseg[index][num_lseg[index]].t_end->center.y;

#if SCAN_DEBUG
      printf("cl: end: %f %f cent: %f %f rad: %f %f\n",
	     t_endpt.x,
	     t_endpt.y,
	     lseg[index][num_lseg[index]].t_end->center.x,
	     lseg[index][num_lseg[index]].t_end->center.y,
	     radial.x,
	     radial.y);
#endif
      end_rot = -1.0 * (atan2f( radial.y, radial.x) + ( PI / 2.0));
      NORM_PI_PI(end_rot);

#if SCAN_DEBUG
      printf("cl: end_rot: %f\n",
	     end_rot * RAD_TO_DEG);
#endif
    }



#if SCAN_DEBUG
  printf("cl: %d ======== entry =========\n",num_lseg[index]);
#endif
  lseg[index][num_lseg[index]].start_rot = start_rot;

  findClothoid( lseg[index][num_lseg[index]].startpt,
		lseg[index][num_lseg[index]].apex,
		lseg[index][num_lseg[index]].arc / 2.0,
		start_rot,
		&(lseg[index][num_lseg[index]].sign_ex),
		&(lseg[index][num_lseg[index]].sign_ey),
		&(lseg[index][num_lseg[index]].apex1_t),
		&(lseg[index][num_lseg[index]].entry_t),
		&(lseg[index][num_lseg[index]].entry_a),
		&(lseg[index][num_lseg[index]].entry_s),
		&(lseg[index][num_lseg[index]].run_in) );

#if SCAN_DEBUG
  printf("cl: %d ======== exit =========\n",num_lseg[index]);
#endif
  lseg[index][num_lseg[index]].end_rot = end_rot;

  findClothoid( lseg[index][num_lseg[index]].endpt,
		lseg[index][num_lseg[index]].apex,
		lseg[index][num_lseg[index]].arc / 2.0,
		end_rot,
		&(lseg[index][num_lseg[index]].sign_xx),
		&(lseg[index][num_lseg[index]].sign_xy),
		&(lseg[index][num_lseg[index]].apex2_t),
		&(lseg[index][num_lseg[index]].exit_t),
		&(lseg[index][num_lseg[index]].exit_a),
		&(lseg[index][num_lseg[index]].exit_s),
		&(lseg[index][num_lseg[index]].run_out));
		

  num_lseg[index]++;

/* FIXME:  There is something wrong? MAX_LINE_SEGS set to 60
	doesnt works if the lenght track as more than 10 000 meters
   So that doesnt works on Spring track!				*/

  if ( num_lseg[index] >= MAX_LINE_SEGS)
    printf("hymie_2016: %s: num_lseg[index] overflow\n",__FUNCTION__);
}

/**********************************************************/


void Line::addSegUnknown(int index,
			 tTrackSeg *start,
			 tTrackSeg *end)
{
  lseg[index][num_lseg[index]].type     = SEG_UNK;

  lseg[index][num_lseg[index]].startseg = start;
  lseg[index][num_lseg[index]].endseg   = end;

  lseg[index][num_lseg[index]].fromstart= start->lgfromstart;

#if SCAN_DEBUG
  printf("*****************************\n");
  printf("AddSegUnknown: start:%d end:%d fromstart:%f\n",
	 start->id,
	 end->id,
	 lseg[index][num_lseg[index]].fromstart);
  printf("*****************************\n");
#endif

  
  num_lseg[index]++;

  if ( num_lseg[index] >= MAX_LINE_SEGS)
    printf("hymie_2016: %s: num_lseg[index] overflow\n",__FUNCTION__);
}


/**********************************************************/

vec2f Line::findApex(tTrackSeg *start,
		    tTrackSeg *end,
		    float targ_arc )
{
  float tot_arc  = 0.0;
  float arc;

  float car_width;

  car_width =  my_car_width;

  tTrackSeg *cs=start;

  while ( tot_arc < targ_arc )
    {
      tot_arc += cs->arc;
      cs = cs->next;
    }

  /* now past apex by tot_arc - targ_arc */
  cs = cs->prev;
  tot_arc -= cs->arc;
  
  arc = targ_arc - tot_arc;
  float arcsign = (cs->type == TR_RGT) ? -1.0f : 1.0f;
  arc = arc * arcsign;

  vec2f s,n,c;

  /* find inside */
  if ( cs->type == TR_RGT )
    {
      s.x = cs->vertex[TR_SR].x;
      s.y = cs->vertex[TR_SR].y;

      n.x = cs->vertex[TR_SL].x - cs->vertex[TR_SR].x;
      n.y = cs->vertex[TR_SL].y - cs->vertex[TR_SR].y;
      n.normalize();
    }
  else
    {
      s.x = cs->vertex[TR_SL].x;
      s.y = cs->vertex[TR_SL].y;

      n.x = cs->vertex[TR_SR].x - cs->vertex[TR_SL].x;
      n.y = cs->vertex[TR_SR].y - cs->vertex[TR_SL].y;
      n.normalize();
    }
  
#if HACKS
  if ( start->id == 383 )
    s = s + n * (WIDTH_FACT + 1.0) * car_width;
  else if ( start->id == 388 )
    s = s + n * (WIDTH_FACT - 1.0) * car_width;
  else
#endif
    s = s + n * WIDTH_FACT * car_width;
  
  c.x = cs->center.x;
  c.y = cs->center.y;

  s = s.rotate(c, arc);

  return s;
}

/**********************************************************/

vec2f Line::findOutside(tTrackSeg *start,
			tTrackSeg *end,
			float targ_arc )
{
  float tot_arc  = 0.0;
  float arc;

  float car_width;

  car_width =  my_car_width;

  tTrackSeg *cs=start;

  while ( tot_arc < targ_arc )
    {
      tot_arc += cs->arc;
      cs = cs->next;
    }
  /* now past midpoint by tot_arc - targ_arc */
  cs = cs->prev;
  tot_arc -= cs->arc;
  
  arc = targ_arc - tot_arc;
  float arcsign = (cs->type == TR_RGT) ? -1.0f : 1.0f;
  arc = arc * arcsign;

  vec2f s,n,c;

  /* find outside */
  if ( cs->type == TR_RGT )
    {
      s.x = cs->vertex[TR_SL].x;
      s.y = cs->vertex[TR_SL].y;

      n.x = cs->vertex[TR_SL].x - cs->vertex[TR_SR].x;
      n.y = cs->vertex[TR_SL].y - cs->vertex[TR_SR].y;
      n.normalize();
    }
  else
    {
      s.x = cs->vertex[TR_SR].x;
      s.y = cs->vertex[TR_SR].y;

      n.x = cs->vertex[TR_SR].x - cs->vertex[TR_SL].x;
      n.y = cs->vertex[TR_SR].y - cs->vertex[TR_SL].y;
      n.normalize();
    }

  s = s - n * WIDTH_FACT * car_width;

  c.x = cs->center.x;
  c.y = cs->center.y;

  s = s.rotate(c, arc);

  return s;
}

/**********************************************************/

void Line::circle3point( vec2f point1,
			 vec2f point2,
			 vec2f point3,
			 vec2f *center,
			 float *radius)
{
  float a, b, c, d, e, f;
  float a2b2, c2d2, e2f2;
  float h, k, r;

  a = point1.x;
  b = point1.y;

  c = point2.x;
  d = point2.y;

  e = point3.x;
  f = point3.y;

  a2b2 = a*a + b*b;
  c2d2 = c*c + d*d;
  e2f2 = e*e + f*f;

  k = 0.5 * ( a2b2*(e-c) + c2d2*(a-e) + e2f2*(c-a)) /
    ( b*(e-c) + d*(a-e) + f*(c-a));

  h = 0.5 * ( a2b2*(f-d) + c2d2*(b-f) + e2f2*(d-b)) /
    ( a*(f-d) + c*(b-f) + e*(d-b));

  r = sqrt( (a-h)*(a-h) + (b-k)*(b-k));

  center->x = h;
  center->y = k;

  *radius = r;

#if 0
  printf("c2p: %f,%f R:%f\n",h,k,r);
#endif
}
/**********************************************************/

void Line::findClothoid( vec2f start,
			 vec2f apex,
			 float angle,
			 float rotation,
			 float *sign_x,
			 float *sign_y,
			 float *t_a,
			 float *t_e,
			 float *a,
			 float *len,
			 float *run_in)
{
  float t_apex;
  /* use angle to find tangent point */
  t_apex = sqrt( 2.0 * angle );

  *t_a = t_apex;

#if SCAN_DEBUG
  printf("cl: ang:%f(%f) t:%f\n",
	 angle, angle * RAD_TO_DEG,
	 t_apex);
#endif

  float x_apex, y_apex;;

  x_apex = cloth_x(t_apex);
  y_apex = cloth_y(t_apex);


#if SCAN_DEBUG
  printf("cl: apex: t:%f x:%f y:%f\n",
	 t_apex,
	 x_apex,
	 y_apex);
#endif


#if SCAN_DEBUG
  printf("cl: start x:%f y:%f apex x:%f y:%f \n",
	 start.x, start.y,
	 apex.x, apex.y);
#endif

  vec2f span;
  float delx, dely;

  span.x = (apex.x - start.x);
  span.y = (apex.y - start.y);

#if SCAN_DEBUG
  printf("cl: pre-rot  span x:%f y:%f rot:%f\n",
	 span.x,
	 span.y,
	 rotation*RAD_TO_DEG);
#endif

  vec2f span2, axis;

  axis.x = 0.0; axis.y = 0.0;

  span2 = span.rotate( axis, rotation);

#if SCAN_DEBUG
  printf("cl: post-rot span x:%f y:%f\n",
	 span2.x,
	 span2.y);
#endif

  delx = fabs(span2.x);
  dely = fabs(span2.y);

  *sign_x = span2.x / delx;
  *sign_y = span2.y / dely;

#if SCAN_DEBUG
  printf("cl: delta span x:%f y:%f xs:%f ys:%f\n",
	 delx,
	 dely,
	 *sign_x,
	 *sign_y);
#endif

#if 1
  /* fit clothoid to start */
  float t_trial;
  float MSE_entry = FLT_MAX;
  float x_delta;
  float t_entry, x_entry, y_entry, a_entry;

  x_entry = 0.0;
  y_entry = 0.0;
  a_entry = 0.0;
  t_entry = 0.0;

  for ( t_trial=0.0; t_trial < t_apex; t_trial+=DT )
    {
      x_entry = cloth_x( t_trial );
      y_entry = cloth_y( t_trial );

      a_entry = dely/(y_apex-y_entry);

      x_delta = a_entry * (x_apex - x_entry) - delx;

      if ( x_delta * x_delta < MSE_entry )
	{
	  MSE_entry = x_delta * x_delta;
	  t_entry = t_trial;
#if 0
	  printf("cl: entry t:%f x:%f y:%f a:%f dx:%f dy:%f mse:%f run-in:%f\n",
		 t_entry, 
		 x_entry,
		 y_entry,
		 a_entry,
		 a_entry * (x_apex - x_entry) - delx,
		 a_entry * (y_apex - y_entry) - dely,
		 MSE_entry,
		 a_entry * (x_apex - x_entry) );
#endif
	}
    }
  x_entry = cloth_x( t_entry );
  y_entry = cloth_y( t_entry );
  a_entry = dely/(y_apex-y_entry);


  *run_in = (delx - x_apex * a_entry);
#if 0
  if ( *run_in > 0.0 )
    printf("findCloth: run_in%f\n",*run_in);
#endif

#if SCAN_DEBUG
  printf("cl: apex t:%f x:%f(%f) y:%f(%f)\n",
	 t_apex, 
	 x_apex, x_apex * a_entry,
	 y_apex, y_apex * a_entry);


  printf("cl: run-in:%5.2f (delx:%5.2f x:%5.2f)\n",
	 *run_in,
	 delx,
	 x_apex * a_entry);
  if ( delx > x_apex * a_entry )
    {
      printf("cl: ******** overrun of %5.2f on entry (delx:%5.2f x:%5.2f)\n",
	     *run_in,
	     delx,
	     x_apex * a_entry);
    }

  printf("cl: entry t:%f a:%f x:%f(%f) y:%f(%f) dx:%f dy:%f mse:%f run-in:%f turn-in:%f(%f)\n",
	 t_entry, 
	 a_entry,
	 x_entry, x_entry * a_entry,
	 y_entry, y_entry * a_entry,
	 a_entry * (x_apex - x_entry) - delx,
	 a_entry * (y_apex - y_entry) - dely,
	 MSE_entry,
	 a_entry * (x_apex - x_entry),
	 t_entry * t_entry / 2.0,
	 t_entry * t_entry / 2.0 * RAD_TO_DEG );
#endif
  *t_e = t_entry;
  *a = a_entry;
  *len = a_entry * (x_apex - x_entry);

#endif


#if SCAN_DEBUG
  printf("cl: t_a:%f t_e:%f a:%f s:%f\n",
	 *t_a,
	 *t_e,
	 *a,
	 *len);
#endif
}

/*===============================================================
 *
 * 		utility functions
 *
 *===============================================================*/

float Line::cloth_y(float t)
{
  int i;
  float y;

  i = (int)floor(t / DT);

  if ( i < 0 ) i = 0;
  if ( i+1 >= T_SIZE ) i = T_SIZE - 2;

  y = cloth_s[i] + ( cloth_s[i+1] - cloth_s[i]) *
    ( t - ((float)i * (float)DT) ) / DT;

#if 0
  printf("cly: t:%f cly:%f (%f-%f)\n",
	 t,
	 y,
	 cloth_s[i],
	 cloth_s[i+1]);
#endif

  return y;
}

/**********************************************************/

void Line::initCloth(void)
{
  int i;
  double t;
  
  double sigma_s, sigma_c;

  sigma_s = 0.0;
  sigma_c = 0.0;
  
  for ( i = 0; i < T_SIZE; i++ )
  {
    t = (double)i * (double)DT;

    sigma_s += sin( t * t / 2.0 ) * DT; 
    sigma_c += cos( t * t / 2.0 ) * DT;

    cloth_s[i] = sigma_s;
    cloth_c[i] = sigma_c;
  }
}

/**********************************************************/

float Line::cloth_x(float t)
{
  int i;
  float x;

  i = (int)floor(t / DT);

  if ( i < 0 ) i = 0;
  if ( i+1 >= T_SIZE ) i = T_SIZE - 2;

  x = cloth_c[i] + ( cloth_c[i+1] - cloth_c[i]) *
    ( t - ((float)i * (float)DT) ) / DT;

#if 0
  printf("clx: t:%f clx:%f (%f-%f)\n",
	 t,
	 x,
	 cloth_c[i],
	 cloth_c[i+1]);
#endif

  return x;
}

/*===============================================================
 *
 * 			Save Dump Data
 *
 *===============================================================*/

void Line::dumpTrack(tTrack *t)
{
  FILE *tfile;

  char filename[256];
	sprintf(filename, "%sdrivers/hymie_2016/data/%s", GetLocalDir(), TRACK_DATA);

  tfile = fopen( filename, "w");
  
  if ( NULL == tfile )
    {
      printf("Failed to open %s for writing\n", filename);
      return;
    }
  else
    printf("Dumping track data to %s\n", filename);

  dumpTrackLeft(tfile, t);
  dumpTrackRight(tfile, t);

  fflush(tfile);
  fclose(tfile);
}

/**********************************************************/

void Line::dumpTrackLeft(FILE *tfile, tTrack *track)
{
  tTrackSeg *cs, *startseg = track->seg;


  /* hack */
  startseg = startseg->next;

  cs = startseg;

  fprintf(tfile,"#=================================\n");
  fprintf(tfile,"#Dumping Track/Left\n");

  do
    {
      fprintf(tfile,"%f %f\n",
	     cs->vertex[TR_SL].x,
	     cs->vertex[TR_SL].y);
      cs = cs->next;
    }
  while (cs != startseg);

  /* grab last set of points */
  cs = cs->prev;

  fprintf(tfile,"%f %f\n",
	 cs->vertex[TR_EL].x,
	 cs->vertex[TR_EL].y);

  fprintf(tfile,"#Done\n");
  fprintf(tfile,"#=================================\n");
  fprintf(tfile,"\n");
}

/**********************************************************/

void Line::dumpTrackRight(FILE *tfile, tTrack *track)
{
  tTrackSeg *cs, *startseg = track->seg;


  /* hack */
  startseg = startseg->next;

  cs = startseg;

  fprintf(tfile,"#=================================\n");
  fprintf(tfile,"#Dumping Track/Right\n");

  do
    {
      fprintf(tfile,"%f %f\n",
	     cs->vertex[TR_SR].x,
	     cs->vertex[TR_SR].y);
      cs = cs->next;
    }
  while (cs != startseg);

  /* grab last set of points */
  cs = cs->prev;

  fprintf(tfile,"%f %f\n",
	 cs->vertex[TR_ER].x,
	 cs->vertex[TR_ER].y);

  fprintf(tfile,"#Done\n");
  fprintf(tfile,"#=================================\n");
  fprintf(tfile,"\n");  
}


/**********************************************************/

void Line::dumpCloth(char *filedata )
{
  FILE *cfile;
  double x;
  
  double sigma_s, sigma_c;
  double dx, x_max;
  double c_max, s_max;

  char filename[256];
	sprintf(filename, "%sdrivers/hymie_2016/data/%s", GetLocalDir(), filedata);

  cfile = fopen(filename, "w");

  if ( NULL == cfile )
    {
      printf("Failed to open %s for wiritng\n",filename);
      return;
    }
  printf("Writing clothoid data to %s\n",filename);

  dx = 0.01;

  //x_max = sqrt ( PI / 2.0 );
  x_max  = sqrt ( 2 * PI );

  fprintf(cfile,"#=================================\n");
  fprintf(cfile,"# Clothoid params x c s\n");
  fprintf(cfile,"# x_max: %f\n",x_max);

  //x_max += 5.0 * dx ;

  sigma_s = 0.0;
  sigma_c = 0.0;
  
  s_max = 0.0;
  c_max = 0.0;

  //for ( x = 0.0; x <= x_max; x+=dx )
	  for ( x = 0.0; x <= x_max; x+=dx )
  {
      
      sigma_s += sin( x * x / 2.0 ) * dx; 
      sigma_c += cos( x * x / 2.0 ) * dx;
      //fprintf(cfile,"%f %f %f\n",x,sigma_c, sigma_s);
      fprintf(cfile,"t:%f C:%f S:%f s:%f R:%f tang:%f\n",
	     x,
	     sigma_c, 
	     sigma_s,
	     x,
	     1.0 / x,
	     (x * x / 2.0) * RAD_TO_DEG);
      

      //if (sigma_s > s_max) s_max = sigma_s;
      //if (sigma_c > c_max) c_max = sigma_c;
      if (sigma_c > c_max) 
	{
	  c_max = sigma_c;
	  s_max = sigma_s;
	}
    }

  fprintf(cfile,"# c_max:%f s_max:%f\n",c_max,s_max);
  fprintf(cfile,"#Done\n");
  fprintf(cfile,"#=================================\n");

  fflush(cfile);
  fclose(cfile);
}

/* ==========================================================================*/
void Line::dumpLine(char *filedata, int index )
{
 FILE *lfile = NULL;
 int curlineseg = 0;

  char filename[256];
	sprintf(filename, "%sdrivers/hymie_2016/data/%s", GetLocalDir(), filedata);


   lfile = fopen(filename, "w");

  if ( NULL == lfile )
    {
      printf("Failed to open %s for writing\n",filename);
      return;
    }
  printf("Dumping line  data to %s\n",filename);

  fprintf(lfile,"#=================================\n");
  fprintf(lfile,"#Dumping Lines\n");

  while (curlineseg < num_lseg[index] )
    {
      if (lseg[index][curlineseg].type == SEG_STR)
	{
	  fprintf(lfile,"#Lineseg:%d STR type:%d\n",
		 curlineseg,
		 lseg[index][curlineseg].type);
	  fprintf(lfile,"%f %f\n",
		 lseg[index][curlineseg].startpt.x,
		 lseg[index][curlineseg].startpt.y);
	  fprintf(lfile,"%f %f\n",
		 lseg[index][curlineseg].endpt.x,
		 lseg[index][curlineseg].endpt.y);
	  fprintf(lfile,"\n");
	}
      else if ( lseg[index][curlineseg].type == SEG_KINK )
	{
	  fprintf(lfile,"#Lineseg:%d KINK type:%d\n",
		 curlineseg,
		 lseg[index][curlineseg].type);
	  fprintf(lfile,"%f %f\n",
		 lseg[index][curlineseg].startpt.x,
		 lseg[index][curlineseg].startpt.y);
	  fprintf(lfile,"%f %f\n",
		 lseg[index][curlineseg].endpt.x,
		 lseg[index][curlineseg].endpt.y);
	  fprintf(lfile,"\n");
	}
      else if ( lseg[index][curlineseg].type == SEG_CL )
	{
	  float t, t_start, t_inc;
	  float x_orig, y_orig;
	  float sign_x, sign_y;
	  float a;
	  float rot;

	  vec2f origin, radial, shift;

	  origin.x = 0.0;
	  origin.y = 0.0;

	  fprintf(lfile,"#Lineseg:%d CL type:%d\n",
		 curlineseg,
		 lseg[index][curlineseg].type);

	  /* --- segment 1 --- */
	  fprintf(lfile,"#Lineseg:%d-entry CL1 t_a:%f t_e:%f a:%f\n",
		 curlineseg,
		 lseg[index][curlineseg].apex1_t,
		 lseg[index][curlineseg].entry_t,
		 lseg[index][curlineseg].entry_a);

	  a = lseg[index][curlineseg].entry_a;

#if DUMP_CLOTH_TAILS 
	  t_start = 0.0;
#else
	  t_start = lseg[index][curlineseg].entry_t;
#endif
	  t_inc = 10.0 / a;

	  x_orig = cloth_x(lseg[index][curlineseg].entry_t) * a;
	  y_orig = cloth_y(lseg[index][curlineseg].entry_t) * a;
	  	  
	  if ( lseg[index][curlineseg].run_in > 0.0 )
	    x_orig -= lseg[index][curlineseg].run_in;

	  shift.x = lseg[index][curlineseg].startpt.x;
	  shift.y = lseg[index][curlineseg].startpt.y;

	  rot =  -1.0 * lseg[index][curlineseg].start_rot;
	  
	  sign_x = lseg[index][curlineseg].sign_ex;
	  sign_y = lseg[index][curlineseg].sign_ey;


	  for ( t = t_start; t < lseg[index][curlineseg].apex1_t; t+=t_inc )
	      {
		radial.x = cloth_x(t) * a - x_orig;
		radial.y = cloth_y(t) * a - y_orig;

		radial.x *= sign_x;
		radial.y *= sign_y;

		radial = radial.rotate( origin, rot) ;

		//fprintf(lfile,"# post-rot %f %f\n", radial.x, radial.y);

		radial = radial + shift;

		fprintf(lfile,"%f %f\n", radial.x, radial.y);
	      }
	  fprintf(lfile,"%f %f\n\n",
		  lseg[index][curlineseg].apex.x,
		  lseg[index][curlineseg].apex.y);

	  /* --- segment 2 --- */
	  fprintf(lfile,"#Lineseg:%d-exit CL2 t_a:%f t_e:%f a:%f\n",
		 curlineseg,
		 lseg[index][curlineseg].apex2_t,
		 lseg[index][curlineseg].exit_t,
		 lseg[index][curlineseg].exit_a);

	  a = lseg[index][curlineseg].exit_a;
	  
#if DUMP_CLOTH_TAILS
	  t_start = 0.0;
#else
	  t_start = lseg[index][curlineseg].exit_t;
#endif
	  t_inc = 10.0 / a;

	  x_orig = cloth_x(lseg[index][curlineseg].exit_t) * a;
	  y_orig = cloth_y(lseg[index][curlineseg].exit_t) * a;
	  
	  if ( lseg[index][curlineseg].run_out > 0.0 )
	    x_orig -= lseg[index][curlineseg].run_out;

	  shift.x = lseg[index][curlineseg].endpt.x;
	  shift.y = lseg[index][curlineseg].endpt.y;

	  rot = -1.0 * lseg[index][curlineseg].end_rot;
	  NORM_PI_PI(rot);

	  sign_x = lseg[index][curlineseg].sign_xx;
	  sign_y = lseg[index][curlineseg].sign_xy;


	  for ( t = t_start; t < lseg[index][curlineseg].apex2_t; t+=t_inc )
	      {
		radial.x = cloth_x(t) * a - x_orig;
		radial.y = cloth_y(t) * a - y_orig;

		radial.x *= sign_x;
		radial.y *= sign_y;

		radial = radial.rotate( origin, rot) ;

		//fprintf(lfile,"# post-rot %f %f\n", radial.x, radial.y);

		radial = radial + shift;

		fprintf(lfile,"%f %f\n", radial.x, radial.y);
	      }
	  fprintf(lfile,"%f %f\n\n",
		  lseg[index][curlineseg].apex.x,
		  lseg[index][curlineseg].apex.y);


	  fprintf(lfile,"\n");
	}
      else if ( lseg[index][curlineseg].type == SEG_UNK )
	{
	  fprintf(lfile,"#\n");
	  fprintf(lfile,"#Lineseg:%d UNKNOWN TYPE\n",curlineseg);
	  fprintf(lfile,"#\n");
	  fprintf(lfile,"\n");
	}

      curlineseg++;
    }

  fprintf(lfile,"#Done\n");
  fprintf(lfile,"#=================================\n");

  fflush(lfile);
  fclose(lfile);
}


