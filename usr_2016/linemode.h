/***************************************************************************

    file                 : linemode.h
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: linemode.h,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef __LINEMODE_H
#define __LINEMODE_H

#include <raceman.h>
#include <tgf.h>
#include <car.h>
#include <robottools.h>
#include <robot.h>

#include "vardef.h"
#include "manual_override.h"
#include "xmldefs.h"

enum { LINE_MID=0, LINE_LEFT, LINE_RIGHT, LINE_RL, LINE_RL_MID, LINE_RL_SLOW, LINE_LEFT_OUTSTEER, LINE_RIGHT_OUTSTEER, LINE_NONE };

class LRaceLine;
class LLineMode {
    public:
        LLineMode(tCarElt *thecar, LManualOverrideCollection *overrideCollection);
        ~LLineMode();

        int getSourceLine() { return source_line; }
        int getTargetLine();
        void setTargetLineToSource() { source_line = target_line; }
        void setRecoveryToRaceLine();
        void setRecoveryToNearest();
        void setAvoidanceToLeft();
        void setAvoidanceToRight();
        void setAvoidanceToMid();
        void checkTransition(double targetSteer, double steer);
        void setPitting();
        void incrementPercentageTransition();
        void updateSituation(tSituation *s);
        void setRaceLine(LRaceLine *rl) { raceline = rl; }
	void setOnHold() { 
		if (!onHold) resetTimer();
		onHold = true;
	}
        double getTransitionTime() { return currentTime - timer_started; }
	double transitionIncrement(int div, double sourceSteer, double targetSteer);
        bool isOnHold(int div, bool stay_inside);
        double transition_lanediff;
        bool is_transitioning;
	bool has_transitioned;

    private:
	LManualOverrideCollection *overrideCollection;
        tCarElt *car;
        LRaceLine *raceline;
        double timer_started;
	double currentTime;
        float angle;
        int hold_apex_div;
        int target_line;
        int source_line;
	int prefer_line;
        double transition_increment;
	bool onHold;

        void resetTimer();
        void updateAngle();
};

#endif // __LINEMODE_H
