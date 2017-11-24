/***************************************************************************

    file                 : linemode.cpp
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: linemode.cpp,v 1.0 2015/02/07 20:11:49 andrew Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <iostream>
#include <math.h>
#include "car.h"
#include "track.h"
#include "linemode.h"
#include "raceline.h"
#include "xmldefs.h"

//#define LINEMODE_DEBUG

//
// Public methods
//
LLineMode::LLineMode(tCarElt *theCar, LManualOverrideCollection *oc)
{
    is_transitioning = true;
    onHold = false;
    target_line = LINE_NONE;
    source_line = LINE_NONE;
    car = theCar;
    transition_increment = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_TRANSITION_INC, (char *) NULL, 0.1);
    transition_lanediff = 100.0;
    hold_apex_div = -1;
    raceline = NULL;
    prefer_line = LINE_MID;
    has_transitioned = false;
    overrideCollection = oc;
    timer_started = currentTime = 0.0;
}

LLineMode::~LLineMode()
{
}

double LLineMode::transitionIncrement(int div, double sourceSteer, double targetSteer)
{
    double ti = transition_increment;
    bool overridden = false;

    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LFT_TRANS_INC);
        if (labelOverride && sourceSteer < targetSteer)
        {
            if (!labelOverride->getOverrideValue(div, &ti))
                ti = transition_increment;
            else
                overridden = true;
        }

        labelOverride = overrideCollection->getOverrideForLabel(PRV_RGT_TRANS_INC);
        if (labelOverride && sourceSteer > targetSteer)
        {
            if (!labelOverride->getOverrideValue(div, &ti))
                ti = transition_increment;
            else
                overridden = true;
        }

        if (!overridden)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_TRANSITION_INC);
            if (labelOverride)
            {
                if (!labelOverride->getOverrideValue(div, &ti))
                    ti = transition_increment;
            }
        }
    }
    if (currentTime - timer_started < 1.0)
        return MAX(0.0, ti * ((currentTime - timer_started) / 1.0));
    return ti;
}

void LLineMode::updateSituation(tSituation *s)
{
    currentTime = s->currentTime;
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "LineMode: Source=%s Target=%s hold_div=%d / %d\n",(source_line==LINE_NONE?"None":(source_line==LINE_LEFT?"Left":(source_line==LINE_RIGHT?"Right":"RL"))),(target_line==LINE_NONE?"None":(target_line==LINE_LEFT?"Left":(target_line==LINE_RIGHT?"Right":"RL"))),hold_apex_div,(raceline?raceline->This:-1));fflush(stderr);
#endif
}

int LLineMode::getTargetLine()
{
    if (target_line == LINE_NONE)
    {
        if (car->_trkPos.toMiddle > 0.0)
            source_line = LINE_LEFT;
        else
            source_line = LINE_RIGHT;
        setRecoveryToRaceLine();
    }
    return target_line;
}

bool LLineMode::isOnHold(int div, bool stay_inside)
{
    //return false;
    if (is_transitioning && target_line == LINE_RL)
    { 
        double distance = 0.0;
        int hold_div = hold_apex_div;
        int this_prefer_line = raceline->findNextCorner(car, raceline->This, &hold_apex_div, &distance);
        if (distance > car->_speed_x * 2 || 
            (hold_div != hold_apex_div && (prefer_line != this_prefer_line)))
        {
            onHold = false;
            return false;
        }
        if (prefer_line != this_prefer_line)
        {
            if (prefer_line == LINE_MID)
                prefer_line = this_prefer_line;
            else
            {
                onHold = false;
                return false;
            }
        }
        if (//hold_apex_div > -1 && div != hold_apex_div &&
            ((prefer_line == TR_LFT && source_line == LINE_LEFT) || (prefer_line == TR_RGT && source_line == LINE_RIGHT)))
        {
            resetTimer();
            return true;
        }
    
    }
    hold_apex_div = -1;
    onHold = false;
    return false;
}

void LLineMode::setRecoveryToRaceLine()
{
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "setRecoveryToRaceLine\n");fflush(stderr);
#endif
    if (target_line != LINE_RL)
    {
        hold_apex_div = -1;
        int prefer_line = raceline->findNextCorner(car, raceline->This, &hold_apex_div);
        if ((prefer_line == TR_LFT && target_line == LINE_LEFT) || (prefer_line == TR_RGT && target_line == LINE_RIGHT))
        {
            if (hold_apex_div > -1)
            {
                if (hold_apex_div < raceline->This)
                    hold_apex_div += raceline->Divs;
                double distance = (hold_apex_div - raceline->This) * 3.0;
                if (distance < car->_speed_x * 5)
                {
                    if (distance > 6.0 || raceline->tSpeed[LINE_RL][raceline->Next] <= raceline->tSpeed[LINE_RL][raceline->This])
                    {
//fprintf(stderr, "DEFERRED CORRECTION TO RL!\n");
                        return;
                    }
                }
            }
            //else
            //   return;
        }
    }

    if (!is_transitioning && target_line != LINE_RL)
        is_transitioning = true;
    if (target_line != LINE_RL)
    {
        if (source_line == LINE_NONE)
            source_line = (car->_trkPos.toMiddle > 0.0 ? LINE_LEFT : LINE_RIGHT);
        else
            source_line = target_line;
        resetTimer();
        transition_lanediff = 100.0;
        int preferred_side = raceline->findNextCorner(car, raceline->This, &hold_apex_div);
        if (!((preferred_side == TR_LFT && target_line == LINE_LEFT) || (preferred_side == TR_RGT && target_line == LINE_RIGHT)))
            hold_apex_div = -1;
        /*
        else
        {
            int brakingdiv = raceline->findNextBrakingZone();
            if (brakingdiv > hold_apex_div || brakingdiv < raceline->This)
                hold_apex_div = -1;
            else
            {
                int divdiff = brakingdiv - raceline->This;
                if (divdiff < 0)
                    divdiff = (brakingdiv + raceline->Divs) - raceline->This;
                double distance = divdiff * raceline->DivLength;
                if (distance > car->_speed_x * 2)
                    hold_apex_div = -1;
            }
        }
        */
    }
    target_line = LINE_RL;
}

void LLineMode::setRecoveryToNearest()
{
    setRecoveryToRaceLine();
}

void LLineMode::setAvoidanceToLeft()
{
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "setAvoidanceToLeft\n");fflush(stderr);
#endif
    hold_apex_div = -1;
    if (!is_transitioning && target_line != LINE_LEFT)
        is_transitioning = true;
    if (target_line != LINE_LEFT)
    {
        resetTimer();
        transition_lanediff = 100.0;
        source_line = target_line;
    }
    target_line = LINE_LEFT;
}

void LLineMode::setAvoidanceToRight()
{
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "setAvoidanceToRight\n");fflush(stderr);
#endif
    hold_apex_div = -1;
    if (!is_transitioning && target_line != LINE_RIGHT)
        is_transitioning = true;
    if (target_line != LINE_RIGHT)
    {
        resetTimer();
        transition_lanediff = 100.0;
        source_line = target_line;
    }
    target_line = LINE_RIGHT;
}

void LLineMode::setAvoidanceToMid()
{
#ifdef LINEMODE_DEBUG
    fprintf(stderr, "setAvoidanceToMid\n");fflush(stderr);
#endif
    hold_apex_div = -1;
    if (!is_transitioning && target_line != LINE_MID)
    {
        resetTimer();
        is_transitioning = true;
        source_line = target_line;
    }
    target_line = LINE_MID;
}

void LLineMode::setPitting()
{
    hold_apex_div = -1;
    resetTimer();
    if (car->_trkPos.toLeft < 0.0)
        source_line = LINE_LEFT;
    else if (car->_trkPos.toRight < 0.0)
        source_line = LINE_RIGHT;
    target_line = LINE_RL;
    is_transitioning = true;
}

void LLineMode::checkTransition(double targetSteer, double steer)
{
    // this should be called from the Raceline class when its isOnLine() method returns true
    if (is_transitioning)
    {
        updateAngle();

        if (fabs(angle) < 0.2f &&
            fabs(targetSteer) < 0.7 &&
            fabs(steer - targetSteer) < 0.3 * transition_increment &&
            car->_accel_x > 0.0 &&
            ((fabs(car->_trkPos.toMiddle) < car->_trkPos.seg->width / 2 - 0.5) || car->_speed_x < 10.0))
        {
            std::cout << "TRANSITION FINAL" << std::endl;
            source_line = target_line;
            is_transitioning = has_transitioned = false;
            transition_lanediff = 100.0;
        }
    }
}

void LLineMode::incrementPercentageTransition()
{
#if 0
    if (hold_apex_div != -1) return;
    double increment = 0.0;
    if (transition_lanediff < 0.1 || transition_lanediff > 0.9)
        increment = transition_increment * 0.25;
    else if (transition_lanediff < 0.2 || transition_lanediff > 0.8)
        increment = transition_increment * 0.5;
    else if (transition_lanediff < 0.3 || transition_lanediff > 0.7)
        increment = transition_increment * 0.75;
    else 
        increment = transition_increment;
    transition_lanediff = MIN(1.0, transition_lanediff + increment);
#endif
}

//
// Private methods
//
void LLineMode::updateAngle()
{
    float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
    if (angle < 0.0)
        angle = -angle;
}

void LLineMode::resetTimer()
{
    timer_started = currentTime;
    //prefer_line = LINE_MID;
}
