/***************************************************************************

    file                 : pit.cpp
    created              : Thu Mai 15 2:43:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: pit.cpp,v 1.11 2006/10/12 21:26:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>

#include "pit.h"
#include "xmldefs.h"

const float Pit::SPEED_LIMIT_MARGIN = 0.5;        // [m/s] safety margin to avoid pit speeding.

#define PIT_HACK_WIDTH 0.0
#define PIT_LANE_HACK_WIDTH -2.0

Pit::Pit(tSituation *s, Driver *driver, float pitoffset)
{
    track = driver->getTrackPtr();
    car = driver->getCarPtr();
    mypit = driver->getCarPtr()->_pit;
    pitinfo = &track->pits;
    pitstop = inpitlane = false;
    pittimer = 0.0;
    float pitspeedmargin = 0.0f;
    float pitstartoverride0 = -1.0f, pitstartoverride1 = -1.0f, pitstartoverride2 = -1.0f, pitexitoverride = -1.0f;
    float pitentryoffset = 0.0f, pitexitoffset = 0.0f;
    usepitmaxspeed = false;
    pitmaxspeed = pitmaxspeedoffset = 0.0f;

    pitspeedmargin = GfParmGetNum( car->_carHandle, "hymie_2015", "pitspeedmargin", (char*)NULL, SPEED_LIMIT_MARGIN);
    pitstartextralength = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXTRA_LENGTH, (char*)NULL, 0.0f);
    pitstartoverride0 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE0, (char*)NULL, -1.0f);
    pitstartoverride1 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE1, (char*)NULL, -1.0f);
    pitstartoverride2 = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_START_OVERRIDE2, (char*)NULL, -1.0f);
    pitexitoverride = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXIT_OVERRIDE, (char*)NULL, -1.0f);
    pitentryoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_ENTRY_OFFSET, (char*)NULL, 0.0f);
    pitexitoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_EXIT_OFFSET, (char*)NULL, 0.0f);
    pitmaxspeedoffset = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_MAX_SPEED_OFFSET, (char*)NULL, 0.0f);
    pitmaxspeed = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_PIT_MAX_SPEED, (char*)NULL, 0.0f);

    usepitmaxspeed = (pitmaxspeed > 0.01f && pitmaxspeedoffset > 1.0f);

    if (mypit != NULL) 
    {
        speedlimit = pitinfo->speedLimit - pitspeedmargin;
        speedlimitsqr = speedlimit*speedlimit;
        pitspeedlimitsqr = pitinfo->speedLimit*pitinfo->speedLimit;

        // Compute pit spline points along the track.
        p[5].x = mypit->pos.seg->lgfromstart + mypit->pos.toStart;
        p[4].x = p[5].x - pitinfo->len;
        p[6].x = p[5].x + pitinfo->len;
        p[0].x = pitinfo->pitEntry->lgfromstart + pitoffset;
        // HACK! pit entry on corkscrew
        p[1].x = p[0].x;
        p[2].x = p[1].x;
        if (pitstartoverride0 > 0.0)
            p[0].x = pitstartoverride0;
        if (pitstartoverride1 > 0.0)
            p[1].x = pitstartoverride1;
        if (pitstartoverride2 > 0.0)
            p[2].x = pitstartoverride2;
        p[3].x = pitinfo->pitStart->lgfromstart; 
        if (pitstartoverride0 < 0.0)
            p[1].x = pitinfo->pitEntry->lgfromstart + pitoffset + pitstartextralength;
        //p[5].x = pitinfo->pitEnd->lgfromstart + pitinfo->len/2.0f;
        //p[5].x = p[3].x + (pitInfo->nMaxPits - car->index) * pitinfo->len;
        p[7].x = p[3].x + pitinfo->nMaxPits * pitinfo->len;
        p[8].x = pitinfo->pitExit->lgfromstart;
        if (pitexitoverride > 0.0)
            p[8].x = pitexitoverride;

        pitentry = p[0].x;
        pitexit = p[8].x;
        pitstopentry = p[4].x;
        pitstopexit = p[6].x;

        // Normalizing spline segments to >= 0.0.
        int i;
        for (i = 0; i < NPOINTS; i++) {
            //fprintf(stderr, "A: %d - %.1f\n",i, p[i].x);
            p[i].s = 0.0;

            
            p[i].x = toSplineCoord(p[i].x);

            }

        // Fix broken pit exit.
        if (p[8].x < p[7].x) {
            
            p[8].x = p[7].x + 50.0;
        }

        // Fix point for first pit if necessary.
        if (p[3].x > p[4].x) {
            p[3].x = p[4].x - 3.0;
        }
        if (p[2].x > p[3].x) {
            p[2].x = p[3].x - 3.0;
            pitentry = p[2].x;
        }

        // Fix point for last pit if necessary.
        if (p[6].x > p[7].x) {
            p[7].x = p[6].x;
        }


        float sign = (pitinfo->side == TR_LFT) ? 1.0 : -1.0;
        p[0].y = pitentryoffset;
        p[8].y = pitexitoffset;
        for (i = 1; i < NPOINTS - 1; i++) {
            //fprintf(stderr, "B: %d - %.1f\n",i, p[i].x);
            p[i].y = fabs(pitinfo->driversPits->pos.toMiddle) - pitinfo->width;
            p[i].y *= sign;
        }
        //fflush(stderr);

        /* HACK */
        p[2].y = p[0].y;
        p[1].y = p[0].y;
        //p[3].y = fabs(pitinfo->driversPits->pos.toMiddle)*sign;
        p[5].y = fabs(pitinfo->driversPits->pos.toMiddle+1.0)*sign;
        spline = new Spline(NPOINTS, p);
    }
}


Pit::~Pit()
{
    if (mypit != NULL) {
        delete spline;
    }
}


// Transforms track coordinates to spline parameter coordinates.
float Pit::toSplineCoord(float x)
{
    x -= pitentry;
    while (x < 0.0f) {
        x += track->length;
    }
    return x;
}


// Computes offset to track middle for trajectory.
float Pit::getPitOffset(float offset, float fromstart)
{
  float pitoffset;

    if (mypit != NULL) {
        if (getInPit() || (getPitstop() && isBetween(fromstart))) {
            fromstart = toSplineCoord(fromstart);
            pitoffset = spline->evaluate(fromstart); 
            return pitoffset;
        }
    }
    return offset;
}


// Sets the pitstop flag if we are not in the pit range.
void Pit::setPitstop(bool pitstop)
{
    if (mypit == NULL) {
        return;
    }

    float fromstart = car->_distFromStartLine;

    if (!isBetween(fromstart)) {
        this->pitstop = pitstop;
    } else if (!pitstop) {
        this->pitstop = pitstop;
        pittimer = 0.0f;
    }
}


// Check if the argument fromstart is in the range of the pit.
bool Pit::isBetween(float fromstart)
{
    if (pitentry <= pitexit) {
        if (fromstart >= pitentry && fromstart <= pitexit) {
            return true;
        } else {
            return false;
        }
    } else {
        // Warning: TORCS reports sometimes negative values for "fromstart"!
        if (fromstart <= pitexit || fromstart >= pitentry) {
            return true;
        } else {
            return false;
        }
    }
}

// check if fromstart is in the actual pit area
bool Pit::isInPit(float fromstart)
{
    if (pitstopentry <= pitstopexit) {
        if (fromstart >= pitstopentry && fromstart <= pitstopexit) {
            return true;
        } else {
            return false;
        }
    } else {
        // Warning: TORCS reports sometimes negative values for "fromstart"!
        if (fromstart <= pitstopexit || fromstart >= pitstopentry) {
            return true;
        } else {
            return false;
        }
    }
}

float Pit::maxSpeed(float fromstart)
{
    if (!usepitmaxspeed)
        return 100000.0f;

    if (fromstart > pitmaxspeedoffset)
        return pitmaxspeed;
    return 100000.0f;
}


// Checks if we stay too long without getting captured by the pit.
// Distance is the distance to the pit along the track, when the pit is
// ahead it is > 0, if we overshoot the pit it is < 0.
bool Pit::isTimeout(float distance)
{
    if (car->_speed_x > 1.0f || distance > 3.0f || !getPitstop()) {
        pittimer = 0.0f;
        return false;
    } else {
        pittimer += (float) RCM_MAX_DT_ROBOTS;
        if (pittimer > 3.0f) {
            pittimer = 0.0f;
            return true;
        } else {
            return false;
        }
    }
}


// Update pit data and strategy.
void Pit::update()
{
    if (mypit != NULL) {
        if (isBetween(car->_distFromStartLine)) {
            if (getPitstop()) {
                setInPit(true);
            }
        } else {
            setInPit(false);
        }

        if (getPitstop()) {
            car->_raceCmd = RM_CMD_PIT_ASKED;
        }
    }
}


float Pit::getSpeedLimitBrake(float speedsqr)
{
    return (speedsqr-speedlimitsqr)/(pitspeedlimitsqr-speedlimitsqr);
}

