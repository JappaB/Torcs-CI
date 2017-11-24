////////////////////////////////////////////////////////////////////////////
//
// K1999.cpp
//
// car driver for TORCS
// created:    (c) Remi Coulom March 2000
//
// modified:    2008 Andrew Sumner novocas7rian@gmail.com
// modified:    2009 John Isham isham.john@gmail.com
// modified:       2014 Michel Luc mluc@cern91.net
// last update:    2015 Andrew Sumner novocas7rian@gmail.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>

#include "portability.h" 
#include "tgf.h" 
#include "track.h" 
#include "car.h"
#include "raceman.h" 
#include "robot.h" 
#include "robottools.h"

#include "raceline.h"
#include "driver.h"
#include "line.h"
#include "manual_override.h"
#include "xmldefs.h"
#include "vardef.h"

////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////

//
// These parameters are for the computation of the path
//
//static const double DivLength = 3.0;   // Length of path elements in meters

static const double SecurityR = 100.0; // Security radius
static double SideDistExt = 2.0; // Security distance wrt outside
static double SideDistInt = 1.0; // Security distance wrt inside          

#define myModuleName (char *)"usr_2016"

/////////////////////////////////////////////////////////////////////////////
// Some utility macros and functions
/////////////////////////////////////////////////////////////////////////////

static double Mag(double x, double y)
{
    return sqrt((x * x) + (y * y));
}

static double Min(double x1, double x2)
{
    if (x1 < x2)
        return x1;
    else
        return x2;
}

static double Max(double x1, double x2)
{
    if (x1 < x2)
        return x2;
    else
        return x1;
}

static double PointDist(vec2f *p1, vec2f *p2)
{
    double dx = (p1->x - p2->x);
    double dy = (p1->y - p2->y);
    return sqrt(dx*dx + dy*dy);
}

LRaceLine::LRaceLine(Driver *pdriver)
{
    driver = pdriver;
    iterations = 100;
    side_iterations = 20;
    fDirt = 0;
    Time = -1.0;
    rl_speed_mode = -1;
    overrideCollection = NULL;
    CornerSpeedSlow = CornerSpeedMid = CornerSpeed = turnSpeed = offlineTurnSpeed = offlineBrakeDist = 0.0;
    CornerSpeedFactor = 1.0;
    outsideCornerSpeed = insideCornerSpeed = 0.0;
    brakeDist = brakeDistMid = brakeDistSlow = 0.0;
    ExtMargin = IntMargin = speedAdjust = wheelbase = wheeltrack = curveFactor = curveAccel = curveBrake = bumpCaution = offlineBumpCaution = 0.0;
    AvoidExtMargin = AvoidIntMargin = 0.0;
    slopeFactor = offlineSlopeFactor = fulltankPercent = midtankPercent = maxfuel = edgeLineMargin = 0.0;
    Width = Length = TargetSpeed = 0.0;
    saveTrack = loadTrack = Divs = DivLength = Segs = racelineOverride = fDirt = This = Next = 0;
    steer_verbose = LineIndex = LineSave = 0;
    minTurnInverse = 0.0;
    lookAhead = lookAheadEmpty = 10.0;
    outsteerSpeedReducer = 1.0;
    steerSkidFactor = 0.9;
    steerSkidOfflineFactor = 0.5;
    errorCorrectionFactor = 1.0;
    Lfactor = last_lane = 0.0;
    outsideSteeringDampener = outsideSteeringDampenerOverlap = outsideSteeringDampenerAccel = 0.0;
    car = NULL;
    last_left_steer = last_rl_steer = last_right_steer = last_steer = last_last_steer = 0.0;
    last_steer_diff = 1000.0;
    useMergedSpeed = 0;
    last_target_raceline = -1;
    cornersteer = 0.0;

    tSegDist = (double *)malloc(MAXSEGMENTS * sizeof(double));
    tSegIndex = (int *)malloc(MAXSEGMENTS * sizeof(int));
    tSegment = (tTrackSeg **)malloc(MAXSEGMENTS * sizeof(tTrackSeg *));
    tElemLength = (double *)malloc(MAXSEGMENTS * sizeof(double));
    tDistance = (double *)malloc(MAXDIVS * sizeof(double));
    tMaxSpeed = (double *)malloc(MAXDIVS * sizeof(double));
    tzLeft = (double *)malloc(MAXDIVS * sizeof(double));
    tzRight = (double *)malloc(MAXDIVS * sizeof(double));
    tFriction = (double *)malloc(MAXDIVS * sizeof(double));

    int i;
    txRight = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        txRight[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(txRight[i], 0, MAXDIVS * sizeof(double));
    }
    tyRight = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tyRight[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tyRight[i], 0, MAXDIVS * sizeof(double));
    }
    txLeft = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        txLeft[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(txLeft[i], 0, MAXDIVS * sizeof(double));
    }
    tyLeft = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tyLeft[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tyLeft[i], 0, MAXDIVS * sizeof(double));
    }
    tx = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tx[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tx[i], 0, MAXDIVS * sizeof(double));
    }
    ty = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        ty[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(ty[i], 0, MAXDIVS * sizeof(double));
    }
    tz = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tz[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tz[i], 0, MAXDIVS * sizeof(double));
    }
    tzd = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tzd[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tzd[i], 0, MAXDIVS * sizeof(double));
    }
    tLane = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tLane[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tLane[i], 0, MAXDIVS * sizeof(double));
    }
    tRInverse = (double **)malloc(NUM_RACELINES * sizeof(double *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tRInverse[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tRInverse[i], 0, MAXDIVS * sizeof(double));
    }
    tSpeed = (double **)malloc(NUM_RACELINE_SPEEDS * sizeof(double *));
    for (i=0; i<NUM_RACELINE_SPEEDS; i++)
    {
        tSpeed[i] = (double *)malloc(MAXDIVS * sizeof(double));
        memset(tSpeed[i], 0, MAXDIVS * sizeof(double));
    }
    tDivSeg = (int **)malloc(NUM_RACELINES * sizeof(int *));
    for (i=0; i<NUM_RACELINES; i++)
    {
        tDivSeg[i] = (int *)malloc(MAXDIVS * sizeof(int));
        memset(tDivSeg[i], 0, MAXDIVS * sizeof(int));
    }
}

LRaceLine::~LRaceLine()
{
    free(tSegDist);
    free(tSegIndex);
    free(tSegment);
    free(tElemLength);
    free(tDistance);
    free(tMaxSpeed);
    free(tzLeft);
    free(tzRight);
    free(tFriction);

    int i;
    for (i=0; i<NUM_RACELINES; i++)
        free(tyRight[i]);
    free(tyRight);
    for (i=0; i<NUM_RACELINES; i++)
        free(txRight[i]);
    free(txRight);
    for (i=0; i<NUM_RACELINES; i++)
        free(tyLeft[i]);
    free(tyLeft);
    for (i=0; i<NUM_RACELINES; i++)
        free(txLeft[i]);
    free(txLeft);
    for (i=0; i<NUM_RACELINES; i++)
        free(tx[i]);
    free(tx);
    for (i=0; i<NUM_RACELINES; i++)
        free(ty[i]);
    free(ty);
    for (i=0; i<NUM_RACELINES; i++)
        free(tz[i]);
    free(tz);
    for (i=0; i<NUM_RACELINES; i++)
        free(tzd[i]);
    free(tzd);
    for (i=0; i<NUM_RACELINES; i++)
        free(tLane[i]);
    free(tLane);
    for (i=0; i<NUM_RACELINES; i++)
        free(tRInverse[i]);
    free(tRInverse);
    for (i=0; i<NUM_RACELINE_SPEEDS; i++)
        free(tSpeed[i]);
    free(tSpeed);
    for (i=0; i<NUM_RACELINES; i++)
        free(tDivSeg[i]);
    free(tDivSeg);
}

/////////////////////////////////////////////////////////////////////////////
// Update tx and ty arrays
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::UpdateTxTy(int i, int rl)
{
    tx[rl][i] = tLane[rl][i] * txRight[rl][i] + (1 - tLane[rl][i]) * txLeft[rl][i];
    ty[rl][i] = tLane[rl][i] * tyRight[rl][i] + (1 - tLane[rl][i]) * tyLeft[rl][i];
}

/////////////////////////////////////////////////////////////////////////////
// Set segment info
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::SetSegmentInfo(const tTrackSeg *pseg, double d, int i, double l)
{
    if (pseg)
    {
        tSegDist[pseg->id] = d;
        tSegIndex[pseg->id] = i;
        tElemLength[pseg->id] = l;
        if (pseg->id >= Segs)
            Segs = pseg->id + 1;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Split the track into small elements
// ??? constant width supposed
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::SplitTrack(tTrack *ptrack, int rl, bool preLoaded)
{
    // DivLength = GfParmGetNum( car->_carHandle, SECT_PRIVATE, "DivLength", (char *)NULL, 3 );
    DivLength = 3;
    Segs = 0;

    tTrackSeg *psegCurrent = ptrack->seg;

    double Distance = 0;
    double Angle = psegCurrent->angle[TR_ZS];
    double xlPos = psegCurrent->vertex[TR_SL].x;
    double ylPos = psegCurrent->vertex[TR_SL].y;
    double xrPos = psegCurrent->vertex[TR_SR].x;
    double yrPos = psegCurrent->vertex[TR_SR].y;
    double xPos = (xlPos + xrPos) / 2;
    double yPos = (ylPos + yrPos) / 2;

    int i = 0;

    std::fill_n(tFriction, MAXDIVS, psegCurrent->surface->kFriction);
    std::fill_n(txLeft[rl], MAXDIVS, 0.0);
    std::fill_n(txRight[rl], MAXDIVS, 0.0);
    std::fill_n(tyLeft[rl], MAXDIVS, 0.0);
    std::fill_n(tyRight[rl], MAXDIVS, 0.0);

    do
    {
        int Divisions = 1 + int((psegCurrent->length / DivLength) * (1.0+Lfactor));
        double Step = psegCurrent->length / Divisions;

        SetSegmentInfo(psegCurrent, Distance + Step, i, Step);
        for (int j = Divisions; --j >= 0;)
        {
            tDivSeg[rl][i] = psegCurrent->id;
            tSegment[psegCurrent->id] = psegCurrent;

            if (preLoaded)
            {
                Distance += Step;
                i++;
                continue;
            }

            double cosine = cos(Angle);
            double sine = sin(Angle);

            if (psegCurrent->type == TR_STR)
            {
                xPos += cosine * Step;
                yPos += sine * Step;
            }
            else
            {
                double r = psegCurrent->radius;
                double Theta = psegCurrent->arc / Divisions;
                double L = 2 * r * sin(Theta / 2);
                double x = L * cos(Theta / 2);
                double y;
                if (psegCurrent->type == TR_LFT)
                {
                    Angle += Theta;
                    y = L * sin(Theta / 2);
                }
                else
                {
                    Angle -= Theta;
                    y = -L * sin(Theta / 2);
                }
                xPos += x * cosine - y * sine;
                yPos += x * sine + y * cosine;
            }

            double dx = -psegCurrent->width * sin(Angle) / 2;
            double dy = psegCurrent->width * cos(Angle) / 2;
            txLeft[rl][i] = xPos + dx;
            tyLeft[rl][i] = yPos + dy;
            txRight[rl][i] = xPos - dx;
            tyRight[rl][i] = yPos - dy;
            if (rl == LINE_LEFT)
            {
                txRight[rl][i] = txLeft[rl][i] + (txRight[rl][i] - txLeft[rl][i]) * edgeLineMargin;
                tyRight[rl][i] = tyLeft[rl][i] + (tyRight[rl][i] - tyLeft[rl][i]) * edgeLineMargin;
            }
            else if (rl == LINE_RIGHT)
            {
                txLeft[rl][i] = txRight[rl][i] - (txRight[rl][i] - txLeft[rl][i]) * edgeLineMargin;
                tyLeft[rl][i] = tyRight[rl][i] - (tyRight[rl][i] - tyLeft[rl][i]) * edgeLineMargin;
            }
            else if (rl == LINE_MID)
            {
                double xright = txLeft[rl][i] + (txRight[rl][i] - txLeft[rl][i]) * 0.50;
                double yright = tyLeft[rl][i] + (tyRight[rl][i] - tyLeft[rl][i]) * 0.50;
                txLeft[rl][i] = txRight[rl][i] - (txRight[rl][i] - txLeft[rl][i]) * 0.50;
                tyLeft[rl][i] = tyRight[rl][i] - (tyRight[rl][i] - tyLeft[rl][i]) * 0.50;
                txRight[rl][i] = xright;
                tyRight[rl][i] = yright;
            }
            tLane[rl][i] = 0.5;
            tFriction[i] = psegCurrent->surface->kFriction;
            if (tFriction[i] < 1) // ??? ugly trick for dirt
            {
                // fprintf("%s: dirt hack segment:%d\n",__FUNCTION__,i); 

                //tFriction[i] *= 0.90;
                fDirt = 1;
                SideDistInt = -1.5;
                SideDistExt = 0.0;
            }
            UpdateTxTy(i, rl);

            Distance += Step;
            i++;
        }

        psegCurrent = psegCurrent->next;
    } while (psegCurrent != ptrack->seg);

    if (!preLoaded)
        Divs = i - 1;

    Width = psegCurrent->width;
    Length = Distance;
}

/////////////////////////////////////////////////////////////////////////////
// Compute the inverse of the radius
/////////////////////////////////////////////////////////////////////////////
double LRaceLine::GetRInverse(int prev, double x, double y, int next, int rl)
{
    double x1 = tx[rl][next] - x;
    double y1 = ty[rl][next] - y;
    double x2 = tx[rl][prev] - x;
    double y2 = ty[rl][prev] - y;
    double x3 = tx[rl][next] - tx[rl][prev];
    double y3 = ty[rl][next] - ty[rl][prev];

    double det = x1 * y2 - x2 * y1;
    double n1 = x1 * x1 + y1 * y1;
    double n2 = x2 * x2 + y2 * y2;
    double n3 = x3 * x3 + y3 * y3;
    double nnn = sqrt(n1 * n2 * n3);

    return 2 * det / nnn;
}

/////////////////////////////////////////////////////////////////////////////
// Change lane value to reach a given radius
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::AdjustRadius(int prev, int i, int next, double TargetRInverse, int rl, double Security)
{
    double OldLane = tLane[rl][i];

    //
    // Start by aligning points for a reasonable initial lane
    //
    tLane[rl][i] = (-(ty[rl][next] - ty[rl][prev]) * (txLeft[rl][i] - tx[rl][prev]) +
        (tx[rl][next] - tx[rl][prev]) * (tyLeft[rl][i] - ty[rl][prev])) /
        ((ty[rl][next] - ty[rl][prev]) * (txRight[rl][i] - txLeft[rl][i]) -
        (tx[rl][next] - tx[rl][prev]) * (tyRight[rl][i] - tyLeft[rl][i]));
    if (tLane[rl][i] < -0.2)
        tLane[rl][i] = -0.2;
    else if (tLane[rl][i] > 1.2)
        tLane[rl][i] = 1.2;
    UpdateTxTy(i, rl);

    //
    // Newton-like resolution method
    //
    const double dLane = 0.0001;

    double dx = dLane * (txRight[rl][i] - txLeft[rl][i]);
    double dy = dLane * (tyRight[rl][i] - tyLeft[rl][i]);

    double dRInverse = GetRInverse(prev, tx[rl][i] + dx, ty[rl][i] + dy, next, rl);

    if (dRInverse > 0.000000001)
    {
        tLane[rl][i] += (dLane / dRInverse) * TargetRInverse;

        /****************** Get Extern/Intern Margin **********************/
#if 0
        double ExtMarginAddition = GfParmGetNum( car->_carHandle, SECT_PRIVATE, "extmargin", (char *)NULL, 0.0 );
        double IntMarginAddition = GfParmGetNum( car->_carHandle, SECT_PRIVATE, "intmargin", (char *)NULL, 0.0 );

        double ExtLane = (0.5 + ExtMarginAddition + Security) / Width;
        double IntLane = (0.5 + IntMarginAddition + Security) / Width;

#else
        double ExtLane = (getExtMargin(rl, i, TargetRInverse) + Security) / Width;
        double IntLane = (getIntMargin(rl, i, TargetRInverse) + Security) / Width;
#endif
        if (ExtLane > 0.5 && rl >= LINE_RL)
            ExtLane = 0.5;
        if (IntLane > 0.5 && rl >= LINE_RL)
            IntLane = 0.5;

        if (TargetRInverse >= 0.0)
        {
            if (tLane[rl][i] < IntLane)
                tLane[rl][i] = IntLane;
            if (1 - tLane[rl][i] < ExtLane)
            {
                if (1 - OldLane < ExtLane)
                    tLane[rl][i] = Min(OldLane, tLane[rl][i]);
                else
                    tLane[rl][i] = 1 - ExtLane;
            }
        }
        else
        {
            if (tLane[rl][i] < ExtLane)
            {
                if (OldLane < ExtLane)
                    tLane[rl][i] = Max(OldLane, tLane[rl][i]);
                else
                    tLane[rl][i] = ExtLane;
            }
            if (1 - tLane[rl][i] < IntLane)
                tLane[rl][i] = 1 - IntLane;
        }

        if (rl < LINE_RL)
        {
            tLane[rl][i] += TargetRInverse / 15;
        }
    }

    UpdateTxTy(i, rl);
}

/////////////////////////////////////////////////////////////////////////////
// Smooth path
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::Smooth(int Step, int rl)
{
    int prev = ((Divs - Step) / Step) * Step;
    int prevprev = prev - Step;
    int next = Step;
    int nextnext = next + Step;

    for (int i = 0; i <= Divs - Step; i += Step)
    {
        double ri0 = GetRInverse(prevprev, tx[rl][prev], ty[rl][prev], i, rl);
        double ri1 = GetRInverse(i, tx[rl][next], ty[rl][next], nextnext, rl);
        double lPrev = Mag(tx[rl][i] - tx[rl][prev], ty[rl][i] - ty[rl][prev]);
        double lNext = Mag(tx[rl][i] - tx[rl][next], ty[rl][i] - ty[rl][next]);

        double TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);

        double Security = lPrev * lNext / (8 * SecurityR);

        if (rl >= LINE_RL)
        {
            if (ri0 * ri1 > 0)
            {
                double ac1 = fabs(ri0);
                double ac2 = fabs(ri1);
                {
                    double cf = getCurveFactor(i, (ac1 < ac2));
                    if (ac1 < ac2) // curve is increasing
                    {
                        ri0 += cf * (ri1 - ri0);
                    }
                    else if (ac2 < ac1) // curve is decreasing
                    {
                        ri1 += cf * (ri0 - ri1);
                    }
                }

                TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);
            }
        }

        AdjustRadius(prev, i, next, TargetRInverse, rl, Security);

        prevprev = prev;
        prev = i;
        next = nextnext;
        nextnext = next + Step;
        if (nextnext > Divs - Step)
            nextnext = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Interpolate between two control points
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::StepInterpolate(int iMin, int iMax, int Step, int rl)
{
    int next = (iMax + Step) % Divs;
    if (next > Divs - Step)
        next = 0;

    int prev = (((Divs + iMin - Step) % Divs) / Step) * Step;
    if (prev > Divs - Step)
        prev -= Step;

    double ir0 = GetRInverse(prev, tx[rl][iMin], ty[rl][iMin], iMax % Divs, rl);
    double ir1 = GetRInverse(iMin, tx[rl][iMax % Divs], ty[rl][iMax % Divs], next, rl);
    for (int k = iMax; --k > iMin;)
    {
        double x = double(k - iMin) / double(iMax - iMin);
        double TargetRInverse = x * ir1 + (1 - x) * ir0;
        AdjustRadius(iMin, k, iMax % Divs, TargetRInverse, rl);
    }
}

/////////////////////////////////////////////////////////////////////////////
// Calls to StepInterpolate for the full path
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::Interpolate(int Step, int rl)
{
    if (Step > 1)
    {
        int i;
        for (i = Step; i <= Divs - Step; i += Step)
            StepInterpolate(i - Step, i, Step, rl);
        StepInterpolate(i - Step, Divs, Step, rl);
    }
}


/*===========================================================*/
/*                  SET DATA                                 */
/*===========================================================*/

void LRaceLine::setRwData(tTrack* t, void **carParmHandle, tSituation *s)
{
    return;
}

/**********************************************************/
double LRaceLine::getMaxSpeed(int Div)
{
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_MAX_SPEED);
        if (labelOverride)
        {
            double speed_override;
            if (labelOverride->getOverrideValue(Div, &speed_override))
                return speed_override;
        }
    }

    return 10000.0;
}

double LRaceLine::getCurveFactor(int Div, bool isBraking)
{
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_RACELINECURVE);
        if (labelOverride)
        {
            double curve_override;
            if (labelOverride->getOverrideValue(Div, &curve_override))
                return curve_override;
        }
    }

    if (isBraking)
        return curveBrake;

    return curveAccel;
}

double LRaceLine::getBumpCaution(int Div, int rl)
{
    double bc_override;
    LManualOverride *labelOverride;

    if (rl >= LINE_RL)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }

        return bumpCaution;
    }
    else if (rl == LINE_LEFT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }
    }
    else if (rl == LINE_RIGHT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_BUMP_CAUTION);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &bc_override))
                    return bc_override;
            }
        }
    }

    return offlineBumpCaution;
}

double LRaceLine::getCornerSpeed(int Div, int rl)
{
    double cornerspeed_override;
    LManualOverride *labelOverride;

    if (rl == LINE_RL)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    return cornerspeed_override * CornerSpeedFactor;
            }
        }

        return CornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_RL_MID)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
        }

        if (CornerSpeedMid > 1.0)
            return CornerSpeedMid * CornerSpeedFactor;
        return CornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_RL_SLOW && hasSlow)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED_SLOW);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_CORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
        }

        if (CornerSpeedSlow > 1.0)
            return CornerSpeedSlow * CornerSpeedFactor;
        if (CornerSpeedMid > 1.0)
            return CornerSpeedMid * CornerSpeedFactor;
        return CornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_LEFT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFTCORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
        }

        if (tRInverse[LINE_RL][Div] < -0.001)
            return outsideCornerSpeed * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] > 0.001)
            return insideCornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_RIGHT)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHTCORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * CornerSpeedFactor;
            }
        }

        if (tRInverse[LINE_RL][Div] > 0.001)
            return outsideCornerSpeed * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] < -0.001)
            return insideCornerSpeed * CornerSpeedFactor;
    }
    else if (rl == LINE_LEFT_OUTSTEER)
    {
        double factor = 1.0;
        if (tRInverse[LINE_RL][Div] < -0.01)
            factor = outsteerSpeedReducer;
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFTCORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * factor * CornerSpeedFactor;
            }
        }

        if (tRInverse[LINE_RL][Div] < -0.001)
            return outsideCornerSpeed * factor * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] > 0.001)
            return insideCornerSpeed * factor * CornerSpeedFactor;
    }
    else if (rl == LINE_RIGHT_OUTSTEER)
    {
        double factor = 1.0;
        if (tRInverse[LINE_RL][Div] > 0.01)
            factor = outsteerSpeedReducer;
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHTCORNERSPEED);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &cornerspeed_override))
                    if (cornerspeed_override > 1.0)
                        return cornerspeed_override * factor * CornerSpeedFactor;
            }
        }

        if (tRInverse[LINE_RL][Div] > 0.001)
            return outsideCornerSpeed * factor * CornerSpeedFactor;
        else if (tRInverse[LINE_RL][Div] < -0.001)
            return insideCornerSpeed * factor * CornerSpeedFactor;
    }

    return offlineTurnSpeed * CornerSpeedFactor;
}

/**********************************************************/
double LRaceLine::getBrakeDist(int Div, int rl)
{
    double brakedelay_override;
    LManualOverride *labelOverride;

    if (rl == LINE_RL_SLOW && hasSlow)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_SLOW);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
        }

        if (brakeDistSlow > 1.0)
            return brakeDistSlow;
        if (brakeDistMid > 1.0)
            return brakeDistMid;
        return brakeDist;
    }
    else if (rl == LINE_RL_MID && hasMid)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY_MID);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
        }

        if (brakeDistMid > 1.0)
            return brakeDistMid;
        return brakeDist;
    }
    else if (rl == LINE_RL)
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_BRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
        }

        return brakeDist;
    }
    else 
    {
        if (overrideCollection)
        {
            labelOverride = overrideCollection->getOverrideForLabel(PRV_AVOIDBRAKEDELAY);
            if (labelOverride)
            {
                if (labelOverride->getOverrideValue(Div, &brakedelay_override))
                    if (brakedelay_override > 1.0)
                        return brakedelay_override;
            }
        }
    }

    return offlineBrakeDist;
}

/**********************************************************/
double LRaceLine::getIntMargin(int raceline, int Div, double rInverse)
{
    double extramargin = 0.0;
    double intmargin = IntMargin;
    LManualOverride *labelOverride;

    if (raceline < LINE_RL)
        intmargin += AvoidIntMargin;

    if (rInverse <= 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
            }
            else
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
            }
        }
    }
    else //if (rInverse > 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
            }
            else
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return intmargin + extramargin;
                }
            }
        }
    }

    return intmargin;
}

/**********************************************************/
double LRaceLine::getExtMargin(int raceline, int Div, double rInverse)
{
    double extramargin = 0.0;
    double extmargin = ExtMargin;
    LManualOverride *labelOverride;

    if (raceline < LINE_RL)
        extmargin += AvoidExtMargin;

#if 0
    tTrackSeg *pseg = car->_trkPos.seg;
    const double    MIN_MU = pseg->surface->kFriction * 0.8;
    const double    MAX_ROUGH = MX(0.005, pseg->surface->kRoughness * 1.2);
    const double    MAX_RESIST = MX(0.04, pseg->surface->kRollRes * 1.2);
    double right_margin = 0.0, left_margin = 0.0;

    for( int s = 0; s < 2; s++ )
    {
        tTrackSeg*  pSide = pseg->side[s];
        if( pSide == 0 )
            continue;

        double  extraW = 0;

        bool  done = false;
        while( !done && pSide )
        {
            double  w = pSide->startWidth +
                        (pSide->endWidth - pSide->startWidth) * t;
//          w = MX(0, w - 0.5);

//          w = MN(w, 1.0);
            if( pSide->style == TR_CURB )
            {
                // always keep 1 wheel on main track.
                w = MN(w, 1.5);
                done = true;

                if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
                   s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
                    pSide->surface->kFriction  < pseg->surface->kFriction )
                    // keep a wheel on the good stuff.
                    w = 0;//MN(w, 1.5);

                // don't go too far up raised curbs (max 2cm).
                if( pSide->height > 0 )
                    w = MN(w, 0.6);
//                  w = MN(0.05 * pSide->width / pSide->height, 1.5);

//              if( pSide->surface->kFriction  < MIN_MU )
//                  w = 0;
          }
          else if( pSide->style == TR_PLAN )
          {
              if( /* FIXME */ (inPit && pitSide == s)               ||
                  (pSide->raceInfo & (TR_SPEEDLIMIT | TR_PITLANE)) )
              {
                  w = 0;
                  done = true;
              }

              if( s == m_sideMod.side &&
                  i >= m_sideMod.start &&
                  i <= m_sideMod.end )
              {
                  if( w > 0.5 )
                    { w = 0.5; done = true; }
              }
              else
                  if( pSide->surface->kFriction  < MIN_MU    ||
                      pSide->surface->kRoughness > MAX_ROUGH  ||
                      pSide->surface->kRollRes   > MAX_RESIST  ||
                      fabs(pSide->Kzw - SLOPE) > 0.005 )
                  {
                      bool  inner = 
                                    s == TR_SIDE_LFT && pseg->type == TR_LFT ||
                                    s == TR_SIDE_RGT && pseg->type == TR_RGT;
                      w = 0;//inner ? MN(w, 0.5) : 0;
                      done = true;
                  }

                  if( (s == TR_SIDE_LFT && pseg->type == TR_RGT ||
                       s == TR_SIDE_RGT && pseg->type == TR_LFT) &&
                        pSide->surface->kFriction  < pseg->surface->kFriction )
                  {
                      // keep a wheel on the good stuff.
                      w = 0;//MN(w, 1.5);
                      done = true;
                  }
                  if (done && pSide->side[s])
                  {
                    if (pSide->side[s]->style == TR_WALL || pSide->side[s]->style == TR_FENCE)
                    {
                      // keep off the walls!
                      w -= 0.5;
                    }
                  }
                  else
                    w -= 0.5;
          }
          else
          {
            // wall of some sort.
            w = -1;
            /*
            w = pSide->style == TR_WALL ? -0.5 :
//              pSide->style == TR_FENCE ? -0.1 : 0;
              0;
              */
            done = true;
          }

          if (s == TR_SIDE_LFT)
            left_margin = w;
          else if (s == TR_SIDE_RGT)
            right_margin = w;

//          if( pSide->style != TR_PLAN || w <= 0 )
//            done = true;

          pSide = pSide->side[s];
        }
    }
#endif

    if (rInverse >= 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
            }
            else
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RIGHT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
            }
        }
    }
    else// if (rInverse < 0.0)
    {
        if (overrideCollection)
        {
            if (raceline >= LINE_RL)
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
            }
            else
            {
                labelOverride = overrideCollection->getOverrideForLabel(PRV_LEFT_MARGIN);
                if (labelOverride)
                {
                    if (labelOverride->getOverrideValue(Div, &extramargin))
                        return extmargin + extramargin;
                }
            }
        }
    }

    return extmargin;
}

/////////////////////////////////////////////////////////////////////////////
// Find changes in line height
/////////////////////////////////////////////////////////////////////////////
void LRaceLine::CalcZCurvature(int rl)
{
    int i;

    for (i = 0; i < Divs; i++)
    {
        tTrackSeg *seg = tSegment[tDivSeg[rl][i]];

        tz[rl][i] = RtTrackHeightG(seg, (tdble)tx[rl][i], (tdble)ty[rl][i]);

        int next = (i + 1) % Divs;
        int prev = (i - 1 + Divs) % Divs;
        double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, rl);
        tRInverse[rl][i] = rInverse;
    }

    for (i = 0; i < Divs; i++)
    {
        int j = ((i - 1) + Divs) % Divs;

        vec2f pi, pj;
        pi.x = (tdble)tx[rl][i]; pi.y = (tdble)ty[rl][i];
        pj.x = (tdble)tx[rl][j]; pj.y = (tdble)ty[rl][j];

        tzd[rl][i] = (tz[rl][i] - tz[rl][j]) / PointDist(&pi, &pj);
    }

    for (i = 0; i < Divs; i++)
    {
        double zd = 0.0;
        for (int nx = 0; nx < 4; nx++)
        {
            int nex = (i + nx) % Divs;
            if (tzd[rl][nex] < 0.0)
                zd += tzd[rl][nex] * 2;
            else
                zd += tzd[rl][nex] * 0.2;
        }

        double camber = SegCamber(rl, i) - 0.002;
        if (camber < 0.0)
        {
            camber *= 5;
            if (rl == LINE_MID)
                camber *= 2;
        }
        double slope = camber + zd / 3 * (rl >= LINE_RL ? slopeFactor : offlineSlopeFactor);
        if (rl < LINE_RL)
        {
            if (slope < 0.0)
                slope *= 1.4;
            else
                slope *= 0.2;
        }

        tFriction[i] *= 1.0 + MAX(-0.4, slope);

        /*
        if (slope < 0.0)
            tBrakeFriction[rl][i] = 1.0 + MAX(-0.4, slope / 10);
        else
            tBrakeFriction[rl][i] = 1.0 + slope / 40;
            */
    }
}

double LRaceLine::SegCamber(int rl, int div)
{
    tTrackSeg *seg = tSegment[tDivSeg[rl][div]];
    double camber = (((seg->vertex[TR_SR].z - seg->vertex[TR_SL].z) / 2) + ((seg->vertex[TR_ER].z - seg->vertex[TR_EL].z) / 2)) / seg->width;
    double camber1 = ((seg->vertex[TR_SR].z - seg->vertex[TR_SL].z)) / seg->width;
    double camber2 = ((seg->vertex[TR_ER].z - seg->vertex[TR_EL].z)) / seg->width;

    if (tRInverse[rl][div] < 0.0)
    {
        camber = -camber;
        camber2 = -camber2;
        camber1 = -camber1;
    }
    if (camber2 < camber1)
        camber = camber2;

    return camber;
}

double LRaceLine::SegCamberForNext()
{
    return SegCamber(LINE_RL, Next);
}


/*===========================================================*/
/*                INIT TRACK                                 */
/*===========================================================*/
void LRaceLine::InitTrack(tTrack* track, tSituation *p)
{
    iterations = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ITERATIONS, (char *)NULL, 100.0);
    side_iterations = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SIDE_ITERATIONS, (char *)NULL, iterations/5);
    offlineTurnSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OFFLINE_TURNSPEED, (char *)NULL, 14.0);
    outsideCornerSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDECORNERSPEED, (char *)NULL, (tdble)offlineTurnSpeed);
    insideCornerSpeed = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_INSIDECORNERSPEED, (char *)NULL, (tdble)offlineTurnSpeed);
    CornerSpeedFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_CORNERSPEED_FACTOR, (char *)NULL, (tdble)1.0);
    minTurnInverse = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_MINTURNINVERSE, (char *)NULL, (tdble) 0.0028);
    speedAdjust = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "speedadjust", (char *)NULL, 2.0);
    brakeDist = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY, (char *)NULL, 15.0);
    brakeDistMid = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY_MID, (char *)NULL, 15.0);
    brakeDistSlow = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKEDELAY_SLOW, (char *)NULL, 15.0);
    offlineBrakeDist = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OFFLINE_BRAKEDELAY, (char *)NULL, 15.0);
    bumpCaution = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "bumpcaution", (char *)NULL, 0.2f);
    offlineBumpCaution = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "offlinebumpcaution", (char *)NULL, 0.2f);
    slopeFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "slopefactor", (char *)NULL, 5.0);
    offlineSlopeFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, "offlineslopefactor", (char *)NULL, 5.0);
    curveFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_CURVE_FACTOR, (char *)NULL, (tdble) 0.13);
    curveAccel = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ACCEL_CURVE, (char *)NULL, (tdble)curveFactor);
    curveBrake = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_BRAKE_CURVE, (char *)NULL, (tdble)curveFactor);
    racelineOverride = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, "raceline_override", (char *)NULL, -1.0);
    racelineDebug = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, "raceline debug", (char *)NULL, 0.0);
    fulltankPercent = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_FULLTANK_PERCENT, (char *)NULL, 2.0);
    midtankPercent = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_MIDTANK_PERCENT, (char *)NULL, 2.0);
    maxfuel = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_TANK, (char*)NULL, 100.0f) - 15.0;
    edgeLineMargin = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_EDGE_LINE_MARGIN, (char*)NULL, 0.3f);
    lookAhead = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD, (char*)NULL, 10.0f);
    lookAheadEmpty = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOOKAHEAD_EMPTY, (char*)NULL, (tdble)lookAhead);
    outsteerSpeedReducer = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSTEER_REDUCER, (char*)NULL, 1.0f);
    steerSkidFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_STEER_SKID, (char*)NULL, 0.9f);
    steerSkidOfflineFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_STEER_SKID_OFFLINE, (char*)NULL, 0.5f);
    errorCorrectionFactor = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_ERROR_CORRECTION, (char*)NULL, 1.0f);
    outsideSteeringDampener = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER, (char*)NULL, -70.0f);
    outsideSteeringDampenerOverlap = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER_O, (char*)NULL, -60.0f);
    outsideSteeringDampenerAccel = GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_OUTSIDE_DAMPENER_A, (char*)NULL, (tdble)outsideSteeringDampener*1.1);
    loadTrack = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_LOAD_TRACK, (char*)NULL, 0.0f);
    saveTrack = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_SAVE_TRACK, (char*)NULL, 0.0f);
    useMergedSpeed = (int)GfParmGetNum(car->_carHandle, SECT_PRIVATE, PRV_USE_MERGED_SPEED, (char*)NULL, 0.0f);
    ExtMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_EXTMARGIN, (char *)NULL, (tdble) 1.4 );
    IntMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_INTMARGIN, (char *)NULL, (tdble) 0.3 );
    AvoidExtMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_AVOID_EXTMARGIN, (char *)NULL, (tdble) 1.4 );
    AvoidIntMargin = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_AVOID_INTMARGIN, (char *)NULL, (tdble) 1.0 );
    Lfactor = (tdble) GfParmGetNum( car->_carHandle, SECT_PRIVATE, "lfactor", (char *)NULL, (tdble) 0.0 );
    cornersteer = GfParmGetNum( car->_carHandle, SECT_PRIVATE, PRV_CORNERSTEER, (char *)NULL, (tdble) 0.0 );

    // split track
    for (int i=0; i<NUM_RACELINES; i++)
    {
        for (int j=0; j<MAXDIVS; j++)
        {
            tRInverse[i][j] = 0;
            tLane[i][j] = 0;
        }
    }

    bool trackPreLoaded = false;
    if (loadTrack)
        trackPreLoaded = LoadTrack(track, p);

    //hasSlow = false, hasMid = false;
    hasSlow = true, hasMid = true;
    /*
    if (overrideCollection)
    {
        char *slowList[] = { PRV_CORNERSPEED_SLOW, PRV_BRAKEDELAY_SLOW, PRV_LEFT_MARGIN_SLOW, PRV_RIGHT_MARGIN_SLOW, NULL };
        int i = 0;
        while (slowList[i])
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(slowList[i]);
            if (labelOverride)
            {
                if (labelOverride->valueCount() > 0)
                {
                    hasSlow = true;
                    break;
                }
            }
            i++;
        }

        char *midList[] = { PRV_CORNERSPEED_MID, PRV_BRAKEDELAY_MID, PRV_LEFT_MARGIN_MID, PRV_RIGHT_MARGIN_MID, NULL };
        i = 0;
        while (midList[i])
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(midList[i]);
            if (labelOverride)
            {
                if (labelOverride->valueCount() > 0)
                {
                    hasMid = true;
                    break;
                }
            }
            i++;
        }
    }
    */

    int i;
    for (int rl = LINE_LEFT; rl <= LINE_RL; rl++)
    {
        //if (rl == LINE_MID) continue;
        //if (rl == LINE_RL_SLOW && !hasSlow) continue;
        //if (rl == LINE_RL_MID && !hasMid) continue;
        /*Split the track into small elements*/
        SplitTrack(track, rl, trackPreLoaded);

        if (!trackPreLoaded)
        {
            // Smoothing loop
            int Iter = (rl < LINE_RL ? side_iterations : iterations);
            for (int Step = 128; (Step /= 2) > 0;)
            {
                for (i = Iter * int(sqrt((float)Step)); --i >= 0;)
                    Smooth(Step, rl);
                Interpolate(Step, rl);
            }

            CalcZCurvature(rl);


            // Compute curvature and speed along the path
            for (i = Divs; --i >= 0;)
            {
                ComputeRacelineSpeed(i, rl, tSpeed, rl);
            }
            //
            // Anticipate braking
            //
            for (int j = 32; --j >= 0;)
            {
                for (i = Divs; --i >= 0;)
                {
                    ComputeRacelineBraking(i, rl, tSpeed, rl);
                }
            }

            if (rl == LINE_RL)
            {
                for (int spdrl = LINE_RL_MID; spdrl <= LINE_RL_SLOW; spdrl++)
                {
                    for (i = Divs; --i >= 0;)
                    {
                        ComputeRacelineSpeed(i, rl, tSpeed, spdrl);
                    }

                    for (int j = 32; --j >= 0;)
                    {
                        for (i = Divs; --i >= 0;)
                        {
                            ComputeRacelineBraking(i, rl, tSpeed, spdrl);
                        }
                    }
                }
            }

            if (rl == LINE_LEFT)
            {
                for (i = Divs; --i >= 0;)
                {
                    ComputeRacelineSpeed(i, LINE_LEFT, tSpeed, LINE_LEFT_OUTSTEER);
                }

                for (int j = 32; --j >= 0;)
                {
                    for (i = Divs; --i >= 0;)
                    {
                        ComputeRacelineBraking(i, LINE_LEFT, tSpeed, LINE_LEFT_OUTSTEER);
                    }
                }
            }

            if (rl == LINE_RIGHT)
            {
                for (i = Divs; --i >= 0;)
                {
                    ComputeRacelineSpeed(i, LINE_RIGHT, tSpeed, LINE_RIGHT_OUTSTEER);
                }

                for (int j = 32; --j >= 0;)
                {
                    for (i = Divs; --i >= 0;)
                    {
                        ComputeRacelineBraking(i, LINE_RIGHT, tSpeed, LINE_RIGHT_OUTSTEER);
                    }
                }
            }
        }
    }

    if (saveTrack)
        SaveTrack(track, p);
}

double LRaceLine::getMinTurnInverse(int raceline)
{
    {
        return minTurnInverse;
    }

    double empty = 15.0f;
    double fuel_gauge = 1.0 - (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - 15.0f) / (maxfuel-15.0f)));
    double slowTurnInverse = 1.0 - (fuel_gauge*0.05);
    return minTurnInverse * slowTurnInverse;
}

void LRaceLine::ComputeRacelineBraking(int i, int rl, double **tSpeed, int speedrl)
{
    //double TireAccel = turnSpeed * tFriction[i];
    double TireAccel = getCornerSpeed(i, speedrl) * tFriction[i];
    int prev = (i - 1 + Divs) % Divs;

    double dx = tx[rl][i] - tx[rl][prev];
    double dy = ty[rl][i] - ty[rl][prev];
    double dist = Mag(dx, dy);

    double Speed = (tSpeed[speedrl][i] + tSpeed[speedrl][prev]) / 2;

    double LatA = tSpeed[speedrl][i] * tSpeed[speedrl][i] *
        (fabs(tRInverse[rl][prev]) + fabs(tRInverse[rl][i])) / 2;

#if 1
    double TanA = TireAccel * TireAccel - LatA * LatA;
    if (TanA < 0.0)
        TanA = 0.0;
    //TanA = sqrt(TanA) + minTurnInverse * Speed * Speed;
    double brakedelay = getBrakeDist(i, speedrl) + (rl != LINE_RL ? speedAdjust : 0.0);
    if (TanA > brakedelay)
        TanA = brakedelay;
#else
    double minturninverse = getMinTurnInverse(rl);
    double TanA = TireAccel * TireAccel + minturninverse * Speed * Speed - LatA * LatA;
    //double brakedelay = brakeDist + (rl == LINE_MID ? speedAdjust : 0.0);
    double brakedelay = getBrakeDist(i, rl) + (rl != LINE_RL ? speedAdjust : 0.0);
    if (TanA > brakedelay * tFriction[i])
        TanA = brakedelay * tFriction[i];
#endif

    double time = dist / Speed;
    double MaxSpeed = tSpeed[speedrl][i] + TanA * time;
    tSpeed[speedrl][prev] = Min(MaxSpeed, tMaxSpeed[prev]);
}

double LRaceLine::getFriction(int i)
{
    double empty = 15.0f;
    double fuel_status = 1.0 - (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - 15.0f) / (maxfuel-15.0f)));
    return tFriction[i] * (1.0 - fuel_status*0.1);
}

void LRaceLine::ComputeRacelineSpeed(int i, int rl, double **tSpeed, int speedrl)
{
    vec2f tmp;
    //double TireAccel = turnSpeed * tFriction[i];
    double TireAccel = getCornerSpeed(i, speedrl) * tFriction[i];
    if (rl < LINE_RL)
        TireAccel += speedAdjust;
    int next = (i + 1) % Divs;
    int prev = (i - 1 + Divs) % Divs;

    double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, rl);
    double rI = fabs(rInverse);
    /* rinverse from clothoid curvature??  */
    if (rl == speedrl)
        tRInverse[rl][i] = rInverse;

    double MaxSpeed;

    double minturninverse = getMinTurnInverse(rl);
    if (fabs(rInverse) > minturninverse * 1.01)
        MaxSpeed = sqrt(TireAccel / (fabs(rInverse) - minturninverse));
    else
        MaxSpeed = 10000;

    //if (fabs(rInverse) > 0.0087 )
    //printf("rInv:%f MaxSpd:%f\n", rInverse,  MaxSpeed);

    double bc = getBumpCaution(i, speedrl);

    if (bc > 0.0)
    {
        // look for a sudden downturn in approaching track segments
        double prevzd = 0.0, nextzd = 0.0;
        int range = int(MAX(40.0, MIN(100.0, MaxSpeed)) / 10.0);

        bc += rI * 80;

        for (int n = 1; n < range; n++)
        {
            int x = ((i - n) + Divs) % Divs;
            prevzd += tzd[rl][x] / MAX(1.0, double(n) / 2);
        }

        for (int n = 0; n < range; n++)
        {
            int x = ((i + n) + Divs) % Divs;
            nextzd += tzd[rl][x] / MAX(1.0, double(n + 1) / 2);
        }

        double diff = (prevzd - nextzd) * 2.2;

        if (diff > 0.10)
        {
            diff -= 0.10;
            double safespeed = MAX(15.0, 100.0 - (diff*diff) * 400 * bc);
            if (safespeed < MIN(70.0, MaxSpeed))
            {
                // do a couple of divs before this one to be sure we actually reduce speed
                for (int n = 0; n < 4; n++)
                {
                    int f = ((i - n) + Divs) % Divs;
                    tSpeed[speedrl][f] = safespeed;
                }

                for (int n = 0; n < 4; n++)
                {
                    int f = (i + n) % Divs;
                    tSpeed[speedrl][f] = safespeed;
                }

                MaxSpeed = MIN(MaxSpeed, safespeed);
            }
        }
    }

    MaxSpeed = MIN(getMaxSpeed(i), MaxSpeed);

    tSpeed[speedrl][i] = tMaxSpeed[i] = MaxSpeed;

    /*
    if (rl >= LINE_RL)
    {
        int turnSpecs;
        for (turnSpecs = 0; turnSpecs < MAX_TDATA; turnSpecs++)
        {
            if ((-1 != MspeedSegStart[turnSpecs]) && (i >= MspeedSegStart[turnSpecs]) && (i <= MspeedSegEnd[turnSpecs]))
            {
                if (segMaxSpeed[turnSpecs] < tSpeed[speedrl][i])
                {
                    if (line_verbose)
                        GfOut("OVERRIDE max speed in div %d old:%f new:%f\n", i, tSpeed[speedrl][i], segMaxSpeed[turnSpecs]);

                    tSpeed[speedrl][i] = tMaxSpeed[i] = segMaxSpeed[turnSpecs];
                }
                else
                {
                    if (line_verbose)
                        GfOut("NO OVERRIDE  max speed in div %d old:%f new:%f\n", i, tSpeed[speedrl][i], segMaxSpeed[turnSpecs]);
                }
            }
        }
    }
    */
}

////////////////////////////////////////////////////////////////////////////
// New race
////////////////////////////////////////////////////////////////////////////
void LRaceLine::NewRace(tCarElt* newcar, tSituation *s)
{
    car = newcar;
    last_lane = MAX(0.0, MIN(1.0, car->_trkPos.toLeft / Width));

    wheelbase = (car->priv.wheel[FRNT_RGT].relPos.x +
        car->priv.wheel[FRNT_LFT].relPos.x -
        car->priv.wheel[REAR_RGT].relPos.x -
        car->priv.wheel[REAR_LFT].relPos.x) / 2;
    wheeltrack = (car->priv.wheel[FRNT_LFT].relPos.y +
        car->priv.wheel[REAR_LFT].relPos.y -
        car->priv.wheel[FRNT_RGT].relPos.y -
        car->priv.wheel[REAR_RGT].relPos.y) / 2;
}

////////////////////////////////////////////////////////////////////////////
// Car control
////////////////////////////////////////////////////////////////////////////
int LRaceLine::findNextBrakingZone()
{
    double prevspeed = tSpeed[LINE_RL][This];
    for (int i=0; i<Divs; i++)
    {
        int div = (This + i) % Divs;
        if (tSpeed[LINE_RL][div] < prevspeed)
            return div;
        prevspeed = tSpeed[LINE_RL][div];
    }
    return -1;
}

int LRaceLine::findNextCorner(tCarElt *theCar, int index, int *apex_div, double *dist)
{
    //tTrackSeg *seg = car->_trkPos.seg;;
    double distance = 0.0;
    int Index = (index >= 0 ? index % Divs : DivIndexForCar(theCar));
    int prefer_side = ((tRInverse[LINE_RL][Index] > 0.003) ? TR_LFT : 
                       ((tRInverse[LINE_RL][Index]) < -0.003 ? TR_RGT : TR_STR));
    //double curlane = car->_trkPos.toLeft / track->width;
    int /*next = (Next+5) % Divs,*/ i = 1, div = -1;
    //double distance = 0.0;
    double CR = tRInverse[LINE_RL][Index];

    if (car->_speed_x < 5.0)
    {
        prefer_side = TR_STR;
    }

    bool override_checked = false;
    if (overrideCollection)
    {
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_PREFERRED_SIDE);
        double override_value = 0.0;
        if (labelOverride)
        {
           if (labelOverride->getOverrideValue(Next, &override_value))
           {
               prefer_side = MIN(TR_STR, MAX(TR_RGT, (int)override_value));
               override_checked = true;
           }
        }
    }

    if (prefer_side == TR_STR && !override_checked)
    {
        int iend = MIN(100.0f, car->_speed_x);
        for (i=1; i<iend; i++)
        {
            div = (Index + i) % Divs;
            if (tRInverse[LINE_RL][div] > 0.001)
            {
                prefer_side = TR_LFT;
                break;
            }
            else if (tRInverse[LINE_RL][div] < -0.001)
            {
                prefer_side = TR_RGT;
                break;
            }
            double x1 = tx[LINE_RL][div], y1 = ty[LINE_RL][div];
            double x2 = tx[LINE_RL][(div+1)%Divs], y2 = ty[LINE_RL][(div+1)%Divs];
            distance += sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        }
    }
    *apex_div = -1;

    bool dist_done = (distance > 0.1);
    if (prefer_side != TR_STR && apex_div != NULL)
    {
        if (div == -1) div = Index;

        *apex_div = div;
        int count = 0;
        do
        {
            int nextdiv = (*apex_div + 1) % Divs;
            if (!dist_done)
            {
                double x1 = tx[LINE_RL][*apex_div], y1 = ty[LINE_RL][*apex_div];
                double x2 = tx[LINE_RL][(*apex_div+1)%Divs], y2 = ty[LINE_RL][(*apex_div+1)%Divs];
                distance += sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            }
            if ((tRInverse[LINE_RL][div] > 0 && tRInverse[LINE_RL][nextdiv] < 0.0) ||
                (tRInverse[LINE_RL][div] < 0 && tRInverse[LINE_RL][nextdiv] > 0.0))
            {
                *apex_div = -1;
                break;
            }
            if ((tRInverse[LINE_RL][div] < 0.0 && tRInverse[LINE_RL][nextdiv] < 0.0 && tLane[LINE_RL][nextdiv] < tLane[LINE_RL][*apex_div] && tLane[LINE_RL][nextdiv] > tLane[LINE_RL][Index]) ||
                (tRInverse[LINE_RL][div] > 0.0 && tRInverse[LINE_RL][nextdiv] > 0.0 && tLane[LINE_RL][nextdiv] > tLane[LINE_RL][*apex_div] && tLane[LINE_RL][nextdiv] < tLane[LINE_RL][Index]))
                break;
            *apex_div = nextdiv;
            count++;
        } while (*apex_div != div && count < 200);

        if (*apex_div == -1 && count < 200 && !override_checked)
        {
            // corner ended before finding an apex and a new corner started, change preferred_side
            if (prefer_side == TR_LFT)
                prefer_side = TR_RGT;
            else
                prefer_side = TR_LFT;
        }
    }

    if (dist)
        *dist = distance;
    return prefer_side;
}

int LRaceLine::DivIndexForCar(tCarElt *theCar, double catchtime)
{
    double dist = theCar->_trkPos.toStart;
    if (catchtime > 0.0)
        dist += catchtime * theCar->_speed_x;
    if (dist < 0)
        dist = 0;
    return DivIndexForCarDistance(theCar, dist);
}

int LRaceLine::DivIndexForCarDistance(tCarElt *theCar, double distance)
{
    int SegId = theCar->_trkPos.seg->id;
    tTrackSeg *seg = theCar->_trkPos.seg;
    if (distance > seg->length)
    {
        distance -= (seg->length - theCar->_trkPos.toStart);
        seg = seg->next;
        int count = 0;
        while (count < 10)
        {
            if (seg->length > distance)
                break;
            distance -= seg->length;
            seg = seg->next;
            count++;
        }
        SegId = seg->id;
    }
    if (seg->type != TR_STR)
    {
        distance *= seg->radius;
    }
    int Index = (tSegIndex[SegId] + int(distance / tElemLength[SegId])) % Divs;
    return Index;
}

double LRaceLine::getLookAhead(int rl, bool coll)
{
    double factor = 1.0;
    double spdmrgn = (coll ? 0.5 : 3.0);
    double speedfactor = (tSpeed[rl][Next] - car->_speed_x < spdmrgn ? 1.0 : MIN(1.0, (car->_speed_x+spdmrgn) / MIN(tSpeed[rl][Next], 80.0)));
    if ((rl == LINE_LEFT && tRInverse[rl][Next] < 0.0) || (rl == LINE_RIGHT && tRInverse[rl][Next] > 0.0))
        factor += fabs(tRInverse[rl][Next]) * 15;

    if (overrideCollection)
    {
        double lookahead = 0.0;
        if (rl == LINE_LEFT)
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD_LEFT);
            if (labelOverride)
               if (labelOverride->getOverrideValue(Next, &lookahead))
                   return lookahead * factor * speedfactor;
        }
        if (rl == LINE_RIGHT)
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD_RIGHT);
            if (labelOverride)
               if (labelOverride->getOverrideValue(Next, &lookahead))
                   return lookahead * factor * speedfactor;
        }
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_LOOKAHEAD);
        if (labelOverride)
           if (labelOverride->getOverrideValue(Next, &lookahead))
               return lookahead * factor * speedfactor;
    }
    double empty = 10.0f;
    double fuel_status = (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - empty) / (maxfuel-empty)));
    return (lookAheadEmpty - (lookAheadEmpty-lookAhead)*fuel_status) * factor * speedfactor;
}

int LRaceLine::DivIndex(RaceLineDriveData *data, tTrackSeg *seg, int raceline, double *X, double *Y)
{
    int Index = DivIndexForCar(car);
    This = Index; //sectors(fromstart)

    Index = (Index + Divs - 5) % Divs;

    int steer_verbose = 0;
    if (Time < 0.0)
        Time = data->s->deltaTime * 7;     // 0.50 + turnAccel/80;
    *X = car->_pos_X + car->_speed_X * Time / 2;
    *Y = car->_pos_Y + car->_speed_Y * Time / 2;
    data->lookahead = 0.0f;

    int tmp_Index = Index;
    do
    {
        Next = (Index + 1) % Divs;
        double dx = tx[raceline][Next] - car->_pos_X;
        double dy = ty[raceline][Next] - car->_pos_Y;
        data->lookahead = (float)sqrt(dx*dx + dy*dy) * MAX(0.5, 1.0 - fabs((tRInverse[raceline][Next]*100)*(tRInverse[raceline][Next]*100))*cornersteer);
        if (data->lookahead > getLookAhead(raceline, data->coll) && // + (1.0 - MIN(0.1, MAX(0.0, 1.0 - car->_fuel / (maxfuel/2))/11)) &&
            ((tx[raceline][Next] - tx[raceline][Index]) * (*X - tx[raceline][Next]) +
             (ty[raceline][Next] - ty[raceline][Index]) * (*Y - ty[raceline][Next]) < 0.1))
            break;
        Index = Next;
    } while (tmp_Index != Index);

    data->lookahead = AdjustLookahead(raceline, data->lookahead, seg);

    if (raceline >= LINE_RL)
    {
       int next = Index;
       double maxFabsRI = -1.0;
       do
       {
           next = (next + 1) % Divs;
           if (fabs(tRInverse[raceline][next]) > 0.01)
           {
               if (fabs(tRInverse[raceline][next]) > maxFabsRI)
                   maxFabsRI = fabs(tRInverse[raceline][next]);
               else if (fabs(tRInverse[raceline][next]) < maxFabsRI)
               {
                   data->next_apex = next;
                   break;
               }
           }
       } while (next != Index);
    }

    Index = tmp_Index;
    return Index;
}

float LRaceLine::AdjustLookahead(int raceline, float lookahead, tTrackSeg *seg)
{
    if ((tRInverse[raceline][Next] > 0.0 && car->_trkPos.toMiddle < 0.0) ||
        (tRInverse[raceline][Next] < 0.0 && car->_trkPos.toMiddle > 0.0))
        lookahead *= (float)MIN(4.0f, 1.5f + fabs(car->_trkPos.toMiddle*0.3));
    else
        lookahead *= (float)MAX(0.7f, 1.5f - fabs(car->_trkPos.toMiddle*0.2));

#if 1
    if ((tRInverse[raceline][Next] < 0.0 && car->_trkPos.toMiddle > 0.0) ||
        (tRInverse[raceline][Next] > 0.0 && car->_trkPos.toMiddle < 0.0))
    {
        lookahead *= (float)MAX(1.0f, MIN(3.6f, 1.0f + (MIN(2.6f, fabs(car->_trkPos.toMiddle) / (seg->width / 2)) / 2) * (1.0f + fabs(tRInverse[raceline][Next]) * 65.0f + car->_speed_x / 120.0f)));
    }
    else if ((tRInverse[raceline][Next] < 0.0 && car->_trkPos.toRight < 5.0) ||
        (tRInverse[raceline][Next] > 0.0 && car->_trkPos.toLeft < 5.0))
    {
        lookahead *= (float)MAX(0.8f, MIN(1.0f, 1.0f - fabs(tRInverse[raceline][Next])*200.0 * ((5.0f - MIN(car->_trkPos.toRight, car->_trkPos.toLeft)) / 5.0f)));
    }
#endif

    return lookahead;
}

double LRaceLine::CalculateSpeed(RaceLineDriveData *data, double X, double Y, int Index, int raceline)
{
    int rl = MIN(raceline, LINE_RL);
    data->target->x = (float)tx[rl][Next];
    data->target->y = (float)ty[rl][Next];

    double c0 = (tx[rl][Next] - tx[rl][Index]) * (tx[rl][Next] - X) +
        (ty[rl][Next] - ty[rl][Index]) * (ty[rl][Next] - Y);
    double c1 = (tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Index]) +
        (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Index]);
    double sum = c0 + c1;

    c0 /= sum;
    c1 /= sum;
    TargetSpeed = (1 - c0) * tSpeed[raceline][Next] + c0 * tSpeed[raceline][Index];
    if (raceline >= LINE_RL && useMergedSpeed)
    {
        double TargetSpeedSlow = (1 - c0) * tSpeed[LINE_RL_SLOW][Next] + c0 * tSpeed[LINE_RL_SLOW][Index];
        double TargetSpeedFast = (1 - c0) * tSpeed[LINE_RL][Next] + c0 * tSpeed[LINE_RL][Index];
        double empty = 10.0f;
        double fuel_status = (car->_fuel <= empty ? 0.0 : (MAX(0.0, car->_fuel - empty) / (maxfuel-empty)));
        TargetSpeed = TargetSpeedFast - (TargetSpeedFast-TargetSpeedSlow)*fuel_status;
    }

    if (TargetSpeed < 10.0f)
    {
        //fprintf(stderr,"RL=%d Index=%d Next=%d c0=%.3f sum=%.3f tSpeed[Next]=%.3f tSpeed[Index]=%.3f TargetSpeed=%.3f\n",raceline,Index,Next,c0,sum,tSpeed[raceline][Next],tSpeed[raceline][Index],TargetSpeed);
        //fflush(stderr);
    }

    /* WARNING: problems on Corkscrew pit in with TargetSpeed being negative? */
    //*speed = TargetSpeed;
    data->speed = (float)MAX(10.0f, TargetSpeed);

    if (raceline == LINE_LEFT)
        data->left_speed_outsteer = (float)((1 - c0) * tSpeed[LINE_LEFT_OUTSTEER][Next] + c0 * tSpeed[LINE_LEFT_OUTSTEER][Index]);
    else if (raceline == LINE_RIGHT)
        data->right_speed_outsteer = (float)((1 - c0) * tSpeed[LINE_RIGHT_OUTSTEER][Next] + c0 * tSpeed[LINE_RIGHT_OUTSTEER][Index]);

    return c0;
}

double LRaceLine::CalculateOffset(int raceline)
{
    double laneoffset = (tLane[raceline][Next] * Width);
    return laneoffset;
}

double LRaceLine::CalculateMixedCurvature(double c0, int Index, double transition_percentage)
{
    double rInverse_next = tRInverse[LINE_LEFT][Next] + (tRInverse[LINE_RIGHT][Next] - tRInverse[LINE_LEFT][Next]) * (1.0 - transition_percentage);
    double rInverse_index = tRInverse[LINE_LEFT][Index] + (tRInverse[LINE_RIGHT][Index] - tRInverse[LINE_LEFT][Index]) * (1.0 - transition_percentage);
    double TargetCurvature = (1 - c0) * rInverse_next + c0 * rInverse_index;
    if (fabs(TargetCurvature) > 0.01)
    {
        double r = 1 / TargetCurvature;
        if (r > 0)
            r -= wheeltrack / 2;
        else
            r += wheeltrack / 2;
        TargetCurvature = 1 / r;
    }
    return TargetCurvature;
}

double LRaceLine::CalculateCurvature(double c0, int Index, int raceline)
{
    //
    // Find target curvature (for the inside wheel)
    //
    double TargetCurvature = (1 - c0) * tRInverse[raceline][Next] + c0 * tRInverse[raceline][Index];
    if (fabs(TargetCurvature) > 0.001)
    {
        double r = 1 / TargetCurvature;
        if (r > 0)
            r -= wheeltrack / 2;
        else
            r += wheeltrack / 2;
        TargetCurvature = 1 / r;
    }
    return TargetCurvature;
}

void LRaceLine::slowestSpeedBetweenDivs(int startdiv, int enddiv, double *rlspeed, double *leftspeed, double *rightspeed)
{
    if (enddiv < startdiv)
        enddiv += Divs;

    *rlspeed = *leftspeed = *rightspeed = 10000;

    for (int i=startdiv; i<=enddiv; i++)
    {
        int div = i % Divs;
        *rlspeed = MIN(*rlspeed, tSpeed[LINE_RL][div]);
        *leftspeed = MIN(*leftspeed, tSpeed[LINE_LEFT][div]);
        *rightspeed = MIN(*rightspeed, tSpeed[LINE_RIGHT][div]);
    }
}

void LRaceLine::updateRLSpeedMode()
{
    int ideal_speed_mode = LINE_RL_SLOW;
    if (car->_fuel / maxfuel < midtankPercent || (!hasSlow && !hasMid))
    {
        ideal_speed_mode = LINE_RL;
    }
    else if (car->_fuel / maxfuel < fulltankPercent || !hasSlow)
    {
        ideal_speed_mode = LINE_RL_MID;
    }

    if (rl_speed_mode == -1 || ideal_speed_mode < rl_speed_mode || (ideal_speed_mode > rl_speed_mode && fabs(tRInverse[LINE_RL][Next]) < 0.001))
    {
        rl_speed_mode = ideal_speed_mode;
    }
}

float LRaceLine::smoothSteering(float steercmd, float laststeer)
{
    /* try to limit sudden changes in steering to avoid loss of control through oversteer. */
    double speedfactor = (((60.0 - (MAX(40.0, MIN(70.0, cardata->getSpeedInTrackDirection() + MAX(0.0, car->_accel_x * 5))) - 25)) / 300) * 1.2) / 0.785 * 0.75;

    if (fabs(steercmd) < fabs(laststeer) && fabs(steercmd) <= fabs(laststeer - steercmd))
        speedfactor *= 2;

    steercmd = (float)MAX(laststeer - speedfactor, MIN(laststeer + speedfactor, steercmd));
    return steercmd;
}

double LRaceLine::Point2Lane(int rl, double x, double y)
{
    double lx = txLeft[rl][Next], ly = tyLeft[rl][Next];
    double rx = txRight[rl][Next], ry = tyRight[rl][Next];
    double dx = lx - rx;
    double dy = ly - ry;
    double disttotal = sqrt((dx * dx) + (dy * dy));
    dx = lx - x;
    dy = ly - y;
    double distpoint = sqrt((dx * dx) + (dy * dy));
    return distpoint / disttotal;
}

void LRaceLine::GetRaceLineData(RaceLineDriveData *data, bool transitioning)
{
    double target_steer;

    updateRLSpeedMode();
    int target_raceline = data->linemode->getTargetLine();
    int source_raceline = data->linemode->getSourceLine();

    bool is_transitioning = data->linemode->is_transitioning;

    if (racelineOverride >= 0)
        target_raceline = racelineOverride;

    if (target_raceline != LINE_RL && overrideCollection)
    {
        double tmp;
        LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_RL_FOR_OVERTAKE);
        if (labelOverride)
        {
           if (labelOverride->getOverrideValue(Next, &tmp))
           {
               if (((int)tmp == 1 && target_raceline == LINE_LEFT) || ((int)tmp == 2 && target_raceline == LINE_RIGHT))
               {
                   target_raceline = LINE_RL;
                   if (source_raceline == LINE_RL && !data->linemode->has_transitioned)
                       is_transitioning = false;
               }
           }
        }
    }

    // get the steering & speed for each of the three racelines.
    // we do this regardless of whether transitioning/avoiding or not, as
    // the information will be needed the moment we want to start a
    // new transition...
    SteerTheCar(data, LINE_LEFT);
    double left_steer = data->racesteer;
    double left_error = data->error;
    data->left_speed = data->speed;
    SteerTheCar(data, LINE_RIGHT);
    double right_steer = data->racesteer;
    double right_error = data->error;
    data->right_speed = data->speed;
    double mid_speed = (data->left_speed + data->right_speed)/2;
    SteerTheCar(data, rl_speed_mode);
    double rl_steer = data->racesteer;
    double rl_speed = data->speed;

    double target_speed = 0.0;

#if 1
    double current_lane, projected_lane, target_lane, source_lane;
    double source_steer, source_speed;

    current_lane = MAX(0.0, MIN(1.0, car->_trkPos.toLeft / Width));
    //projected_lane = current_lane + (current_lane - last_lane) * car->_speed_x / 10;
    projected_lane = current_lane; // + (data->speedangle*10)/Width;
    if (car->_speed_x < 15)
      projected_lane = (current_lane*Width + (data->speedangle + tRInverse[LINE_RL][Next]*10))/Width;
    double new_lane = (current_lane*0.2 + projected_lane*0.8);

    // set the target line steer & speed variables
    if (target_raceline == LINE_RL)
    {
        target_lane = tLane[LINE_RL][Next];
        target_steer = rl_steer;
        target_speed = rl_speed;
    }
    else if (target_raceline == LINE_LEFT)
    {
        target_lane = MIN(tLane[LINE_RL][Next], Point2Lane(LINE_RL, tx[LINE_LEFT][Next], ty[LINE_LEFT][Next]));
        target_steer = (is_transitioning ? MAX(rl_steer, left_steer) : left_steer);
        target_speed = data->left_speed;
    }
    else if (target_raceline == LINE_RIGHT)
    {
        target_lane = MAX(tLane[LINE_RL][Next], Point2Lane(LINE_RL, tx[LINE_RIGHT][Next], ty[LINE_RIGHT][Next]));
        target_steer = (is_transitioning ? MIN(rl_steer, right_steer) : right_steer);
        target_speed = data->right_speed;
    }
    else if (target_raceline == LINE_MID)
    {
        target_lane = (tLane[LINE_LEFT][Next] + tLane[LINE_RIGHT][Next])/2;
        target_steer = (left_steer + right_steer)/2;
        target_speed = (data->left_speed + data->right_speed)/2;
    }

    if (source_raceline == LINE_RL)
    {
        source_lane = tLane[LINE_RL][Next];
        source_steer = rl_steer;
        source_speed = rl_speed;
    }
    else if (source_raceline == LINE_LEFT)
    {
        source_lane = Point2Lane(LINE_RL, tx[LINE_LEFT][Next], ty[LINE_LEFT][Next]);
        source_steer = (is_transitioning ? MAX(rl_steer, left_steer) : left_steer);
        source_speed = data->left_speed;
    }
    else if (source_raceline == LINE_RIGHT)
    {
        source_lane = Point2Lane(LINE_RL, tx[LINE_RIGHT][Next], ty[LINE_RIGHT][Next]);
        source_steer = (is_transitioning ? MIN(rl_steer, right_steer) : right_steer);
        source_speed = data->right_speed;
    }
    else if (source_raceline == LINE_MID)
    {
        source_lane = 0.5;
        source_steer = (left_steer + right_steer)/2;
        source_speed = (data->left_speed + data->right_speed)/2;
    }

    target_steer = MIN(1.0, MAX(-1.0, target_steer));
    data->leftlane_2left = Point2Lane(LINE_RL, tx[LINE_LEFT][Next], ty[LINE_LEFT][Next]) * Width;
    data->rightlane_2right = (1.0-Point2Lane(LINE_RL, tx[LINE_RIGHT][Next], ty[LINE_RIGHT][Next])) * Width;

    double transition_increment = data->linemode->transitionIncrement(Next, source_steer, target_steer);
    bool outsteer = false;
    double skidAng = cos(atan2(car->_speed_Y, car->_speed_X) - car->_yaw);
    double ti = transition_increment * 2;
    double transition_lanediff = 0.0;
    bool isOnHold = false;

    if (is_transitioning)  // switching between racelines...
    {
        data->linemode->has_transitioned = true;
        if (last_target_raceline != target_raceline)
            last_steer_diff = 1000.0;
        last_target_raceline = target_raceline;

        double stay_inside = 0.0;
        if (overrideCollection)
        {
            LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_STAY_INSIDE);
            if (labelOverride)
               if (!labelOverride->getOverrideValue(Next, &stay_inside))
                   stay_inside = 0.0;
        }

        if ((data->linemode->isOnHold(This, (stay_inside > 0.5)) && target_raceline >= LINE_RL &&
             ((source_raceline != LINE_RIGHT && rl_steer < left_steer) ||
              (source_raceline != LINE_LEFT && rl_steer > right_steer))))
        {
            if (target_lane > source_lane)
                transition_lanediff = MAX(0.0, MIN(1.0, (target_lane - projected_lane) / (target_lane - source_lane)));
            else if (target_lane < source_lane)
                transition_lanediff = MAX(0.0, MIN(1.0, (projected_lane - target_lane) / (source_lane - target_lane)));
            if (transition_lanediff > 0.5)
            {
                target_raceline = source_raceline;
                target_steer = source_steer;
                target_lane = source_lane;
                isOnHold = true;
                data->linemode->setOnHold();
                ti = data->linemode->transitionIncrement(Next, source_steer, target_steer) * 2;
            }
        }
        else if (target_raceline == LINE_RL && !outsteer)
            ti *= 1.5;

        {
//            if (((target_lane<new_lane || data->speedangle > fabs(tRInverse[LINE_RL][Next])) && tRInverse[LINE_RL][Next] < -0.0015) || 
 //               ((target_lane>new_lane || data->speedangle < 1.0-fabs(tRInverse[LINE_RL][Next])) && tRInverse[LINE_RL][Next] > 0.0015))
            int tmp = 0;
            double dst2corner = 0.0;
            int prefer_line = findNextCorner(car, This, &tmp, &dst2corner);
            if (/*(source_raceline == LINE_LEFT && prefer_line == TR_LFT && dst2corner < car->_speed_x*2) ||
                (source_raceline == LINE_RIGHT && prefer_line == TR_RGT && dst2corner < car->_speed_x*2) || */
                ((target_raceline == LINE_LEFT || (target_raceline == LINE_RL && data->error < 0.0)) && tRInverse[LINE_RL][Next] < -0.0005) ||
                ((target_raceline == LINE_RIGHT || (target_raceline == LINE_RL && data->error > 0.0)) && tRInverse[LINE_RL][Next] > 0.0005))
            {
                outsteer = true;
                bool accelerating = (tSpeed[LINE_RL][Next] > tSpeed[LINE_RL][This] && MIN(source_speed, target_speed) > car->_speed_x);
                double outside_dampener = (accelerating && target_raceline < LINE_RL ? outsideSteeringDampenerAccel : outsideSteeringDampener);
                if (overrideCollection)
                {
                    double tmp = outside_dampener;
                    LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_OUTSIDE_DAMPENER);
                    if (labelOverride)
                       if (!labelOverride->getOverrideValue(Next, &outside_dampener))
                           outside_dampener = tmp;
                }
#if 1
                //if (car->_speed_x > 30)
                //    outside_dampener *= (1.0 - (MIN(car->_speed_x, 75) - 30) / 60);
                ti *= outside_dampener;
#else

                /* if (fabs(tRInverse[LINE_RL][Next]) > 0.02)
                    outside_dampener *= MAX(0.3, 1.0 - ((fabs(tRInverse[LINE_RL][Next])-0.02)*100));
                else */ if (car->_speed_x > 30)
                {
                    double f = (1.0 + MIN(1.0, (car->_speed_x-30) / 30));
                    outside_dampener *= f * f * f;
                }
                if (data->avoidmode & avoidside)
                    outside_dampener /= 2;

                // allow greater movement away from inside of corner, less away movt at outside
                if (tRInverse[LINE_RL][Next] < 0.0)
                    ti *= MIN(1.0, 1.0 - (car->_trkPos.toMiddle/(Width/2))/3);
                else
                    ti *= MIN(1.0, 1.0 + (car->_trkPos.toMiddle/(Width/2))/3);

                ti = MAX(ti/10, ti + outside_dampener * fabs(tRInverse[target_raceline][This])/10);
#endif
            }
            //else
            //    ti *= MAX(0.2, 1.0 - fabs(tRInverse[LINE_RL][Next]) * 20);

#if 0
            if ((new_lane > current_lane && target_lane < current_lane) || (new_lane < current_lane && target_lane > current_lane))
            {
                // reduce TI in order to avoid a harsh jerk on changing direction
                ti /= 2;
            }
#endif
        }


        if (car->_speed_x > 30)
            ti *= MIN(1.0, MAX(0.2, 1.0 - ((car->_speed_x - 30) / 80)));
        if (car->_accel_x > 4.0)
            ti *= MIN(1.0, MAX(0.2, 1.0 - (car->_accel_x-4.0)/13));
        if (data->s->currentTime < 5)
            ti *= MAX(0.2, data->s->currentTime / 5);

        if (target_lane < projected_lane)
        {
            //if (target_lane > source_lane)
             //   new_lane = target_lane;
            //else
                new_lane = MAX(target_lane, projected_lane - ti);
        }
        else
        {
            //if (target_lane < source_lane)
            //    new_lane = target_lane;
            //else
                new_lane = MIN(target_lane, projected_lane + ti);
        }
        data->target_lane = (float)new_lane;

#if 1
        if (target_lane > source_lane)
            transition_lanediff = MAX(0.0, MIN(1.0, (target_lane - new_lane) / (target_lane - source_lane)));
        else if (target_lane < source_lane)
            transition_lanediff = MAX(0.0, MIN(1.0, (new_lane - target_lane) / (source_lane - target_lane)));

        if (target_steer < source_steer)
            data->racesteer = (float)MAX(target_steer, target_steer + (source_steer - target_steer) * transition_lanediff);
        else
            data->racesteer = (float)MIN(target_steer, target_steer - (target_steer - source_steer) * transition_lanediff);
        if (steerSkidOfflineFactor > 0)
        {
            // skid correction to keep the car straight
            // no need for this unless we're midway through a transition
            double carspeed = hypot(car->_speed_x, car->_speed_y);
            double vx = car->_speed_X;
            double vy = car->_speed_Y;
            double dirx = cos(car->_yaw);
            double diry = sin(car->_yaw);
            double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
            Skid = MAX(-0.9, MIN(0.9, Skid));
            data->racesteer += (float)((asin(Skid) / car->_steerLock) * steerSkidOfflineFactor);
        }
//fprintf(stderr, "%d/%d out=%d cur_lane=%.3f proj_lane=%.3f trg_lane=%.3f ti=%.3f/%.3f new_lane=%.3f steer=%.3f trg_steer=%.3f\n",This,Next,outsteer,current_lane,projected_lane,target_lane,ti,transition_increment,new_lane,data->racesteer,target_steer);fflush(stderr);
#else
        {
            double lookahead = 1.0 * MAX(1.0, MIN(2.0, 1.0 + car->_accel_x/2));
            double aheadDist = car->_dimension_x / 2 + 48 * lookahead;
            //double aheadDist = car->_dimension_x / 2 + hypot(car->_speed_x, car->_speed_y) * lookahead;
            double dist = 0.0;
            int thisdiv = This, nextdiv;
            vec2f t;
            for (int i=1; i<Divs; i++)
            {
                nextdiv = (thisdiv+i) % Divs;
                dist += sqrt((tx[LINE_RL][nextdiv]-tx[LINE_RL][thisdiv])*(tx[LINE_RL][nextdiv]-tx[LINE_RL][thisdiv]) + (ty[LINE_RL][nextdiv]-ty[LINE_RL][thisdiv])*(ty[LINE_RL][nextdiv]-ty[LINE_RL][thisdiv]));
                if (aheadDist < dist)
                {
                    t.x = txLeft[target_raceline][nextdiv] + (txRight[target_raceline][nextdiv]-txLeft[target_raceline][nextdiv]) * new_lane;
                    t.y = tyLeft[target_raceline][nextdiv] + (tyRight[target_raceline][nextdiv]-tyLeft[target_raceline][nextdiv]) * new_lane;
                    break;
                }
                thisdiv = nextdiv;
            }
            /*
            double trackpos = RtGetDistFromStart(car) + aheadDist;
            double offset = (Width - (new_lane*Width)) - Width/2;
            vec2f t = driver->getTargetPoint(0.0, offset, -1.0f); //(float)aheadDist);
            */
            double angle = atan2(t.y - car->_pos_Y, t.x - car->_pos_X) - car->_yaw;
            NORM_PI_PI(angle);
            data->racesteer = angle / car->_steerLock;

            double carspeed = hypot(car->_speed_x, car->_speed_y);
            double vx = car->_speed_X;
            double vy = car->_speed_Y;
            double dirx = cos(car->_yaw);
            double diry = sin(car->_yaw);
            double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
            Skid = MAX(-0.9, MIN(0.9, Skid));
            data->racesteer += (float)((asin(Skid) / car->_steerLock) * 0.6);
        }
fprintf(stderr, "%d/%d out=%d cur_lane=%.3f proj_lane=%.3f trg_lane=%.3f ti=%.3f/%.3f new_lane=%.3f steer=%.3f\n",This,Next,outsteer,current_lane,projected_lane,target_lane,ti,transition_increment,new_lane,data->racesteer);fflush(stderr);
#endif

/*
        if (target_steer > data->racesteer)
            data->racesteer = MAX(data->racesteer, target_steer - last_steer_diff);
        else if (data->racesteer > target_steer)
            data->racesteer = MIN(data->racesteer, target_steer + last_steer_diff);
            */
        last_steer_diff = fabs(target_steer - data->racesteer);

        //data->racesteer = SteerTheCar(new_lane, data->s->deltaTime);
            
        // calculate the speed based on track position
        double track_percentage = MIN(1.0, MAX(0.0, current_lane * 1.0 - (fabs(tRInverse[LINE_RL][This])*100)));
        bool braking = (tSpeed[source_raceline][Next] < tSpeed[source_raceline][This] || tSpeed[target_raceline][Next] < tSpeed[target_raceline][This]);
        if (target_speed < car->_speed_x)
            data->speed = (float)target_speed;
        else if ((source_raceline != LINE_RL && target_raceline != LINE_RL) || braking)
            data->speed = (float)(data->left_speed + (data->right_speed - data->left_speed) * track_percentage);
        //else if (outsteer)
        //    data->speed = ((target_speed - (target_speed - source_speed) * transition_lanediff) + (data->left_speed + (data->right_speed - data->left_speed) * track_percentage)) / 2;
        else
            data->speed = (float)(target_speed - (target_speed - source_speed) * transition_lanediff);

        if (outsteer ||
            (target_raceline == LINE_RIGHT && tRInverse[LINE_RIGHT][Next] < -0.001 && right_error > 0.0) ||
            (target_raceline == LINE_LEFT && tRInverse[LINE_LEFT][Next] > 0.001 && left_error < 0.0) ||
            (target_raceline == LINE_RL && tRInverse[LINE_RL][Next] < -0.001 && data->error > 0.0) ||
            (target_raceline == LINE_RL && tRInverse[LINE_RL][Next] > 0.001 && data->error < 0.0))
        {
            data->speed -= (float)MIN(4.0, fabs(tRInverse[target_raceline][This])*200*outsteerSpeedReducer);
            target_speed = data->speed;
            /*
            if (tRInverse[target_raceline][This] < 0.0)
                data->left_speed = target_speed = data->left_speed_outsteer;
            else
                data->right_speed = target_speed = data->right_speed_outsteer;
                */
        }
        if (target_raceline == LINE_RL && fabs(target_steer) < 0.5 && !braking)
        {
            double steer_percentage = 1.0 - fabs(data->racesteer - target_steer) / 2.0;
            if (source_raceline == LINE_LEFT)
                data->speed = (float)(data->left_speed + (target_speed - data->left_speed) * steer_percentage);
            else if (source_raceline == LINE_RIGHT)
                data->speed = (float)(data->right_speed + (target_speed - data->right_speed) * steer_percentage);
            else if (source_raceline == LINE_MID)
                data->speed = (float)(source_speed + (target_speed - source_speed) * steer_percentage);
            else
                data->speed = (float)(rl_speed + (target_speed - rl_speed) * steer_percentage);
        }
        if (target_raceline == LINE_RL)
        {
            if ((tRInverse[LINE_RL][Next] > 0.001 && data->error < 0.0) || (tRInverse[LINE_RL][Next] < -0.001 && data->error > 0.0))
                data->speed = MAX(data->speed, rl_speed * (1.0 - fabs(data->error)*1.75));
            else
                data->speed = MAX(data->speed, rl_speed);
        }
        if (data->speed > car->_speed_x)
        {
            // don't accelerate if car is skidding as it'll cause a spinout
            data->speed = (float)MAX(car->_speed_x - 0.0, data->speed * (skidAng * skidAng));
        }
    }
    else
    {
        // we're on a raceline (not transitioning)
        data->target_lane = (float)target_lane;
        data->racesteer = (float)target_steer;
        if (target_raceline == LINE_LEFT)
        {
            data->speed = data->left_speed;
            if (left_error < -0.1 && fabs(tRInverse[LINE_LEFT][Next]) > 0.001)
                data->speed = MIN(car->_speed_x-0.1, data->speed + left_error * 2);
        }
        else if (target_raceline == LINE_RIGHT)
        {
            data->speed = data->right_speed;
            if (right_error > 0.1 && fabs(tRInverse[LINE_RIGHT][Next]) < -0.001)
                data->speed = MIN(car->_speed_x-0.1, data->speed - right_error * 2);
        }
        else if (target_raceline == LINE_MID)
            data->speed = (float)mid_speed;

        last_target_raceline = -1;
        last_steer_diff = 1000.0;
    }
    last_lane = current_lane;

#ifdef WIN32
    if (data->s->_raceType == RM_TYPE_PRACTICE)
    {
        char buf[16];
        fprintf(stderr, "%d %d: [%d:%s] spd=%.3f l%.2f r%.2f R%.2f %.2f dist=%.3f tA=%.2f rI=%.4f ti=%.3f tdf=%.3f steer=%.3f %.3f lne=c%.2f p%.2f s%.2f t%.2f n%.2f skidA=%.3f spdA=%.3f err=%.3f acc=%.2f os=%d\n",This,Next,!data->linemode->is_transitioning, (target_raceline==LINE_LEFT?"left":(target_raceline==LINE_RIGHT?"right":(target_raceline==LINE_MID?"mid":(target_raceline>=LINE_RL?"rl":itoa(target_raceline,buf,10))))),car->_speed_x,data->left_speed,data->right_speed,rl_speed,data->speed,car->_distFromStartLine,RtTrackSideTgAngleL(&(car->_trkPos)),tRInverse[LINE_RL][This],ti,transition_lanediff,data->laststeer,data->racesteer,current_lane,projected_lane,source_lane,target_lane,new_lane,skidAng,data->speedangle,data->error,car->_accel_x,outsteer);fflush(stderr);
    }
#endif
    
    if (!isOnHold && target_raceline != LINE_MID && /* !outsteer &&*/ data->linemode->is_transitioning && 
        car->_speed_x <= MIN(target_speed, source_speed) &&
        fabs(target_steer - data->racesteer) < 0.2 && fabs(data->racesteer) < 0.6 &&
        fabs(target_lane*Width - current_lane*Width) < 0.5)
    {
        // check to see if we're still transitioning...
//fprintf(stderr,"TRANSITION COMPLETE %d to %d\n", source_raceline, target_raceline);
        data->linemode->is_transitioning = false;
        data->linemode->setTargetLineToSource();
        last_steer_diff = 1000.0;
        last_target_raceline = -1;
        //data->linemode->checkTransition(data->racesteer, data->laststeer);
    }

    last_last_steer = last_steer;
    last_steer = data->racesteer;
#else
    // set the target line steer & speed variables
    if (target_raceline == LINE_RL)
    {
        last_target_steer = last_rl_steer;
        target_steer = rl_steer;
        target_speed = rl_speed;
    }
    else if (target_raceline == LINE_LEFT)
    {
        last_target_steer = last_left_steer;
        target_steer = (data->linemode->is_transitioning ? MAX(rl_steer, left_steer) : left_steer);
        target_speed = data->left_speed;
    }
    else
    {
        last_target_steer = last_right_steer;
        target_steer = (data->linemode->is_transitioning ? MIN(rl_steer, right_steer) : right_steer);
        target_speed = data->right_speed;
    }
    
    // get the source raceline speed/steer variables
    double source_steer, source_speed, last_source_steer;
    if (source_raceline == LINE_RL)
    {
        last_source_steer = last_rl_steer;
        source_steer = rl_steer;
        source_speed = rl_speed;
    }
    else if (source_raceline == LINE_LEFT)
    {
        last_source_steer = last_left_steer;
        source_steer = left_steer;
        source_speed = data->left_speed;
    }
    else if (source_raceline == LINE_RIGHT)
    {
        last_source_steer = last_right_steer;
        source_steer = right_steer;
        source_speed = data->right_speed;
    }
    else
    {
        last_source_steer = (last_left_steer + last_right_steer) / 2;
        source_steer = (left_steer + right_steer) / 2;
        source_speed = (data->left_speed + data->right_speed) / 2;
    }

    bool accelerating = (tSpeed[LINE_RL][Next] > tSpeed[LINE_RL][This] && MIN(source_speed, target_speed) > car->_speed_x);
    double transition_increment = data->linemode->transitionIncrement() * (target_raceline >= LINE_RL ? (!accelerating ? 1.4 : 0.5) : 1.0);
    double transition_percentage = data->linemode->transition_percentage;
    double current_steer = -1.0;

    // not going to transition if car is stuck or facing the wrong way
    if (fabs(data->angle) > 0.9 || car->_speed_x < 5)
        data->linemode->is_transitioning = false;

    double toLeft = car->_trkPos.toLeft;
    double track_percentage = MAX(0.0, MIN(1.0, toLeft / Width));
    double transition_time = data->linemode->getTransitionTime();
    double trackpos_steer = (left_steer * (1.0-track_percentage)) + (right_steer * track_percentage);
    double projected_steer = last_steer + (last_steer - last_last_steer) * MIN(0.8, MAX(0.3, (0.8 - transition_time)));
    double steer1 = (trackpos_steer * 0.2 + projected_steer * 0.8);
    double steer_diff = 0.0, steer_diff2 = 0.0;

    double projected_percentage = (track_percentage + (track_percentage - last_percentage));

    if (data->linemode->is_transitioning)  // switching between racelines...
    {
        int apex_div = -1;
        int next_corner = findNextCorner(car, Next, &apex_div);
        int target_rl = data->linemode->getTargetLine();
        int source_rl = data->linemode->getSourceLine();
        int nextnext = (Next + MAX(10, (int)(car->_speed_x*0.45))) % Divs;
        bool hold_the_line = false;
        double steer_change = 0, steer_change_factor = 0;

        {
            // skid correction to keep the car straight
            // no need for this unless we're midway through a transition
            double carspeed = hypot(car->_speed_x, car->_speed_y);
            double vx = car->_speed_X;
            double vy = car->_speed_Y;
            double dirx = cos(car->_yaw);
            double diry = sin(car->_yaw);
            double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
            Skid = MAX(-0.9, MIN(0.9, Skid));
            steer1 += (asin(Skid) / car->_steerLock) * steerSkidOfflineFactor;
        }
        steer_diff = target_steer - steer1;
        double steer2 = steer1, steer3 = steer1;

#if 1
        if ((data->linemode->isOnHold(This) &&
             ((source_rl == LINE_LEFT && rl_steer < left_steer) ||
              (source_rl == LINE_RIGHT && rl_steer > right_steer))) /*||
            (target_rl >= LINE_RL && 
             (tSpeed[target_rl][nextnext] < car->_speed_x ||
              tSpeed[target_rl][Next] < car->_speed_x) &&
             ((source_rl == LINE_LEFT && next_corner == TR_LFT) ||
              (source_rl == LINE_RIGHT && next_corner == TR_RGT))) */)
        {
            // don't transition back to raceline as with upcoming corner it
            // would take the car away from the apex
            hold_the_line = true;
            transition_increment = transition_percentage = 0.0;
        }
//fprintf(stderr, "TRANSITION %s->%s %d\n",(source_rl == LINE_LEFT?"LFT":(source_rl==LINE_RIGHT?"RGT":"RL")),(target_rl==LINE_LEFT?"LFT":(target_rl==LINE_RIGHT?"RGT":"RL")),hold_the_line);fflush(stderr);
#endif
    
        bool outsteer = false;
        //if (!hold_the_line && target_raceline == LINE_RL)
        //    transition_increment *= 2;

        {
            double target_lane = tLane[target_raceline][Next];
            if ((target_lane > track_percentage && 
            if (last_steer_diff < 999.0)
            {
                if (last_steer_diff > 0.0 && steer_diff > last_steer_diff)
                    steer2 = target_steer - (last_steer_diff + steer_diff) / 2;
                else if (last_steer_diff < 0.0 && steer_diff < last_steer_diff)
                    steer2 = target_steer - (last_steer_diff + steer_diff) / 2;
            }
            steer_diff2 = target_steer - steer2;

            steer_change_factor = (transition_increment + fabs(tRInverse[LINE_RL][This])/10);
            if ((target_steer > last_steer && (tRInverse[target_raceline][This] < 0.0 || (next_corner == TR_RGT && !accelerating))) ||
                (target_steer < last_steer && (tRInverse[target_raceline][This] > 0.0 || (next_corner == TR_LFT && !accelerating))))
            {
                outsteer = true;           
                //if ((target_steer > last_steer && last_steer < steer2) || (target_steer < last_steer && last_steer > steer2))
                //    steer2 = (last_steer + steer2) / 2;

                double outside_dampener = (accelerating && target_raceline < LINE_RL ? outsideSteeringDampenerAccel : outsideSteeringDampener);
                if (overrideCollection)
                {
                    LManualOverride *labelOverride = overrideCollection->getOverrideForLabel(PRV_OUTSIDE_DAMPENER);
                    if (labelOverride)
                       if (!labelOverride->getOverrideValue(Next, &outside_dampener))
                           outside_dampener = outsideSteeringDampener;
                }
                if (fabs(tRInverse[LINE_RL][Next]) > 0.02)
                    outside_dampener *= MAX(0.3, 1.0 - ((fabs(tRInverse[LINE_RL][Next])-0.02)*100));
                else if (car->_speed_x > 40)
                {
                    double f = (1.0 + MIN(1.0, (car->_speed_x-40) / 40));
                    outside_dampener *= f * f * f;
                }
                if (data->avoidmode & avoidside)
                    outside_dampener /= 2;
                
                //double speed_percentage = MAX(0.8, car->_speed_x / target_speed);
                //outside_dampener *= speed_percentage;
                //if (target_steer > last_steer)
                //    outside_dampener *= MIN(1.0, (car->_trkPos.toRight / car->_trkPos.seg->width + 0.5) * 1.3);
                //else
                //    outside_dampener *= MIN(1.0, (car->_trkPos.toLeft / car->_trkPos.seg->width + 0.5) * 1.3);

                double min_change_factor = 0.05;//(target_rl >= LINE_RL && accelerating ? MAX(0.03, 0.5 - MAX(0.0, (car->_speed_x-25)/100)) : 0.03);
                steer_change_factor = MAX(steer_change_factor * min_change_factor, steer_change_factor + outside_dampener * fabs(tRInverse[target_raceline][This])/10);
            }
            steer_change = (MAX(0.05, 0.4 - MAX(0.0, (car->_speed_x-50) / 100)) * steer_change_factor);
            steer_change *= MIN(1.0, transition_time);
            if (target_steer > steer2)
                data->racesteer = MIN(target_steer, steer2 + steer_change);
            else
                data->racesteer = MAX(target_steer, steer2 - steer_change);
            data->racesteer = MIN(last_steer + steer_change * 2, MAX(last_steer - steer_change*2, data->racesteer));
//fprintf(stderr, "%d TRANSITION to %s: s2=%.3f scf=%.3f sc=%.3f racesteer=%.3f\n",Next,(target_raceline==LINE_LEFT?"left":(target_raceline==LINE_RIGHT?"right":(target_raceline==LINE_MID?"mid":"rl"))),steer2,steer_change_factor,steer_change,data->racesteer);fflush(stderr);
        }

//fprintf(stderr, "%d->%d hold=%d rI=%.3f ang=%.3f LAST: lft=%.3f rgt=%.3f rl=%.3f src=%.3f tgt=%.3f lst=%.3f prc=%.3f inc=%.3f  NOW: lft=%.3f rgt=%.3f rl=%.3f src=%.3f tgt=%.3f steer=%.3f\n", source_raceline,target_raceline,hold_the_line,tRInverse[LINE_RL][Next],data->angle,last_left_steer,last_right_steer,last_rl_steer,last_source_steer,last_target_steer,last_steer,steer_trans_percentage,transition_increment,left_steer,right_steer,rl_steer,source_steer,target_steer,data->racesteer); fflush(stderr);

        //if ((steer_trans_percentage < 1.0 && steer_trans_percentage > 0.0) || fabs(data->racesteer) > 0.6)
        double nearfinalsteer = data->racesteer;
{
char buf[16], buf2[16];
fprintf(stderr, "%d %d [%d:%s <- %s] hold=%d out=%d lft=%.4f rgt=%.4f rl=%.4f last=%.4f trackpos=%.4f proj=%.4f str1=%.4f diff=%.2f str2=%.4f str_chg=%.4f newsteer=%.4f finalsteer=%.4f\n", This, Next, !data->linemode->is_transitioning, (target_raceline==LINE_LEFT?"left":(target_raceline==LINE_RIGHT?"right":(target_raceline==LINE_MID?"mid":(target_raceline>=LINE_RL?"rl":itoa(target_raceline,buf,10))))), (source_raceline==LINE_LEFT?"left":(source_raceline==LINE_RIGHT?"right":(source_raceline==LINE_MID?"mid":(source_raceline>=LINE_RL?"rl":itoa(source_raceline,buf2,10))))), hold_the_line,outsteer,left_steer,right_steer,rl_steer,last_steer,trackpos_steer,projected_steer,steer1,steer_diff,steer2,steer_change,nearfinalsteer,data->racesteer);fflush(stderr);
}

        // calculate the speed based on track position
        track_percentage = MIN(1.0, MAX(0.0, track_percentage * 1.0 - (fabs(tRInverse[LINE_RL][This])*100)));
        data->speed = data->left_speed + (data->right_speed - data->left_speed) * track_percentage;
        if (outsteer)
        {
            data->speed -= MIN(4.0, fabs(tRInverse[target_raceline][This])*200*outsteerSpeedReducer);
            target_speed = data->speed;
            /*
            if (tRInverse[target_raceline][This] < 0.0)
                data->left_speed = target_speed = data->left_speed_outsteer;
            else
                data->right_speed = target_speed = data->right_speed_outsteer;
                */
        }
        if (target_raceline == LINE_RL && fabs(target_steer) < 0.5)
        {
            double steer_percentage = 1.0 - fabs(data->racesteer - target_steer) / 2.0;
            if (source_raceline == LINE_LEFT)
                data->speed = data->left_speed + (target_speed - data->left_speed) * steer_percentage;
            else if (source_raceline == LINE_RIGHT)
                data->speed = data->right_speed + (target_speed - data->right_speed) * steer_percentage;
            else
                data->speed = rl_speed + (target_speed - rl_speed) * steer_percentage;
        }
        if (data->speed > car->_speed_x)
        {
            // don't accelerate if car is skidding as it'll cause a spinout
            double skidAng = cos(atan2(car->_speed_Y, car->_speed_X) - car->_yaw);
            data->speed = MAX(car->_speed_x - 0.0, data->speed * (skidAng * skidAng));
        }
    }
    else
    {
        // we're on a raceline (not transitioning)
        last_steer_diff = 1000.0;
        data->racesteer = target_steer;
        if (target_raceline == LINE_LEFT)
            data->speed = data->left_speed;
        else if (target_raceline == LINE_RIGHT)
            data->speed = data->right_speed;
        else if (target_raceline == LINE_MID)
            data->speed = mid_speed;
    }

    if (data->s->_raceType == RM_TYPE_RACE && target_raceline >= LINE_RL && data->s->currentTime < 5.0)
        data->racesteer = steer1;

    steer_diff = target_steer - data->racesteer;
    last_steer_diff = steer_diff;

    if (data->s->_raceType == RM_TYPE_PRACTICE)
    //if (fabs(data->angle) > 0.9)
    {
        // print out a bunch of spam to the commandline
#if 1
        char buf[16];
        int apex;
        int prefer_side = findNextCorner(car, This, &apex);
        double y = car->_speed_Y;
        NORM_PI_PI(y);
        fprintf(stderr, "%d %d [%d:%s] prefer=%s y=%.2f fuel=%.2f spd=%.2f->%.2f accel=%.2f steer=%.3f lft=%.3f rgt=%.3f tgt=%.3f crt=%.3f | fromStart=%.1f ri=%.4f yaw=%.2f/%.2f err=%.2f\n", This, Next, !data->linemode->is_transitioning, (target_raceline==LINE_LEFT?"left":(target_raceline==LINE_RIGHT?"right":(target_raceline==LINE_MID?"mid":(target_raceline>=LINE_RL?"rl":itoa(target_raceline,buf,10))))), (prefer_side==TR_LFT?"LFT":(prefer_side==TR_RGT?"RGT":"STR")), y, car->_fuel/car->_tank,car->_speed_x, data->speed, car->_accel_x, data->racesteer,left_steer,right_steer,target_steer,current_steer,car->_distFromStartLine,tRInverse[LINE_RL][This],car->_yaw,car->_yaw_rate,data->error);
#endif
        fflush(stderr);
    }

    // remember values for next time...
    last_left_steer = left_steer;
    last_right_steer = right_steer;
    last_rl_steer = rl_steer;
    last_last_steer = last_steer;
    last_steer = data->racesteer;

    if (data->linemode->is_transitioning && car->_speed_x <= rl_speed)
    {
        // check to see if we're still transitioning...
        data->linemode->checkTransition(data->racesteer, data->laststeer);
    }
#endif
}


void LRaceLine::SteerTheCar(RaceLineDriveData *data, int target_raceline)
{
    int rl = MIN(LINE_RL, target_raceline);
    //int target_raceline = data->linemode->getTargetLine();

    // 
    // Find index in data arrays
    //
    /*====================================================*/
    /* Display Index used in the data file */
    double X, Y;
    tTrackSeg *seg = car->_trkPos.seg;
    int Index = DivIndex(data, seg, rl, &X, &Y);
    int diff = Next - Index;
    if (diff < 0)
        diff = (Next + Divs) - Index;
    NextNextNext = (Next + diff) % Divs;

    if (LineIndex)
        GfOut("index: %d\n", Index);
    /*====================================================*/


#if 0
    double dx = X4 - car->_pos_X;
    double dy = Y4 - car->_pos_Y;
    *lookahead = sqrt(dx*dx + dy*dy);
#endif

    double c0, TargetCurvature, transition_percentage = 0.0;
    bool transition_required = false;

    {
        c0 = CalculateSpeed(data, X, Y, Index, target_raceline);
        data->raceoffset = (float)CalculateOffset(rl);
        TargetCurvature = CalculateCurvature(c0, Index, rl);
    }

    //
    // Steering control
    //
    double Error = 0;
    double VnError = 0;
    double carspeed = Mag(car->_speed_X, car->_speed_Y);


    //
    // Ideal value
    //
    double steer = atan(wheelbase * TargetCurvature) / car->_steerLock;

    if (0 && steer_verbose)
        GfOut("Steer A:%f", steer);

    //
    // Servo system to stay on the pre-computed path
    //
    int Prev = (Index + Divs - 1) % Divs;
    int NextNext = (Next + 1) % Divs;

    {
        double dx = tx[rl][Next] - tx[rl][Index];
        double dy = ty[rl][Next] - ty[rl][Index];
        Error = (dx * (Y - ty[rl][Index]) - dy * (X - tx[rl][Index])) / Mag(dx, dy);

        double Prevdx = tx[rl][Next] - tx[rl][Prev];
        double Prevdy = ty[rl][Next] - ty[rl][Prev];
        double Nextdx = tx[rl][NextNext] - tx[rl][Index];
        double Nextdy = ty[rl][NextNext] - ty[rl][Index];
        dx = c0 * Prevdx + (1 - c0) * Nextdx;
        dy = c0 * Prevdy + (1 - c0) * Nextdy;
        double n = Mag(dx, dy);
        dx /= n;
        dy /= n;
        double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);
        double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);

        VnError = asin(sError);

        VnError = MAX((PI / -2.0), MIN(VnError, (PI / 2.0)));

        if (cError < 0)
        {
            VnError = PI - VnError;
            //VnError = (2.0 *PI) - VnError;
            //VnError = -1.0 * VnError;
        }

        NORM_PI_PI(VnError);

        data->error = VnError;
    }

    // Correcting Steer:
    steer -= ((atan(Error * (300.0 / (carspeed + 300.0)) / 15.0) + VnError) * errorCorrectionFactor) / car->_steerLock;

    if (steer_verbose)
        GfOut("steer: B:%f\n", steer);

    //
    // Steer into the skid
    //
    double vx = car->_speed_X;
    double vy = car->_speed_Y;
    double dirx = cos(car->_yaw);
    double diry = sin(car->_yaw);
    double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
    if (Skid > 0.9)
        Skid = 0.9;
    if (Skid < -0.9)
        Skid = -0.9;
    steer += (asin(Skid) / car->_steerLock) * steerSkidFactor;//0.9;

    if (0 && steer_verbose)
        GfOut(" C:%f", steer);

    double yr = carspeed * TargetCurvature;
    double yawdiff = car->_yaw_rate - yr;
    steer -= (0.08 * (100 / (carspeed + 100)) * yawdiff) / car->_steerLock;
    data->racesteer = (float)steer;
}

double LRaceLine::SteerTheCar(double lane, double deltaTime)
{
#if 1
    double TargetCurvature, LeftCurvature, RightCurvature;
    bool transition_required = false;

    lane = MAX(tLane[LINE_RL][Next], MIN(tLane[LINE_RL][Next], lane));

    double X = car->_pos_X + car->_speed_X * Time * 3.5;
    double Y = car->_pos_Y + car->_speed_Y * Time * 3.5;
    double c0 = (tx[LINE_LEFT][Next] - tx[LINE_LEFT][This]) * (tx[LINE_LEFT][Next] - X) +
        (ty[LINE_LEFT][Next] - ty[LINE_LEFT][This]) * (ty[LINE_LEFT][Next] - Y);
    double c1 = (tx[LINE_LEFT][Next] - tx[LINE_LEFT][This]) * (X - tx[LINE_LEFT][This]) +
        (ty[LINE_LEFT][Next] - ty[LINE_LEFT][This]) * (Y - ty[LINE_LEFT][This]);
    double sum = c0 + c1;
    c0 /= sum;
    LeftCurvature = CalculateCurvature(c0, This, LINE_LEFT);

    c0 = (tx[LINE_RIGHT][Next] - tx[LINE_RIGHT][This]) * (tx[LINE_RIGHT][Next] - X) +
        (ty[LINE_RIGHT][Next] - ty[LINE_RIGHT][This]) * (ty[LINE_RIGHT][Next] - Y);
    c1 = (tx[LINE_RIGHT][Next] - tx[LINE_RIGHT][This]) * (X - tx[LINE_RIGHT][This]) +
        (ty[LINE_RIGHT][Next] - ty[LINE_RIGHT][This]) * (Y - ty[LINE_RIGHT][This]);
    sum = c0 + c1;
    c0 /= sum;
    RightCurvature = CalculateCurvature(c0, This, LINE_RIGHT);

    double tlane = (lane - tLane[LINE_LEFT][Next]) / tLane[LINE_RIGHT][Next];
    if (lane < tLane[LINE_LEFT][Next])
        TargetCurvature = LeftCurvature;
    else if (lane > tLane[LINE_RIGHT][Next])
        TargetCurvature = RightCurvature;
    else
        TargetCurvature = LeftCurvature + (RightCurvature - LeftCurvature) * tlane;

    //
    // Steering control
    //
    double Error = 0;
    double VnError = 0;
    double carspeed = Mag(car->_speed_X, car->_speed_Y);

    //
    // Ideal value
    //
    double steer = atan(wheelbase * TargetCurvature) / car->_steerLock;

    //
    // Servo system to stay on the pre-computed path
    //
    int Prev = (This + Divs - 1) % Divs;
    int NextNext = (Next + 1) % Divs;

    {
        double txThis = txLeft[LINE_RL][This] + (txRight[LINE_RL][This] - txLeft[LINE_RL][This]) * lane;
        double tyThis = tyLeft[LINE_RL][This] + (tyRight[LINE_RL][This] - tyLeft[LINE_RL][This]) * lane;
        double txPrev = txLeft[LINE_RL][Prev] + (txRight[LINE_RL][Prev] - txLeft[LINE_RL][Prev]) * lane;
        double tyPrev = tyLeft[LINE_RL][Prev] + (tyRight[LINE_RL][Prev] - tyLeft[LINE_RL][Prev]) * lane;
        double txNext = txLeft[LINE_RL][Next] + (txRight[LINE_RL][Next] - txLeft[LINE_RL][Next]) * lane;
        double tyNext = tyLeft[LINE_RL][Next] + (tyRight[LINE_RL][Next] - tyLeft[LINE_RL][Next]) * lane;
        double txNextNext = txLeft[LINE_RL][NextNext] + (txRight[LINE_RL][NextNext] - txLeft[LINE_RL][NextNext]) * lane;
        double tyNextNext = tyLeft[LINE_RL][NextNext] + (tyRight[LINE_RL][NextNext] - tyLeft[LINE_RL][NextNext]) * lane;
        double dx = txNext - txThis;
        double dy = tyNext - tyThis;
        Error = (dx * (Y - tyThis) - dy * (X - txThis)) / Mag(dx, dy);

        double Prevdx = txNext - txPrev;
        double Prevdy = tyNext - tyPrev;
        double Nextdx = txNextNext - txThis;
        double Nextdy = tyNextNext - tyThis;
        dx = c0 * Prevdx + (1 - c0) * Nextdx;
        dy = c0 * Prevdy + (1 - c0) * Nextdy;
        double n = Mag(dx, dy);
        dx /= n;
        dy /= n;
        double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);
        double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (carspeed == 0.0 ? carspeed+0.01 : carspeed);

        VnError = asin(sError);
        VnError = MAX((PI / -2.0), MIN(VnError, (PI / 2.0)));
        if (cError < 0)
            VnError = PI - VnError;
        NORM_PI_PI(VnError);
    }

    // Correcting Steer:
    steer -= (atan(Error * (300.0 / (carspeed + 300.0)) / 15.0) + VnError) / car->_steerLock;

    //
    // Steer into the skid
    //
    double vx = car->_speed_X;
    double vy = car->_speed_Y;
    double dirx = cos(car->_yaw);
    double diry = sin(car->_yaw);
    double Skid = (dirx * vy - vx * diry) / (carspeed == 0.0 ? carspeed+0.1 : carspeed);
    Skid = MAX(-0.9, MIN(0.9, Skid));
    steer += (asin(Skid) / car->_steerLock) * 0.2;

    double yr = carspeed * TargetCurvature;
    double yawdiff = car->_yaw_rate - yr;
    steer -= (0.08 * (100 / (carspeed + 100)) * yawdiff) / car->_steerLock;
    return steer;
#endif
    return 0.0;
}

/**********************************************************/
double LRaceLine::getSlowestSpeedForDistance(double distance, int raceline, int *slowdiv)
{
    double dist = car->_trkPos.toStart + distance;
    int distancediv = (distance > 0.0 ? DivIndexForCarDistance(car, dist) : This);
    int div = This;
    double minspeed = 100000;
    double dist_covered = 0.0;

    do {
        if (tSpeed[raceline][div] < minspeed)
        {
            minspeed = tSpeed[raceline][div];
            if (slowdiv)
                *slowdiv = div;
        }
        int ldiv = div;
        div = (div + 1) % Divs;
        dist_covered += Mag(tx[raceline][div]-tx[raceline][ldiv], ty[raceline][div]-ty[raceline][ldiv]);
        if (dist_covered >= distance) 
            break;
    } while (div != This);

    return minspeed;
}

int LRaceLine::isOnLine(int line)
{
    double lane2left = tLane[line][Next] * Width;

    double pos2middle = fabs(car->_trkPos.toLeft - lane2left);

    if (steer_verbose)
        fprintf(stderr, "Lane ToLeft:%.3f, Car ToLeft:%.3f, Car ToMiddle:%.3f\n", lane2left, car->_trkPos.toLeft, car->_trkPos.toMiddle);

    if (pos2middle < MAX(0.1, 1.0 - (car->_speed_x * (car->_speed_x / 10)) / 600))
    {
        if (steer_verbose)
            fprintf(stderr, "%s is OnLine: PosToMiddle: %.3f \n", car->_name, pos2middle);
        return 1;
    }

    return 0;
}

/**********************************************************/
void LRaceLine::GetPoint(float offset, float lookahead, vec2f *rt)
{
#if 0
    double lane = (Width / 2 - offset) / Width;
#if 0
    int ndiv = (Next + 1 + int((lookahead)/DivLength)) % Divs;
    rt->x = (float) (lane * txRight[ndiv] + (1 - lane) * txLeft[lane][ndiv]);
    rt->y = (float) (lane * tyRight[ndiv] + (1 - lane) * tyLeft[lane][ndiv]);
#endif
    double length = 0.0;
    //double la = (double) lookahead * MAX(0.8, car->_speed_x/TargetSpeed); //0.8;
    double la = (double)lookahead * MIN(1.0, MAX(0.8, car->_speed_x / TargetSpeed)); //0.8;
    vec2f last;
    last.x = (float)(lane * txRight[lane][This] + (1 - lane) * txLeft[lane][This]);
    last.y = (float)(lane * tyRight[lane][This] + (1 - lane) * tyLeft[lane][This]);
    //last.x = car->_pos_X;
    //last.y = car->_pos_Y;
    //int ndiv = (Next + 1 + int((lookahead)/DivLength)) % Divs;
    int ndiv = Next, count = 0;
    while (length < la && count < (int)(la / DivLength))
    {
        rt->x = (float)(lane * txRight[lane][ndiv] + (1 - lane) * txLeft[lane][ndiv]);
        rt->y = (float)(lane * tyRight[lane][ndiv] + (1 - lane) * tyLeft[lane][ndiv]);
        double dx = rt->x - last.x;
        double dy = rt->y - last.y;
        double thislength = sqrt(dx*dx + dy*dy);
        length += thislength;
        ndiv = (ndiv + 1) % Divs;
        count++;
        last.x = rt->x;
        last.y = rt->y;
    }
#endif
}

/**********************************************************/
double LRaceLine::correctLimit(int line)
{
    // this returns true if we're approaching a corner & are significantly
    // inside the ideal racing line. The idea is to prevent a sudden outwards
    // movement at a time when we should be looking to turn in.

    double nlane2left = tLane[line][Next] * Width;
    if ((tRInverse[line][Next] > 0.001 && car->_trkPos.toLeft < nlane2left - 2.0) ||
        (tRInverse[line][Next] < -0.001 && car->_trkPos.toLeft > nlane2left + 2.0))
        //return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * 80.0));
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * 100.0));

    int nnext = (Next + (int)(car->_speed_x / 3)) % Divs;
    double nnlane2left = tLane[line][nnext] * Width;
    if ((tRInverse[line][nnext] > 0.001 && car->_trkPos.toLeft < nnlane2left - 2.0) ||
        (tRInverse[line][nnext] < -0.001 && car->_trkPos.toLeft > nnlane2left + 2.0))
        //return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * 40.0));
        return MAX(0.3, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * 40.0));

    /* OK, we're not inside the racing line! */
#if 0
    // Check and see if we're significantly outside it and turning a corner, 
    // in which case we don't want to correct too much either.
    if ((tRInverse[line][Next] > 0.001 && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][Next] < -0.001 && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][Next]) * (car->_speed_x-40.0)*4));

    if ((tRInverse[line][nnext] > 0.001 && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][nnext] < -0.001 && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(0.2, MIN(1.0, 1.0 - fabs(tRInverse[line][nnext]) * (car->_speed_x-40.0)*2));
#endif
    // Check and see if we're outside it and turning into a corner,
    //  in which case we want to correct more to try and get closer to the apex.
    if ((tRInverse[line][Next] > 0.001 && tLane[Next] <= tLane[This] && car->_trkPos.toLeft > nlane2left + 2.0) ||
        (tRInverse[line][Next] < -0.001 && tLane[Next] >= tLane[This] && car->_trkPos.toLeft < nlane2left - 2.0))
        return MAX(1.0, MIN(1.5, 1.0 + fabs(tRInverse[line][Next])));

    return 1.0;
}

/**********************************************************/
double LRaceLine::getAvoidSpeed(float distance1, float distance2)
{
    int i;
    //double speed1 = 1000.0, speed2 = 1000.0, speed3 = 1000.0;
    double speed1 = 1000.0, speed2 = 1000.0;

    int count = 0;
    for (i = Next; count < (int)(distance1 / DivLength); i++)
    {
        count++;
        i = (i % Divs);
        speed1 = MIN(speed1, (tSpeed[LINE_LEFT][i] + tSpeed[LINE_RIGHT][i])/2);
    }

    count = 0;
    distance2 = (MIN(distance2, distance1 * 3) - distance1) / DivLength;
    for (; count < (int)distance2; i++)
    {
        count++;
        i = (i % Divs);
        speed2 = MIN(speed2, (tSpeed[LINE_LEFT][i] + tSpeed[LINE_RIGHT][i])/2 + (double)count * 0.25);
    }

    return MIN(speed1, speed2);
}

/********************************************************************/
/*                                                                  */
/*            STORE K1999 DATA                            */
/*        Raceline K1999 (aka Andrew's Locus)                 */
/*                                                                  */
/********************************************************************/
/* Set "savedata" to 1 in the xml file
               Not used in race, just for look at */

// saved in your Local directory .torcs/drivers/[module_name]/tracks/[track_name].data

//void LRaceLine::storeData( void **carParmHandle )
void LRaceLine::StoreData(tTrack* t)
{
    int rlLine = 0;


    rlLine = (int) GfParmGetNum(car->_carHandle, SECT_PRIVATE, (char*)"savedata", (char*)NULL, 0);

    if (rlLine)
    {
        WriteLine(t);
        WriteTrack(t);
    }
    else
        return;

}

/**********************************************************/
void LRaceLine::WriteLine(tTrack* track)
{

    FILE *tfile = NULL;
    int i;
    int rl = LINE_RL;

    char filename[256];
    sprintf(filename, "%sdrivers/%s/tracks/%s_line.data", GetLocalDir(), myModuleName, track->internalname);

    //tfile = fopen( "/home/gamer/TRB/torcs_data/hymie_2015/tracks/wheel-2.learn", "w");

    tfile = fopen(filename, "w");

    if (tfile == NULL)
        return;

    fprintf(tfile, "#==========================================================================================\n");
    fprintf(tfile, "# K1999 RaceLine data\n");
    fprintf(tfile, "# i tDistance[i] tLane[rl][i] tx[rl][i] ty[rl][i] tRInverse[rl][i] tSpeed[rl][i]\n");
    fprintf(tfile, "# Divs:%d\n", Divs);
    for (i = 0; i < Divs; i++)
    {
        fprintf(tfile, "%d %f %f %f %f %f %f\n",
            i,
            tDistance[i],
            tLane[rl][i],
            tx[rl][i], ty[rl][i],
            tRInverse[rl][i],
            tSpeed[rl][i]
            );
    }
    fprintf(tfile, "#End\n");
    fprintf(tfile, "#===============\n");
    fprintf(tfile, "\n");

    fflush(tfile);
    fclose(tfile);

}

/**********************************************************/

void LRaceLine::WriteTrack(tTrack* track)
{
    /*
    FILE *tfile = NULL;
    int i;

    char filename[256];
    sprintf(filename, "%sdrivers/%s/tracks/%s_track.data", GetLocalDir(), myModuleName, track->internalname);
    tfile = fopen(filename, "w");

    if (tfile == NULL)
        return;

    fprintf(tfile, "#=============================================================================\n");
    fprintf(tfile, "# K1999 Track data\n");
    fprintf(tfile, "# i tDistance[i] txLeft[i] tyLeft[i] txRight[i] tyRight[i] tFriction[i]\n");
    fprintf(tfile, "# Divs:%d\n", Divs);

    for (i = 0; i < Divs; i++)
    {
        fprintf(tfile, "%d %f %f %f %f %f %f\n",
            i,
            tDistance[i],
            txLeft[i], tyLeft[i],
            txRight[i], tyRight[i],
            tFriction[i]
            );
    }

    fprintf(tfile, "#Done\n");
    fprintf(tfile, "#=================================\n");
    fprintf(tfile, "\n");

    fflush(tfile);
    fclose(tfile);

    */
}

/*==========================================================*/

void LRaceLine::removeNewLineCharacters(char *text)
{
    char *p = text + (strlen(text)-1);
    while (p >= text && (*p == 13 || *p == 10 || *p == ' ' || *p == '\t'))
    {
        *p = 0;
        p--;
    }
}

double LRaceLine::readDouble(FILE *fp)
{
    if (!fp) return -1000.0;
    char buffer[257];
    if (!fgets(buffer, 256, fp)) return -1000.0;

    removeNewLineCharacters(buffer);
    return atof(buffer);
}

int LRaceLine::readInt(FILE *fp)
{
    if (!fp) return -1000;
    char buffer[257];
    if (!fgets(buffer, 256, fp)) return -1000;

    removeNewLineCharacters(buffer);
    return atoi(buffer);
}

bool LRaceLine::LoadTrack(tTrack *track, tSituation *s)
{
    int i, rl;
    char fileName[257];

    snprintf(fileName, 256, "%sdrivers/%s/data/%s_%s.dat", GetDataDir(), myModuleName, track->internalname, (s->_raceType == RM_TYPE_QUALIF ? "qual" : "race"));
    FILE *fp = fopen(fileName, "r");
    if (!fp)
        return false;

    char buffer[257];

    // absolutely no error checking here - I must be nuts.
    Divs = readInt(fp);
    DivLength = readInt(fp);
    Segs = readInt(fp);

    for (rl = LINE_MID; rl <= LINE_RL; rl++)
    {
        fgets(buffer, 256, fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tx[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            ty[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tz[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tzd[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tLane[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tRInverse[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tSpeed[rl][i] = readDouble(fp);
        fgets(buffer, 256, fp);
        for (i=0; i<=Divs; i++)
            tDivSeg[rl][i] = readInt(fp);
    }

    fgets(buffer, 256, fp);
    /*
    for (i=0; i<=Divs; i++)
        txLeft[rl][i] = readDouble(fp);
    fgets(buffer, 256, fp);
    for (i=0; i<=Divs; i++)
        txRight[rl][i] = readDouble(fp);
    fgets(buffer, 256, fp);
    */
    for (i=0; i<Segs; i++)
        tSegDist[i] = readDouble(fp);
    fgets(buffer, 256, fp);
    for (i=0; i<Segs; i++)
        tSegIndex[i] = readInt(fp);
    fgets(buffer, 256, fp);
    for (i=0; i<Segs; i++)
        tElemLength[i] = readDouble(fp);

    fclose(fp);
    return true;
}

void LRaceLine::SaveTrack(tTrack *track, tSituation *s)
{
    char fileName[257];
    snprintf(fileName, 256, "%sdrivers/%s/data/%s_%s.dat", GetDataDir(), myModuleName, track->internalname, (s->_raceType == RM_TYPE_QUALIF ? "qual" : "race"));

    FILE *fp = fopen(fileName, "w");
    if (!fp)
        return;

    fprintf(fp, "%d\n", Divs);
    fprintf(fp, "%d\n", DivLength);
    fprintf(fp, "%d\n", Segs);

    char *rlName[] = {"LINE_MID", "LINE_LEFT", "LINE_RIGHT", "LINE_RL_SLOW", "LINE_RL_MID", "LINE_RL"};

    int rl, i;

    for (rl = LINE_MID; rl <= LINE_RL; rl++)
    {
        fprintf(fp, "%s\n", rlName[rl]);
        fprintf(fp, "tx %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tx[rl][i]);
        fprintf(fp, "%.10f\n", (tx[rl][0] + tx[rl][i-1]) / 2);
        fprintf(fp, "ty %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", ty[rl][i]);
        fprintf(fp, "%.10f\n", (ty[rl][0] + ty[rl][i-1]) / 2);
        fprintf(fp, "tz %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tz[rl][i]);
        fprintf(fp, "%.10f\n", (tz[rl][0] + tz[rl][i-1]) / 2);
        fprintf(fp, "tzd %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tzd[rl][i]);
        fprintf(fp, "%.10f\n", (tzd[rl][0] + tzd[rl][i-1]) / 2);
        fprintf(fp, "tLane %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tLane[rl][i]);
        fprintf(fp, "%.10f\n", (tLane[rl][0] + tLane[rl][i-1]) / 2);
        fprintf(fp, "tRInverse %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tRInverse[rl][i]);
        fprintf(fp, "%.10f\n", (tRInverse[rl][0] + tRInverse[rl][i-1]) / 2);
        fprintf(fp, "tSpeed %s\n", rlName[rl]);
        for (i=0; i<Divs; i++)
            fprintf(fp, "%.10f\n", tSpeed[rl][i]);
        fprintf(fp, "%.10f\n", (tSpeed[rl][0] + tSpeed[rl][i-1]) / 2);
        fprintf(fp, "tDivSeg %s\n", rlName[rl]);
        for (i=0; i<=Divs; i++)
            fprintf(fp, "%d\n", tDivSeg[rl][i]);
    }
    /*
    fprintf(fp, "txLeft\n");
    for (i=0; i<Divs; i++)
        fprintf(fp, "%.10f\n", txLeft[rl][i]);
    fprintf(fp, "%.10f\n", (txLeft[rl][0] + txLeft[i-1]) / 2);
    fprintf(fp, "txRight\n");
    for (i=0; i<Divs; i++)
        fprintf(fp, "%.10f\n", txRight[rl][i]);
    fprintf(fp, "%.10f\n", (txRight[rl][0] + txRight[i-1]) / 2);
    */
    fprintf(fp, "tSegDist\n");
    for (i=0; i<Segs; i++)
        fprintf(fp, "%.10f\n", tSegDist[i]);
    fprintf(fp, "tSegIndex\n");
    for (i=0; i<Segs; i++)
        fprintf(fp, "%d\n", tSegIndex[i]);
    fprintf(fp, "tElemLength\n");
    for (i=0; i<Segs; i++)
        fprintf(fp, "%.10f\n", tElemLength[i]);

    fclose(fp);
}
