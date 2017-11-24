/***************************************************************************

    file                 : opponent.cpp
    created              : Thu Apr 22 01:20:19 CET 2003
    copyright            : (C) 2003-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: opponent.cpp,v 1.9 2006/03/06 22:43:50 berniw Exp $

    ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

//#define OPP_DEBUG
//#include "quadratic.h"
#include "opponent.h"
#include "Vec2d.h"
#include "vardef.h"

// class variables and constants.
tTrack* Opponent::track;
const float Opponent::FRONTCOLLDIST = 200.0f;            // [m] distance to check other cars in front me.
const float Opponent::BACKCOLLDIST = 7.0f;            // [m] distance to check other cars behind me.

const float Opponent::EXACT_DIST = 12.0f;            // [m] if the estimated distance is smaller, compute it more accurate
const float Opponent::LAP_BACK_TIME_PENALTY = -30.0f;        // [s]
const float Opponent::OVERLAP_WAIT_TIME = 5.0f;            // [s] overlaptimer must reach this time before we let the opponent pass.
const float Opponent::SPEED_PASS_MARGIN = 5.0f;            // [m/s] avoid overlapping opponents to stuck behind me.
const int Opponent::MAX_DAMAGE_DIFF = 1000;

Opponent::Opponent()
{
    t_impact = 0.0f;
    toLeft = 0.0f;
    prevToLeft = -200.0f;
    brake_overtake_filter = 1.0f;
    teammate = false;
    team = -1;
    FRONTCOLL_MARGIN = 0.0;            // [m] front safety margin.
    SIDECOLL_MARGIN = 3.0;            // [m] side safety margin.
    brake_multiplier = 0.3;
    brake_warn_multiplier = 0.3;
    relativeangle = 0.0f;
    distance = distance2 = distance3 = trueSpeedDiff = 0.0;
    prev_speed_X = prev_speed_Y = d_prev_speed_X = d_prev_speed_Y = 0.0;
    average_AX = average_AY = collspeed = 0.0;
    for (int i=0; i<4; i++)
        speedAngle[i] = 0;
    deltamult = deltaTime = speedDelta = prevSpeed = 0;
}



void Opponent::update(tSituation *s, Driver *driver)
{
    bool teamMate = false;
    //tCarElt *mycar = driver->getCarPtr();
    mycar = driver->getCarPtr();
    teamMate = !strncmp(car->_teamname, mycar->_teamname, 12);

    toLeft = car->_trkPos.toLeft;
    if (prevToLeft < -100.0f)
        avgLateralMovt = 0;
    else
        avgLateralMovt = avgLateralMovt*0.75 + (toLeft-prevToLeft)*0.25;
    prevToLeft = car->_trkPos.toLeft;

    if (team == -1)
    {
        deltamult = (float)(1.0 / s->deltaTime);
        deltaTime = s->deltaTime;
    }

    if (teamMate && team == -1) {
        team = TEAM_FRIEND;
    }
    else if (!teamMate) {
        team = TEAM_FOE;
    }

    // Set state
    initState();
    // If the car is out of the simulation ignore it.
    //if (car->_state & (RM_CAR_STATE_NO_SIMU & ~RM_CAR_STATE_PIT)) 
    if (car->_state & RM_CAR_STATE_NO_SIMU)
    {
        return;
    }

    /* Is opponent in relevant range */

    calcDist();
    calcSpeed();
    oppSpeed = (float)speed;
    mySpeed = driver->getSpeed();

    //calcState(driver->getSpeed());
    calcState(mySpeed, driver);

    // Opponent is faster mycar
    if (oppSpeed > mySpeed + SPEED_PASS_MARGIN) {
        oppFaster = true;
    }

    // Opponent is behind mycar
    if (distance <= -BACKCOLLDIST) {
        oppBehind = true;
    }
    // Check if we should let overtake the opponent.
    updateOverlapTimer(s, mycar);
    if (oppFaster && oppBehind && overlaptimer > OVERLAP_WAIT_TIME && car->_pos < mycar->_pos) {
        state = OPP_LETPASS;
    }

    double dt = s->deltaTime;
    brake_overtake_filter *= (float) exp(-dt * .5);

    brakedistance = (float)(distance - car->_dimension_x);

    prev_speed_X = car->_speed_X;
    prev_speed_Y = car->_speed_Y;
    d_prev_speed_X = mycar->_speed_X;
    d_prev_speed_Y = mycar->_speed_Y;
}

void Opponent::initState()
{
    distance = DBL_MAX;
    oppBehind = false;
    //oppFront = false;
    //oppSlower = false;
    oppFaster = false;
    oppLetPass = false;
    oppFaraway = false;
    state = OPP_IGNORE;
}


// Updating distance along the middle.
void Opponent::calcDist()
{
    distance3 = distance2;
    distance2 = distance;
    distance = car->_distFromStartLine - mycar->_distFromStartLine;
    //distance = (car->_trkPos.seg->lgfromstart + getDistToSegStart(car)) - (mycar->_trkPos.seg->lgfromstart + getDistToSegStart(mycar));
    //float oppToStart = car->_trkPos.seg->lgfromstart + getDistToSegStart();
    //distance = oppToStart - mycar->_distFromStartLine;

    if (distance > track->length / 2.0f) {
        distance -= track->length;
    }
    else if (distance < -track->length / 2.0f) {
        distance += track->length;
    }

    trueSpeedDiff = (distance3 - distance) * deltamult / 2;

    speedangle = (float)-(cardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X));
    NORM_PI_PI(speedangle);
}


// Compute the length to the start of the segment.
float Opponent::getDistToSegStart(tCarElt *theCar)
{
    if (theCar->_trkPos.seg->type == TR_STR) {
        return theCar->_trkPos.toStart;
    }
    else {
        return theCar->_trkPos.toStart*theCar->_trkPos.seg->radius;
    }
}


void Opponent::calcSpeed()
{
    relativeangle = car->_yaw - mycar->_yaw;
    NORM_PI_PI(relativeangle);
    if (fabs(distance) < 20.0) {
        if (fabs(relativeangle) > 0.5) {
            speed = getSpeedAngle(mycar->_yaw);
        }
        else {
            speed = car->_speed_x;
        }
    }
    else {
        speed = getSpeedAngle(RtTrackSideTgAngleL(&(car->_trkPos)));
    }

    speedDelta = speedDelta * 0.75 + (car->_speed_x - prevSpeed)*0.25;
    prevSpeed = car->_speed_x;

    average_AX = average_AX * 0.75 + car->pub.DynGCg.vel.x * 0.25;
    average_AY = average_AY * 0.75 + car->pub.DynGCg.vel.y * 0.25;

    {
        for (int j=3; j>0; j--)
            speedAngle[j] = speedAngle[j-1];
        double newx = car->_corner_x(FRNT_LFT) + car->_speed_X;
        double newy = car->_corner_y(FRNT_LFT) + car->_speed_Y;
        double dx = newx - car->_corner_x(FRNT_LFT);
        double dy = newy - car->_corner_y(FRNT_LFT);
        speedAngle[0] = atan2(dx, dy);
    }
}

double Opponent::getSpeedAngle(double angle)
{
    Vec2d speed, dir;
    speed.x = car->_speed_X;
    speed.y = car->_speed_Y;
    dir.x = cos(angle);
    dir.y = sin(angle);
    return speed * dir;
}


void Opponent::calcState(float Speed, Driver *driver)
{
    //float COLLDIST = MAX(car->_dimension_x, mycar->_dimension_x);
    float COLLDIST = car->_dimension_x + 0.1f;
    state = 0;

    if (distance > -BACKCOLLDIST && distance < FRONTCOLLDIST) 
    {

        // Is opponent aside.
        if (distance > -COLLDIST && distance < COLLDIST) 
        {
//fprintf(stderr, "%s SIDE\n",car->_name);fflush(stderr);

            sidedist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
            state = OPP_SIDE;
            if (distance > 0.0) // && Speed > oppSpeed)
            {
                double newdistance = getCornerDist();
                if (distance > car->_dimension_x * 0.3) // && newdistance < 1.0 && (Speed - oppSpeed) < 2.0)
                    state |= testQuadraticCollision(driver);
                distance = newdistance;
                t_impact = (float) MAX(0.0, MIN(1.5, distance / MAX(0.05, Speed - oppSpeed)));
                t_impact = (float)(distance / (Speed - oppSpeed));
                catchdist = (float)(Speed * distance / (Speed - oppSpeed));
                //int collision = testLinearCollision(driver, t_impact, (Speed-oppSpeed), catchdist, brake_multiplier);
            }
        }
        // Is opponent in front.
        else if (distance >= COLLDIST) 
        {
//fprintf(stderr, "%s FRONT\n",car->_name);fflush(stderr);
            state = OPP_FRONT;
            //if (testLinearCollision2(driver))
            state |= testQuadraticCollision(driver);
            //oppFront = true;
            // Is opponent slower.
            if (oppSpeed <= Speed) 
            {
                //oppSlower = true;
                if (team == TEAM_FRIEND && car->_dammage - MAX_DAMAGE_DIFF < mycar->_dammage)
                    state |= OPP_FRONT_FOLLOW;
                distance -= MAX(car->_dimension_x, mycar->_dimension_x);
                float colldistance = (float)(distance - FRONTCOLL_MARGIN);
                // If the distance is small we compute it more accurate.
                if (colldistance < EXACT_DIST) 
                {
                    distance = getCornerDist();
                    //if (mindist < distance) 
                    {
                        colldistance = (float)(distance - FRONTCOLL_MARGIN);
                    }
                }

                catchdist = (float)(Speed * colldistance / MAX(0.05, (Speed - oppSpeed)));

#if 1
                t_impact = (float) MAX(0.0, colldistance / MAX(0.05, (Speed - oppSpeed)));
                float cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
                sidedist = cardist;
                cardist = fabs(cardist) - fabs(getWidth() / 2.0f) - mycar->_dimension_y / 2.0f;
                double speed_diff = (Speed - oppSpeed);
                double brake_distance = driver->getBrakeCoefficient() * ((Speed+1.0) * (Speed+1.0));
                bool off_the_track = (fabs(car->_trkPos.toMiddle) > car->_trkPos.seg->width/2 + 3.0 && fabs(car->_trkPos.toMiddle - mycar->_trkPos.toMiddle) > 5.0);
#if 1
                //if (testQuadraticCollision(driver))
#else
                if (off_the_track)
                        state |= OPP_OFF_TRACK;
                else if (!off_the_track && brake_distance*2 > colldistance && (colldistance < 10.0 || (t_impact < 6.0 && t_impact > 0.0)))
                {
                    double impact_distance = t_impact * mycar->_speed_x;
                    double accel_diff = car->_accel_x - mycar->_accel_x;
                    //if (accel_diff < 0.0)
                    if (0)
                    {
                        speed_diff *= (1.0 - accel_diff*0.05);
                        impact_distance *= MAX(0.5, (1.0 + accel_diff*0.05));
                    }
                    int collision = 0;
                    int linear_collision = (speed_diff > 0.0 && brake_distance > impact_distance*0.9 ? testLinearCollision(driver, t_impact, speed_diff, catchdist, brake_multiplier) : 0);
                    if (linear_collision)
                        collision = (OPP_COLL | OPP_COLL_LINEAR);
                    else if (driver->isOnRaceline())
                    {
                        int raceline_conflict = 0;
                        if (impact_distance <= brake_distance)// / 2.0)
                            raceline_conflict = testRacelineCollision(driver, impact_distance, t_impact, driver->currentRaceline(), 1.0 + brake_warn_multiplier*2);
                        if (raceline_conflict)
                        {
                            if (raceline_conflict == 2 /*&& t_impact < 4.0*/ && impact_distance < brake_distance)
                                collision = OPP_COLL;
                            else
                               state |= OPP_RACELINE_CONFLICT;
                        }
                        //fprintf(stderr,"%s (%d:%d) raceline = %d/%d ti=%.2f d=%.2f id=%.2f bd=%.2f\n",car->_name,driver->isOnRaceline(),driver->currentRaceline(),raceline_conflict,collision, t_impact,distance,impact_distance,brake_distance); fflush(stderr);
                    }
                    if (!collision)
                    {
                        //if (testLinearCollision(driver, t_impact, speed_diff, catchdist, brake_warn_multiplier))
                        if (testQuadraticCollision(driver))
                            state |= OPP_RACELINE_CONFLICT;
                    }
                    else if (impact_distance < brake_distance / 2)
                        state |= OPP_COLL_URGENT;

                    state |= collision;

                    //fflush(stderr);

                }
#endif
#else
                if (catchdist < 5.0f || colldistance < 1.2)
                {
                    state = OPP_COLL;
                }
#endif
            }

            // Is opponent behind and faster.
        }
        else if (distance <= -COLLDIST && oppSpeed > Speed - SPEED_PASS_MARGIN) {
//fprintf(stderr, "%s BACK\n",car->_name);fflush(stderr);
            oppFaster = true;
            state = OPP_BACK;
            catchdist = (float)(Speed * distance / (oppSpeed - Speed));
            distance -= MAX(car->_dimension_x, mycar->_dimension_x);
            distance -= FRONTCOLL_MARGIN;

            // Is opponent in front and faster.
        }
        else if (distance >= COLLDIST && oppSpeed > Speed) {
//fprintf(stderr, "%s FRONT FAST\n",car->_name);fflush(stderr);
            oppFaster = true;
            state = OPP_FRONT_FAST;
            distance -= MAX(car->_dimension_x, mycar->_dimension_x);
            if (distance < EXACT_DIST) 
                distance = getCornerDist();
            if (team == TEAM_FRIEND && car->_dammage - MAX_DAMAGE_DIFF < mycar->_dammage) {
                state = OPP_FRONT_FOLLOW;
            }
            if (distance < 20.0 - (oppSpeed - Speed) * 4) {
                state = OPP_FRONT;
            }
        }
    }
    else if (distance <= -BACKCOLLDIST || distance >= FRONTCOLLDIST) {
//fprintf(stderr, "%s FARAWAY\n",car->_name);fflush(stderr);
        oppFaraway = true;
        state = OPP_IGNORE;
    }

#ifdef OPP_DEBUG
    if (state && state != OPP_IGNORE)
    {
        fprintf(stderr, "%s: %s %s\n", car->_name, (state & OPP_SIDE) ? "SIDE" : ((state & OPP_FRONT) ? "FRONT" : ((state & OPP_BACK) ? "BACK" : "")), (state & OPP_COLL) ? "COLL" : "");
	fflush(stderr);
    }
#endif
}


float Opponent::getCornerDist()
{
    straight2d frontLine(
        mycar->_corner_x(FRNT_LFT),
        mycar->_corner_y(FRNT_LFT),
        mycar->_corner_x(FRNT_RGT) - mycar->_corner_x(FRNT_LFT),
        mycar->_corner_y(FRNT_RGT) - mycar->_corner_y(FRNT_LFT)
        );
    straight2d rearLine(
        mycar->_corner_x(REAR_LFT),
        mycar->_corner_y(REAR_LFT),
        mycar->_corner_x(REAR_RGT) - mycar->_corner_x(REAR_LFT),
        mycar->_corner_y(REAR_RGT) - mycar->_corner_y(REAR_LFT)
        );
    straight2d leftLine(
        mycar->_corner_x(FRNT_LFT),
        mycar->_corner_y(FRNT_LFT),
        mycar->_corner_x(REAR_LFT) - mycar->_corner_x(FRNT_LFT),
        mycar->_corner_y(REAR_LFT) - mycar->_corner_y(FRNT_LFT)
        );
    straight2d rightLine(
        mycar->_corner_x(FRNT_RGT),
        mycar->_corner_y(FRNT_RGT),
        mycar->_corner_x(REAR_RGT) - mycar->_corner_x(FRNT_RGT),
        mycar->_corner_y(REAR_RGT) - mycar->_corner_y(FRNT_RGT)
        );
    double mindist = DBL_MAX;
    bool left[4];
    bool right[4];
    for (int i = 0; i < 4; i++) {
        Vec2d corner(car->_corner_x(i), car->_corner_y(i));
        double frontdist = frontLine.dist(corner);
        double reardist = rearLine.dist(corner);
        double leftdist = leftLine.dist(corner);
        double rightdist = rightLine.dist(corner);
        bool front = frontdist < reardist && reardist > mycar->_dimension_x ? true : false;
        bool rear = reardist < frontdist && frontdist > mycar->_dimension_x ? true : false;
        left[i] = leftdist < rightdist && rightdist > mycar->_dimension_y ? true : false;
        right[i] = rightdist < leftdist && leftdist > mycar->_dimension_y ? true : false;
        double dist = DBL_MAX;
        if (front) {
            dist = frontdist;
        }
        else if (rear) {
            dist = -reardist;
        }
        if (fabs(dist) < fabs(mindist)) {
            mindist = dist;
        }
    }
    if (fabs(mindist) > 3.0) {
        mindist -= SIGN(mindist) * 2.99;
    }
    else {
        mindist = SIGN(mindist) * 0.01;
    }
    /*
    bool lft = true;
    bool rgt = true;
    for (int j = 0; j < 4; j++) {
        if (!left[j]) {
            lft = false;
        }
    }
    for (int k = 0; k < 4; k++) {
        if (!right[k]) {
            rgt = false;
        }
    }
    if (lft || rgt) {
        // opponent aside
        mindist = 0.0;
    }
    */
    return (float)mindist;
}


// Update overlaptimers of opponents.
void Opponent::updateOverlapTimer(tSituation *s, tCarElt *mycar)
{
#if 0
    float mycarFromStartLane = mycar->_distRaced;
    float oppFromStartLane = car->_distRaced;
    float distOpponentCar = oppFromStartLane - mycarFromStartLane;
    fprintf(stderr,"DistOffmyCar %.2f DistOffOpp %.2f OppCarDist %.2f\n", mycarFromStartLane, oppFromStartLane, distOpponentCar);
#endif
    if (car->_remainingLaps < mycar->_remainingLaps ||
        ((team == TEAM_FRIEND) && mycar->_dammage > car->_dammage + MAX_DAMAGE_DIFF))
    {
        if (getState() & (OPP_BACK | OPP_SIDE)) {
            overlaptimer += s->deltaTime;
        }
        else if (getState() & OPP_FRONT) {
            overlaptimer = LAP_BACK_TIME_PENALTY;
        }
        else {
            if (overlaptimer > 0.0) {
                if (getState() & OPP_FRONT_FAST) {
                    overlaptimer = MIN(0.0, overlaptimer);
                }
                else {
                    overlaptimer -= s->deltaTime;
                }
            }
            else {
                overlaptimer += s->deltaTime;
            }
        }
    }
    else {
        overlaptimer = 0.0;
    }
}



int Opponent::polyOverlap(tPosd *op, tPosd *dp)
{
    int i, j;

    // need this to ensure corners are used in the right order
    int cpos[4] = { 1, 0, 2, 3 };

    for (j = 0; j < 4; j++)
    {
        tPosd *j1 = op + cpos[j];
        tPosd *j2 = op + cpos[((j + 1) % 4)];

        for (i = 0; i < 4; i++)
        {
            tPosd *i1 = dp + cpos[i];
            tPosd *i2 = dp + cpos[((i + 1) % 4)];

            double aM, bM, aB, bB, isX = 0, isY = 0;
            double lineAx1 = j1->ax;
            double lineAx2 = j2->ax;
            double lineAy1 = j1->ay;
            double lineAy2 = j2->ay;
            double lineBx1 = i1->ax;
            double lineBx2 = i2->ax;
            double lineBy1 = i1->ay;
            double lineBy2 = i2->ay;

            if ((lineAx2 - lineAx1) == 0.0)
            {
                if ((lineBx2 - lineBx1) == 0.0)
                    continue;
                isX = lineAx1;
                bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
                bB = lineBy2 - bM * lineBx2;
                isY = bM * isX + bB;
            }
            else if ((lineBx2 - lineBx1) == 0.0)
            {
                isX = lineBx1;
                aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
                aB = lineAy2 - aM * lineAx2;
                isY = aM * isX + aB;
            }
            else
            {
                aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
                bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
                aB = lineAy2 - aM * lineAx2;
                bB = lineBy2 - bM * lineBx2;
                isX = MAX(((bB - aB) / (aM - bM)), 0);
                isY = aM * isX + aB;
            }

            if (isX < MIN(lineAx1, lineAx2) || isX < MIN(lineBx1, lineBx2) || isX > MAX(lineAx1, lineAx2) || isX > MAX(lineBx1, lineBx2))
                continue;
            if (isY < MIN(lineAy1, lineAy2) || isY < MIN(lineBy1, lineBy2) || isY > MAX(lineAy1, lineAy2) || isY > MAX(lineBy1, lineBy2))
                continue;

            return 1;
        }
    }

    return 0;
}

static double Mag(double x, double y)
{
    return sqrt((x*x) + (y*y));
}

static double findDistanceBetweenCars(tPosd *c1, tPosd *c2)
{
    double mindist = 10000000;
    
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
        {
            double x = (c2[j].ax-c1[i].ax), y = (c2[j].ay - c1[i].ay);
            double thisdist = sqrt((x * x) + (y * y));
            mindist = MIN(thisdist, mindist);
        }
    }

    return mindist;
}

//#define BRAKE_DEBUG

int Opponent::testLinearCollision2(Driver *driver)
{
    tPosd o_cur[4], d_cur[4], o_new[4], d_new[4], o_new2[4], d_new2[4];
    int i;
    // set up car current positions
    for (i = 0; i < 4; i++)
    {
        o_new[i].ax = o_cur[i].ax = car->_corner_x(i);
        o_new[i].ay = o_cur[i].ay = car->_corner_y(i);
        d_new2[i].ax = d_new[i].ax = d_cur[i].ax = mycar->_corner_x(i);
        d_new2[i].ay = d_new[i].ay = d_cur[i].ay = mycar->_corner_y(i);
    }

    double myta = fabs(RtTrackSideTgAngleL(&(mycar->_trkPos))) * 180 / 3.14159;
    double ota = fabs(RtTrackSideTgAngleL(&(car->_trkPos))) * 180 / 3.14159;
    double tadiff = fabs(myta - ota);
    double trackdistance = car->_distFromStartLine - mycar->_distFromStartLine;
    distance = (tdble)findDistanceBetweenCars(o_cur, d_cur);
#if 0
    if (distance > MAX(5.0, 6 * ((90 - tadiff) / 18)))
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL due to track angle\n",car->_name,mycar->_dammage);fflush(stderr);
#endif
        return 0;
    }
#endif
    double ospeed = car->_speed_x + (distance < 2.0 ? MIN(0.0, car->_accel_x * t_impact * 2.0) : 0.0);
    tdble speedDiff = mycar->_speed_x - car->_speed_x;
    t_impact = (distance) / speedDiff;
    if (distance < 5.0 && car->_accel_x < 0.0)
        speedDiff = mycar->_speed_x - (car->_speed_x + MAX(-2.0, car->_accel_x) * t_impact * 4);
    t_impact = (distance) / speedDiff;
    if (t_impact > MAX(1.0, MIN(trackdistance*2, car->_speed_x / 20)))
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL: t_impact %.2f > speed factor %.2f\n",car->_name,mycar->_dammage,t_impact,MAX(1.0, car->_speed_x/20));fflush(stderr);
#endif
        return 0;
    }
    tdble t_impact2 = distance / speedDiff;

    if (car->_speed_x > 15)
    {
        // make the opponent a little bigger so we stop short of actually hitting
        tdble s1 = (driver->getBrakeMargin() / mycar->_dimension_x)/2, s2 = s1*0.7;
        if (distance < 2.0)
        {
            if (car->_speed_x > 5.0)
            {
                s1 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle))));
                s2 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle))));
            }
            else if (fabs(relativeangle) > 0.5)
            {
                s1 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle)-0.5)));
                s2 *= (tdble)MIN(2.0, (1.0+(fabs(relativeangle)-0.7)));
            }
        }
        o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + (o_cur[REAR_LFT].ax-o_cur[REAR_RGT].ax)*s1;
        o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + (o_cur[REAR_LFT].ay-o_cur[REAR_RGT].ay)*s1;
        o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + (o_cur[REAR_RGT].ax-o_cur[REAR_LFT].ax)*s1;
        o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + (o_cur[REAR_RGT].ay-o_cur[REAR_LFT].ay)*s1;
        o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + (o_cur[FRNT_LFT].ax-o_cur[FRNT_RGT].ax)*s1;
        o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + (o_cur[FRNT_LFT].ay-o_cur[FRNT_RGT].ay)*s1;
        o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + (o_cur[FRNT_RGT].ax-o_cur[FRNT_LFT].ax)*s1;
        o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + (o_cur[FRNT_RGT].ay-o_cur[FRNT_LFT].ay)*s1;
        o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + (o_cur[FRNT_LFT].ax-o_cur[REAR_LFT].ax)*s2;
        o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + (o_cur[FRNT_LFT].ay-o_cur[REAR_LFT].ay)*s2;
        o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + (o_cur[FRNT_RGT].ax-o_cur[REAR_RGT].ax)*s2;
        o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + (o_cur[FRNT_RGT].ay-o_cur[REAR_RGT].ay)*s2;
        o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + (o_cur[REAR_LFT].ax-o_cur[FRNT_LFT].ax)*s2;
        o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + (o_cur[REAR_LFT].ay-o_cur[FRNT_LFT].ay)*s2;
        o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + (o_cur[REAR_RGT].ax-o_cur[FRNT_RGT].ax)*s2;
        o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + (o_cur[REAR_RGT].ay-o_cur[FRNT_RGT].ay)*s2;
    }

    d_new2[FRNT_LFT].ax += (d_new[FRNT_LFT].ax - d_new[REAR_LFT].ax) / 3;
    d_new2[FRNT_LFT].ay += (d_new[FRNT_LFT].ay - d_new[REAR_LFT].ay) / 3;
    d_new2[FRNT_RGT].ax += (d_new[FRNT_RGT].ax - d_new[REAR_RGT].ax) / 3;
    d_new2[FRNT_RGT].ay += (d_new[FRNT_RGT].ay - d_new[REAR_RGT].ay) / 3;

    if (polyOverlap(o_new, d_new2))
    {
        collspeed = (float) MIN(collspeed, car->_speed_x-2.0);
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d close overlap collision collspeed=%.2f\n",car->_name,mycar->_dammage,collspeed);fflush(stderr);
#endif
        return OPP_COLL | OPP_COLL_WARNING; // cars are overlapping now.
    }

    double brake_coefficient = driver->getBrakeCoefficient();
    if (fabs(relativeangle) > 0.3)
        brake_coefficient /= (1.0 + (fabs(relativeangle) - 0.3)*1);
    //if (car->_speed_x < 10 && distance < 3)
    //    brake_coefficient = MIN(brake_coefficient, 0.02);
#if 1
    if (!(state && OPP_SIDE))
    {
        double mySpd = hypot(mycar->_speed_X, mycar->_speed_Y);
        if (fabs(mySpd) < 0.01) mySpd = 0.01;
        double myDirX = car->_speed_X / mySpd;
        double myDirY = car->_speed_Y / mySpd;

        double dPX = car->pub.DynGCg.pos.x - mycar->pub.DynGCg.pos.x;
        double dPY = car->pub.DynGCg.pos.y - mycar->pub.DynGCg.pos.y;
        double dVX = car->_speed_X - mycar->_speed_X;
        double dVY = car->_speed_Y - mycar->_speed_Y;
        double rdPX = myDirX * dPX + myDirY * dPY;
        double rdPY = myDirY * dPX - myDirX * dPY;
        double rdVX = myDirX * dVX + myDirY * dVY;
        double rdVY = myDirY * dVX - myDirX * dVY;
        double oVX = car->_speed_x + rdVX;
        double minDY = (mycar->_dimension_y + car->_dimension_y) / 2;
        double minDX = (mycar->_dimension_x + car->_dimension_x) / 2;

        double currentEk = driver->mass() * mycar->_speed_x * mycar->_speed_x / 2;
        double targetEk = driver->mass() * ospeed * ospeed / 2;
        if (currentEk - targetEk > t_impact * 750000 / brake_coefficient)
        {
            if (state & OPP_SIDE)
            {
                if (rdPY < minDY)      // colliding now
                    collspeed = oVX-3;
                else                   // colliding soon
                {
                    double t = (fabs(rdPY) - minDY) / fabs(rdVY);
                    double collX = rdPX + rdVX * t;
                    if (collX > minDX*0.5 && collX < minDX)
                        collspeed = oVX-3;
                }
            }
            else
                collspeed = oVX;
           //collspeed = MAX(ospeed, mycar->_speed_x - (speedDiff * ((currentEk - (t_impact * 100000 / brake_coefficient)) / (currentEk - targetEk))));
        }
        else
           collspeed = mycar->_speed_x + 2.0;
    }
    else
        collspeed = car->_speed_x - 1.0;
#else
    collspeed = (float)(car->_speed_x + (t_impact * MAX(1.0, t_impact/2) * (brake_coefficient*10)));
#endif
    //collspeed = (float)(car->_speed_x + (car->_accel_x * t_impact2) + t_impact * (brake_coefficient*10));
    //collspeed = MIN(collspeed, (collspeed - fabs(relativeangle)*3) + MAX(0.0, (distance-1.0)*2.3));

    if (!(state & OPP_SIDE) && speedDiff <= 0 && car->_speed_x > 5.0) 
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL: speedDiff %.1f <= 0 and speed %.2f > 5.0\n",car->_name,mycar->_dammage,speedDiff,car->_speed_x);fflush(stderr);
#endif
        return 0;
    }
    //speedDiff *= MIN(1.0, speedDiff/3);

    LRaceLine *raceline = driver->getRaceLine();
    int rl = driver->currentRaceline();
    int div = raceline->DivIndexForCar(car, t_impact);
    double rlspeed = raceline->tSpeed[rl][div];

#if 0
    if (driver->isOnRaceline() && rlspeed > collspeed && rlspeed <= car->_speed_x)
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d NO COLL: speedDiff=%.1f collspeed=%.2f\n",car->_name,mycar->_dammage,speedDiff,collspeed);fflush(stderr);
#endif
        return 0;
    }
#endif

    if (t_impact > ((state & OPP_SIDE) ? 1.5 : 4.0))//MIN(3.0, MAX(0.7, speedDiff)))
    {
#ifdef BRAKE_DEBUG
        fprintf(stderr, "%s %d SIDE - NO COLL: t_impact %.2f > 5.0, dist=%.1f speedDiff=%.1f collspeed=%.2f\n",car->_name,mycar->_dammage,t_impact,distance,speedDiff,collspeed);fflush(stderr);
#endif
        return 0;
    }

    // move the opponent back a little towards our car, and project everything forward in time
    double deltax = 0.0, deltay = 0.0;
    for (i = 0; i < 4; i++)
    {
        // project opponent car by linear velocity, and make o_new2 moved back a little
        // in the direction of our car.
        double thisdist = sqrt((deltax * deltax) + (deltay * deltay));
        o_new2[i].ax = (tdble)(o_cur[i].ax + car->_speed_X * t_impact2);
        o_new2[i].ay = (tdble)(o_cur[i].ay + car->_speed_Y * t_impact2);
        deltax = o_new2[i].ax - o_cur[i].ax;
        deltay = o_new2[i].ay - o_cur[i].ay;
        o_new[i].ax = o_cur[i].ax + (tdble)(thisdist * sin(getProjectedSpeedAngle(t_impact2)));
        o_new[i].ay = o_cur[i].ay + (tdble)(thisdist * cos(getProjectedSpeedAngle(t_impact2)));

        if (i == FRNT_LFT || i == FRNT_RGT)
        {
            // project front of our car by speed and turn direction
            d_new[i].ax += (tdble)(mycar->_speed_X * t_impact2);
            d_new[i].ay += (tdble)(mycar->_speed_Y * t_impact2);
            deltax = d_new[i].ax - d_cur[i].ax;
            deltay = d_new[i].ay - d_cur[i].ay;
            //theta = atan2(deltax, deltay) - ((mycar->_yaw_rate*0.8) * t_impact2);
            thisdist = sqrt((deltax * deltax) + (deltay * deltay));
            double newax = d_cur[i].ax + thisdist * sin(driver->getSpeedAngle(t_impact2));
            double neway = d_cur[i].ay + thisdist * cos(driver->getSpeedAngle(t_impact2));
            if ((mycar->_yaw_rate < 0.0 && i == FRNT_RGT) || (mycar->_yaw_rate > 0.0 && i == FRNT_LFT))
            {
                d_new2[i].ax = (tdble)newax;
                d_new2[i].ay = (tdble)neway;
            }
            else
            {
                d_new2[i].ax = (tdble)(d_new[i].ax * 0.3 + newax * 0.7);
                d_new2[i].ay = (tdble)(d_new[i].ay * 0.3 + neway * 0.7);
            }
//fprintf(stderr, "%s: old=%.2f/%.2f new1=%.2f/%.2f yr=%.5f theta=%.5f new2=%.2f/%.2f\n",car->_name,d_cur[i].ax,d_cur[i].ay,new_ax,new_ay,mycar->_yaw_rate,theta,d_new[i].ax,d_new[i].ay);fflush(stderr);
        }
    }

    deltax = ((d_new2[FRNT_LFT].ax-o_new[REAR_LFT].ax)+(d_new2[FRNT_RGT].ax-o_new[REAR_RGT].ax))/2;
    deltay = ((d_new2[FRNT_LFT].ay-o_new[REAR_LFT].ay)+(d_new2[FRNT_RGT].ay-o_new[REAR_RGT].ay))/2;
    double theta = atan2(deltax, deltay);
    double movedist = 0.5 + MIN(1.5, speedDiff/15);
    for (i = 0; i < 4; i++)
    {
        o_new2[i].ax = (tdble)(o_new[i].ax + movedist * sin(theta));
        o_new2[i].ay = (tdble)(o_new[i].ay + movedist * cos(theta));
    }

    double trackdist = car->_distFromStartLine - mycar->_distFromStartLine;
    double sidedist = fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft);
    if ((!(MIN(car->_trkPos.toLeft, car->_trkPos.toRight) < -3 && MIN(mycar->_trkPos.toLeft, mycar->_trkPos.toRight) >= 0)) &&
        (sidedist / 2 < MAX(t_impact, trackdist/2) || sidedist < 2.0 || speedDiff > 15))
        if (polyOverlap(o_new, d_new2) || polyOverlap(o_new2, d_new2))
        {
#ifdef BRAKE_DEBUG
            fprintf(stderr, "%s %d projected overlap collision dist=%.2f ospd=%.2f spd=%.2f collspeed=%.2f t_impact=%.2f sidedist=%.2f\n",car->_name,mycar->_dammage,distance,car->_speed_x,mycar->_speed_x,collspeed,t_impact,fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft));fflush(stderr);
#endif
            return OPP_COLL | OPP_COLL_WARNING;
        }

    //if (driver->isOnRaceline())// && car->_pos > mycar->_pos)
    if ((state & OPP_SIDE))
        return 0;

    {
        int coll = testRacelineCollision(driver, distance, t_impact);
        if (coll)
        {
#ifdef BRAKE_DEBUG
            if (coll & OPP_COLL)
                fprintf(stderr, "%s %d projected %s collision, collspeed=%.2f\n",car->_name,mycar->_dammage,((coll & OPP_RACELINE_CONFLICT) ? "raceline" : "linear"),collspeed);fflush(stderr);
#endif
            return coll;
        }
    }
#ifdef BRAKE_DEBUG
    fprintf(stderr, "%s %d NO collision t_impact=%.2f dist=%.2f\n",car->_name,mycar->_dammage,t_impact,distance);fflush(stderr);
#endif

    if (mycar->_yaw_rate < 0.0)
    {
        d_new2[FRNT_LFT].ax = d_new[FRNT_LFT].ax;
        d_new2[FRNT_LFT].ay = d_new[FRNT_LFT].ay;
    }
    else
    {
        d_new2[FRNT_RGT].ax = d_new[FRNT_RGT].ax;
        d_new2[FRNT_RGT].ay = d_new[FRNT_RGT].ay;
    }
    o_new[REAR_LFT].ax = (o_new2[REAR_LFT].ax + (o_new2[REAR_LFT].ax-o_new2[REAR_RGT].ax)/3);
    o_new[REAR_LFT].ay = (o_new2[REAR_LFT].ay + (o_new2[REAR_LFT].ay-o_new2[REAR_RGT].ay)/3);
    o_new[REAR_RGT].ax = (o_new2[REAR_RGT].ax + (o_new2[REAR_RGT].ax-o_new2[REAR_LFT].ax)/3);
    o_new[REAR_RGT].ay = (o_new2[REAR_RGT].ay + (o_new2[REAR_RGT].ay-o_new2[REAR_LFT].ay)/3);
    o_new[FRNT_LFT].ax = (o_new2[FRNT_LFT].ax + (o_new2[FRNT_LFT].ax-o_new2[FRNT_RGT].ax)/3);
    o_new[FRNT_LFT].ay = (o_new2[FRNT_LFT].ay + (o_new2[FRNT_LFT].ay-o_new2[FRNT_RGT].ay)/3);
    o_new[FRNT_RGT].ax = (o_new2[FRNT_RGT].ax + (o_new2[FRNT_RGT].ax-o_new2[FRNT_LFT].ax)/3);
    o_new[FRNT_RGT].ay = (o_new2[FRNT_RGT].ay + (o_new2[FRNT_RGT].ay-o_new2[FRNT_LFT].ay)/3);
    o_new[FRNT_LFT].ax = (o_new2[FRNT_LFT].ax + (o_new2[FRNT_LFT].ax-o_new2[REAR_LFT].ax)/3);
    o_new[FRNT_LFT].ay = (o_new2[FRNT_LFT].ay + (o_new2[FRNT_LFT].ay-o_new2[REAR_LFT].ay)/3);
    o_new[FRNT_RGT].ax = (o_new2[FRNT_RGT].ax + (o_new2[FRNT_RGT].ax-o_new2[REAR_RGT].ax)/3);
    o_new[FRNT_RGT].ay = (o_new2[FRNT_RGT].ay + (o_new2[FRNT_RGT].ay-o_new2[REAR_RGT].ay)/3);
    o_new[REAR_LFT].ax = (o_new2[REAR_LFT].ax + (o_new2[REAR_LFT].ax-o_new2[FRNT_LFT].ax)/3);
    o_new[REAR_LFT].ay = (o_new2[REAR_LFT].ay + (o_new2[REAR_LFT].ay-o_new2[FRNT_LFT].ay)/3);
    o_new[REAR_RGT].ax = (o_new2[REAR_RGT].ax + (o_new2[REAR_RGT].ax-o_new2[FRNT_RGT].ax)/3);
    o_new[REAR_RGT].ay = (o_new2[REAR_RGT].ay + (o_new2[REAR_RGT].ay-o_new2[FRNT_RGT].ay)/3);

    if (polyOverlap(o_new, d_new2))
    {
        //fprintf(stderr, "%s warning collision\n",car->_name);fflush(stderr);
        return OPP_COLL_WARNING;
    }

    return 0;
}

int Opponent::testLinearCollision(Driver *driver, double ti, double speed_difference, double catch_distance, double multiplier)
{
    if (mycar->_speed_x < 15) return testLinearCollision2(driver);
#if 0
    double o_accel_X = (car->_speed_X - prev_speed_X) * deltamult;
    double o_accel_Y = (car->_speed_Y - prev_speed_Y) * deltamult;
    double d_accel_X = (mycar->_speed_X - d_prev_speed_X) * deltamult;
    double d_accel_Y = (mycar->_speed_Y - d_prev_speed_Y) * deltamult;
    double velX = (mycar->_speed_X + d_accel_X) - (car->_speed_X + o_accel_X);
    double velY = (mycar->_speed_Y + d_accel_Y) - (car->_speed_Y + o_accel_Y);
    speed_difference = Mag(velX, velY);
    //speed_difference = MAX(1.0, speed_difference);
    double brake_coefficient = driver->getBrakeCoefficient();
    double brake_distance = brake_coefficient * (speed_difference * speed_difference);
    //if (brake_distance < catch_distance - 2.0 && speed_difference < 2.0)
     //   return 0;

    tCarElt *mycar = driver->getCarPtr();
    int i;
    tCarElt *dcar = driver->getCarPtr();
    double o_speedX = car->_speed_X;
    double o_speedY = car->_speed_Y;
    double d_speedX = dcar->_speed_X;
    double d_speedY = dcar->_speed_Y;

    ti *= 1.0;

    tPosd o_cur[4], d_cur[4], o_new[4], d_new[4], o_new2[4], d_new2[4];

    // set up car current positions
    for (i = 0; i < 4; i++)
    {
        o_cur[i].ax = car->_corner_x(i);
        o_cur[i].ay = car->_corner_y(i);
        d_cur[i].ax = dcar->_corner_x(i);
        d_cur[i].ay = dcar->_corner_y(i);
    }

    //tPosd carCenter, ocarCenter;
    //centerPoint(&d_cur[0], &d_cur[1], &d_cur[2], &d_cur[3], &carCenter);
    //centerPoint(&o_cur[0], &o_cur[1], &o_cur[2], &o_cur[3], &ocarCenter);

    //double angleFromOCarToCar = AngleBetweenPoints(&carCenter, &ocarCenter);

    double sizefactor = MAX(0.1, speed_difference*speed_difference/40);
    //if (dcar->_speed_x < 7.0)
    //    sizefactor = 0.1;
    double rear_lft_ax = d_cur[REAR_LFT].ax + dcar->_speed_X*ti;
    double rear_lft_ay = d_cur[REAR_LFT].ay + dcar->_speed_Y*ti;
    double rear_rgt_ax = d_cur[REAR_RGT].ax + dcar->_speed_X*ti;
    double rear_rgt_ay = d_cur[REAR_RGT].ay + dcar->_speed_Y*ti;
    d_new[REAR_LFT].ax = d_cur[REAR_LFT].ax;
    d_new[REAR_LFT].ay = d_cur[REAR_LFT].ay;
    d_new[REAR_RGT].ax = d_cur[REAR_RGT].ax;
    d_new[REAR_RGT].ay = d_cur[REAR_RGT].ay;
    d_new[FRNT_LFT].ax = d_cur[FRNT_LFT].ax + (d_cur[FRNT_LFT].ax-d_cur[REAR_LFT].ax)*sizefactor + dcar->_speed_X*(ti);// + sizefactor/3);
    d_new[FRNT_LFT].ay = d_cur[FRNT_LFT].ay + (d_cur[FRNT_LFT].ay-d_cur[REAR_LFT].ay)*sizefactor + dcar->_speed_Y*(ti);// + sizefactor/3);
    d_new[FRNT_RGT].ax = d_cur[FRNT_RGT].ax + (d_cur[FRNT_RGT].ax-d_cur[REAR_RGT].ax)*sizefactor + dcar->_speed_X*(ti);// + sizefactor/3);
    d_new[FRNT_RGT].ay = d_cur[FRNT_RGT].ay + (d_cur[FRNT_RGT].ay-d_cur[REAR_RGT].ay)*sizefactor + dcar->_speed_Y*(ti);// + sizefactor/3);
    //d_new[FRNT_LFT].ax += (d_new[FRNT_LFT].ax-d_new[REAR_LFT].ax)*sizefactor;
    //d_new[FRNT_LFT].ay += (d_new[FRNT_LFT].ay-d_new[REAR_LFT].ay)*sizefactor;
    //d_new[FRNT_RGT].ax += (d_new[FRNT_RGT].ax-d_new[REAR_RGT].ax)*sizefactor;
    //d_new[FRNT_RGT].ay += (d_new[FRNT_RGT].ay-d_new[REAR_RGT].ay)*sizefactor;

    /*
    sizefactor = MIN(1.0, speed_difference / (multiplier * 3));
    double lft_ax = d_new[FRNT_LFT].ax, lft_ay = d_new[FRNT_LFT].ay;
    d_new[FRNT_LFT].ax += (d_new[FRNT_LFT].ax - d_new[FRNT_RGT].ax) * sizefactor;
    d_new[FRNT_LFT].ay += (d_new[FRNT_LFT].ay - d_new[FRNT_RGT].ay) * sizefactor;
    d_new[FRNT_RGT].ax += (d_new[FRNT_RGT].ax - lft_ax) * sizefactor;
    d_new[FRNT_RGT].ay += (d_new[FRNT_RGT].ay - lft_ay) * sizefactor;
    */

    o_new[REAR_LFT].ax = o_cur[REAR_LFT].ax + car->_speed_X*ti;
    o_new[REAR_LFT].ay = o_cur[REAR_LFT].ay + car->_speed_Y*ti;
    o_new[REAR_RGT].ax = o_cur[REAR_RGT].ax + car->_speed_X*ti;
    o_new[REAR_RGT].ay = o_cur[REAR_RGT].ay + car->_speed_Y*ti;
    o_new[FRNT_LFT].ax = o_cur[FRNT_LFT].ax + car->_speed_X*ti;
    o_new[FRNT_LFT].ay = o_cur[FRNT_LFT].ay + car->_speed_Y*ti;
    o_new[FRNT_RGT].ax = o_cur[FRNT_RGT].ax + car->_speed_X*ti;
    o_new[FRNT_RGT].ay = o_cur[FRNT_RGT].ay + car->_speed_Y*ti;

#if 1
    {
        // the faster the speed difference the larger the opponent car is
        double sizefactor = MAX(0.5, speed_difference/7);
        double o_rear_lft_ax = o_new[REAR_LFT].ax, o_rear_lft_ay = o_new[REAR_LFT].ay;
        double o_rear_rgt_ax = o_new[REAR_RGT].ax, o_rear_rgt_ay = o_new[REAR_RGT].ay;
        o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[FRNT_RGT].ax) * brake_multiplier * sizefactor;
        o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[FRNT_RGT].ay) * brake_multiplier * sizefactor;
        o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[FRNT_LFT].ax) * brake_multiplier * sizefactor;
        o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[FRNT_LFT].ay) * brake_multiplier * sizefactor;
        o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_rear_rgt_ax) * brake_multiplier * sizefactor;
        o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_rear_rgt_ay) * brake_multiplier * sizefactor;
        o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_rear_lft_ax) * brake_multiplier * sizefactor;
        o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_rear_lft_ay) * brake_multiplier * sizefactor;
        /*
        if (fabs(relativeangle) < 0.7)
        {
            o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[FRNT_LFT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[FRNT_LFT].ay) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[FRNT_RGT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[FRNT_RGT].ay) * MAX(0.3, sizefactor);
        }
        else if (fabs(relativeangle) > 1.45)
        {
            o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_new[REAR_LFT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_new[REAR_LFT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_new[REAR_RGT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_new[REAR_RGT].ay) * MAX(0.3, sizefactor);
        }
        else if (relativeangle < 0.0)
        {
            o_new[REAR_RGT].ax += (o_new[REAR_RGT].ax - o_new[REAR_LFT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_RGT].ay += (o_new[REAR_RGT].ay - o_new[REAR_LFT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ax += (o_new[FRNT_RGT].ax - o_new[FRNT_LFT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_RGT].ay += (o_new[FRNT_RGT].ay - o_new[FRNT_LFT].ay) * MAX(0.3, sizefactor);
        }
        else
        {
            o_new[REAR_LFT].ax += (o_new[REAR_LFT].ax - o_new[REAR_RGT].ax) * MAX(0.3, sizefactor);
            o_new[REAR_LFT].ay += (o_new[REAR_LFT].ay - o_new[REAR_RGT].ay) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ax += (o_new[FRNT_LFT].ax - o_new[FRNT_RGT].ax) * MAX(0.3, sizefactor);
            o_new[FRNT_LFT].ay += (o_new[FRNT_LFT].ay - o_new[FRNT_RGT].ay) * MAX(0.3, sizefactor);
        }
        */
    }
#endif

    // test for collision
    if (polyOverlap(o_new, d_new))
    {
        if (brake_distance > catch_distance + 1.0)
            return 2;
        return 1;
    }
    d_new[REAR_LFT].ax = rear_lft_ax;
    d_new[REAR_LFT].ay = rear_lft_ay;
    d_new[REAR_RGT].ax = rear_rgt_ax;
    d_new[REAR_RGT].ay = rear_rgt_ay;
    if (polyOverlap(o_new, d_new))
    {
        if (brake_distance > catch_distance + 1.0)
            return 2;
        return 1;
    }

#endif // 0
    return 0;
}

int Opponent::testQuadraticCollision(Driver *driver)
{
    int collision = 0;
    return testLinearCollision2(driver);
#if 0
    // quadratics from mouse/sieger ... except I can't get them to work :\
    double d_Spd = Mag(mycar->_speed_X, mycar->_speed_Y);
    double o_Spd = Mag(car->_speed_X, car->_speed_Y);
    double d_DirX = mycar->_speed_X / d_Spd;
    double d_DirY = mycar->_speed_Y / d_Spd;
    double o_DirX = car->_speed_X / o_Spd;
    double o_DirY = car->_speed_Y / o_Spd;

    double d_ragAX = d_DirX * driver->average_AX + d_DirY * driver->average_AY;
    double o_ragAX = o_DirX * average_AX + o_DirY * average_AY;
    double d_ragAY = d_DirY * driver->average_AX - d_DirX * driver->average_AY;
    double o_ragAY = o_DirY * average_AX - o_DirX * average_AY;

    double dPX = car->pub.DynGCg.pos.x - mycar->pub.DynGCg.pos.x;
    double dPY = car->pub.DynGCg.pos.y - mycar->pub.DynGCg.pos.y;
    double dVX  = car->_speed_X - mycar->_speed_X;
    double dVY  = car->_speed_Y - mycar->_speed_Y;

    double o_rdPX = d_DirX * dPX + d_DirY * dPY;
    double o_rdPY = d_DirY * dPX - d_DirX * dPY;
    double o_rdVX = d_DirX * dVX + d_DirY * dVY;
    double o_rdVY = d_DirY * dVX - d_DirX * dVY;

    double minDX = (car->_dimension_x + mycar->_dimension_x) / 2;
    double minDY = (car->_dimension_y + mycar->_dimension_y) / 2;

    Quadratic myPar(0, 0, 0, d_ragAY);
    Quadratic oPar(0, o_rdPY, o_rdVY, o_ragAY);
    Quadratic relPar = oPar - myPar;

    double acc = o_ragAX;
    Quadratic q(acc/2, o_rdVX, o_rdPX - minDX);
    double t = 0.0;
    if (q.SmallestNonNegativeRoot(t))
    {
        double catchY = relPar.CalcY(t);
        if (fabs(catchY) < minDY)
            collision = 1;
        else
        {
            q.Setup( acc/2, o_rdVX, o_rdPX + minDX );
            if (q.SmallestNonNegativeRoot(t))
            {
                catchY = relPar.CalcY(t);
                if (fabs(catchY) < minDY || catchY * o_rdPY < 0)
                    collision = 1;
            }
        }
    }

    return collision;
#endif
}

static void centerPoint(tPosd *p1, tPosd *p2, tPosd *p3, tPosd *p4, tPosd *pt)
{
    pt->ax = (p1->ax + p2->ax + p3->ax + p4->ax) / 4;
    pt->ay = (p1->ay + p2->ay + p3->ay + p4->ay) / 4;
}

static double AngleBetweenPoints(tPosd *target, tPosd *origin)
{
    return atan2(target->ay - origin->ay, target->ax - origin->ax);
}

static void FindPointAlongAngle(tPosd *origin, double angle, double distance, tPosd *target)
{
    tdble deltax = (tdble)(distance * cos(angle));
    tdble deltay = (tdble)(distance * sin(angle));
    target->ax = origin->ax + deltax;
    target->ay = origin->ay + deltay;
}

static double sign(tPosd *p1, tPosd *p2, tPosd *p3)
{
    return (p1->ax - p3->ax) * (p2->ay - p3->ay) - (p2->ax - p3->ax) * (p1->ay - p3->ay);
}

static int pointInTriangle(tPosd *pt, tPosd *v1, tPosd *v2, tPosd *v3)
{
#if 0
    // Barycentric method
    double s = v1->ay * v3->ax - v1->ax * v3->ay + (v3->ay - v1->ay) * pt->ax + (v1->ax - v3->ax) * pt->ay;
    double t = v1->ax * v2->ay - v1->ay * v2->ax + (v1->ay - v2->ay) * pt->ax + (v2->ax - v1->ax) * pt->ay;

    if ((s < 0) != (t < 0))
        return 0;

    double A = -v2->ay * v3->ax + v1->ay * (v3->ax - v2->ax) + v1->ax * (v2->ay - v3->ay) + v2->ax * v3->ay;
    if (A < 0.0)
    {
        s = -s;
        t = -t;
        A = -A;
    }
    return s > 0 && t > 0 && (s + t) < A;
#else
    int i1, i2, i3;

    i1 = (sign(pt, v1, v2) < 0.0);
    i2 = (sign(pt, v2, v3) < 0.0);
    i3 = (sign(pt, v3, v1) < 0.0);

    return ((i1 == i2) && (i2 == i3));
#endif
}

int Opponent::testRacelineCollision(Driver *driver, double distance, double t_impact)
{
    int coll = 0;
    LRaceLine *raceline = driver->getRaceLine();
    int div = raceline->DivIndexForCar(car, t_impact);
    double rlspeed = raceline->tSpeed[driver->currentRaceline()][div];

    if (rlspeed <= collspeed)
        return 0;

    double o_toLeft = car->_trkPos.toLeft + (avgLateralMovt * t_impact * deltamult);
    if (car->_trkPos.toLeft >= 0.0)
        o_toLeft = MAX(o_toLeft, 0.0);
    if (car->_trkPos.toRight <= track->width)
        o_toLeft = MIN(o_toLeft, track->width);
    double speedDiff = distance / t_impact;
    double factor = 1.0;

    if (driver->isOnRaceline() && driver->currentRaceline() >= LINE_RL)
    {
        double lane = raceline->tLane[LINE_RL][div];
        double lane2left = lane * track->width;
        if (distance < 5.0 && (lane2left < o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] > 0.005) || (lane2left > o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] < -0.005))
            factor += fabs(raceline->tRInverse[LINE_RL][raceline->Next]) * 20;

        if (distance > 2.0 && t_impact < 3.0 && fabs(o_toLeft - lane2left) < 2.0 * factor - MIN((15 - MIN(10.0, (distance-10)/4)) / 5, MAX(0, 2 - speedDiff/10)))
            return OPP_COLL | OPP_COLL_WARNING | OPP_RACELINE_CONFLICT;
        else if (fabs(o_toLeft - lane2left) < 6.0 && t_impact < 5.0)
            return OPP_COLL_WARNING;
    }
    
    {
        double m_toLeft = mycar->_trkPos.toLeft + (driver->avgLateralMovt * t_impact * deltamult);
        if (mycar->_trkPos.toLeft >= 0.0)
            m_toLeft = MAX(m_toLeft, 0.0);
        if (mycar->_trkPos.toRight <= track->width)
            m_toLeft = MIN(m_toLeft, track->width);
        if (distance < 5.0 && ((m_toLeft < o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] > 0.005) || (m_toLeft > o_toLeft && raceline->tRInverse[LINE_RL][raceline->Next] < -0.005)))
            factor += fabs(raceline->tRInverse[LINE_RL][raceline->Next]) * 20;

        if (distance > 2.0 && t_impact < 3.0 && fabs(o_toLeft - m_toLeft) < 2.0 * factor - MIN((15 - MIN(10.0, (distance-10)/4)) / 5, MAX(0, 2 - speedDiff/10)))
            return OPP_COLL | OPP_COLL_WARNING | OPP_COLL_LINEAR;
        else if (fabs(o_toLeft - m_toLeft) < 6.0 && t_impact < 5.0)
            return OPP_COLL_WARNING;
    }

    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
// OPPONENTS CLASS
///////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize the list of opponents.
Opponents::Opponents(tSituation *s, Driver *driver, Cardata *c, double brake_multiplier, double brake_warn_multiplier)
{
    opponent = new Opponent[s->_ncars - 1];
    int i, j = 0;
    for (i = 0; i < s->_ncars; i++) {
        if (s->cars[i] != driver->getCarPtr()) {
            opponent[j].setCarPtr(s->cars[i]);
            opponent[j].setCarDataPtr(c->findCar(s->cars[i]));
            opponent[j].setIndex(i);
            opponent[j].brake_multiplier = brake_multiplier;
            opponent[j].brake_warn_multiplier = brake_warn_multiplier;
            j++;
        }
    }
    Opponent::setTrackPtr(driver->getTrackPtr());
    nopponents = s->_ncars - 1;
}


Opponents::~Opponents()
{
    delete[] opponent;
}


void Opponents::update(tSituation *s, Driver *driver)
{
    int i;
    for (i = 0; i < s->_ncars - 1; i++) {
        opponent[i].update(s, driver);
    }
}


void Opponents::setTeamMate(const char *teammate)
{
    int i;
    for (i = 0; i < nopponents; i++) {
        if (strcmp(opponent[i].getCarPtr()->_name, teammate) == 0) {
            opponent[i].markAsTeamMate();
            break;    // Name should be unique, so we can stop.
        }
    }
}




