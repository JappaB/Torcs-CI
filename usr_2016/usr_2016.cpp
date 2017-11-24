/***************************************************************************

    file                 : usr_2016.cpp
    created              : Wed Jan 8 18:31:16 CET 2003
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: bt.cpp,v 1.5 2006/03/06 22:43:50 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

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

#include "driver.h"

#define NBBOTS 2

// edit daan
// csv stuff
#include <iostream>
#include <fstream>
std::ofstream fs;
// sensordata stuff
#include "sensors.h"
#include "SimpleParser.h"
#include "CarControl.h"
#include "ObstacleSensors.h"
static tTrack* curTrack;
static tdble oldAccel[NBBOTS];
static tdble oldBrake[NBBOTS];
static tdble oldSteer[NBBOTS];
static tdble oldClutch[NBBOTS];
static tdble prevDist[NBBOTS];
static tdble distRaced[NBBOTS];
static int oldFocus[NBBOTS];//ML
static int oldGear[NBBOTS];
#define __SENSORS_RANGE__ 200.0
#define __FOCUS_RANGE__ 200.0
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02
#define __FOCUS_NOISE_STD__ 0.01
static Sensors* trackSens[NBBOTS];
static ObstacleSensors* oppSens[NBBOTS];
// /edit daan

static const char* botname[NBBOTS] = {"Giskard", "Daneel"};
static const char* botdesc[NBBOTS] = {"Giskard", "Daneel"};

static Driver *driver[NBBOTS];

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newRace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static int pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);
static void endRace(int index, tCarElt *car, tSituation *s);


// edit daan
double normRand(double avg,double std)
{
     double x1, x2, w, y1, y2;

        do {
                x1 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
                x2 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
                w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );

        w = sqrt( (-2.0 * log( w ) ) / w );
        y1 = x1 * w;
        y2 = x2 * w;
        return y1*std + avg;
}
// /edit daan

// Module entry point.
extern "C" int usr_2016(tModInfo *modInfo)
{
    int i;
    
    // Clear all structures.
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (i = 0; i < NBBOTS; i++) {
        modInfo[i].name    = strdup(botname[i]);          // name of the module (short).
        modInfo[i].desc    = strdup(botdesc[i]);        // Description of the module (can be long).
        modInfo[i].fctInit = InitFuncPt;            // Init function.
        modInfo[i].gfId    = ROB_IDENT;                // Supported framework version.
        modInfo[i].index   = i;                    // Indices from 0 to 9.
    }
    return 0;
}


// Module interface initialization.
static int InitFuncPt(int index, void *pt)
{
    tRobotItf *itf = (tRobotItf *)pt;

    // Create robot instance for index.
    driver[index] = new Driver(index);
    itf->rbNewTrack = initTrack;    // Give the robot the track view called.
    itf->rbNewRace  = newRace;        // Start a new race.
    itf->rbDrive    = drive;        // Drive during race.
    itf->rbPitCmd   = pitcmd;        // Pit commands.
    itf->rbEndRace  = endRace;        // End of the current race.
    itf->rbShutdown = shutdown;        // Called before the module is unloaded.
    itf->index      = index;        // Index used if multiple interfaces.
    return 0;
}


// Called for every track change or new race.
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    // edit daan
    curTrack = track;
    // /edit daan
    driver[index]->initTrack(track, carHandle, carParmHandle, s);
}


// Start a new race.
static void newRace(int index, tCarElt* car, tSituation *s)
{
    // edit daan
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::stringstream filename;
    filename << curTrack->category << "_"
             << curTrack->internalname << "_"
             << s->raceInfo.ncars << "cars_"
             << ltm->tm_mday
             << ltm->tm_hour
             << ltm->tm_min
             << ltm->tm_sec
             << ".csv";
    fs.open(("data/"+filename.str()).c_str());
    std::string columns[] = {
        "accel",
        "brake",
        "steer",
        "angle",
        "curLapTime",
        "distFromStart",
        "distRaced",
        "gear",
        "lastLapTime",
        "racePos",
        "rpm",
        "speedX",
        "speedY",
        "speedZ",
        "trackPos",
        "z",
        "wheelSpinVel01",
        "wheelSpinVel02",
        "wheelSpinVel03",
        "wheelSpinVel04",
        "track00","track01","track02","track03","track04",
        "track05","track06","track07","track08","track09",
        "track10","track11","track12","track13","track14",
        "track15","track16","track17","track18",
        "oppos00","oppos01","oppos02","oppos03","oppos04",
        "oppos05","oppos06","oppos07","oppos08","oppos09",
        "oppos10","oppos11","oppos12","oppos13","oppos14",
        "oppos15","oppos16","oppos17","oppos18","oppos19",
        "oppos20","oppos21","oppos22","oppos23","oppos24",
        "oppos25","oppos26","oppos27","oppos28","oppos29",
        "oppos30","oppos31","oppos32","oppos33","oppos34",
        "oppos35"
    };
    std::stringstream ss;
    for (int i = 0; i < 75; ++i)
    {
        if(i != 0)
            ss << ";";
        ss << columns[i];
    }
    std::string columnString = ss.str();
    fs << columnString << std::endl;
    float angles[19] = {
        -90.0, 
        -53.1107291436, 
        -28.1000420415,
        -15.7275288623,
        -9.36938509649,
        -5.81853020071,
        -3.70934256685,
        -2.40436404382,
        -1.57523692569,
        0,
        1.57523692569,
        2.40436404382,
        3.70934256685,
        5.81853020071,
        9.36938509649,
        15.7275288623,
        28.1000420415,
        53.1107291436,
        90.0}; // sensorfocus of 25
    trackSens[index] = new Sensors(car, 19);
    for (int i = 0; i < 19; ++i) {
        trackSens[index]->setSensor(i,angles[i],__SENSORS_RANGE__);
    }
    oppSens[index] = new ObstacleSensors(36, curTrack, car, s, (int) __SENSORS_RANGE__);
    prevDist[index]=-1.0;
    distRaced[index]=0.0;
    // /edit daan
    driver[index]->newRace(car, s);
}


// Drive during race.
static void drive(int index, tCarElt* car, tSituation *s)
{
    // edit daan
    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI
    trackSens[index]->sensors_update();
    oppSens[index]->sensors_update(s);

    if (prevDist[index]<0.0)
    {
        prevDist[index] = car->race.distFromStartLine;
    }
    float curDistRaced = car->race.distFromStartLine - prevDist[index];
    prevDist[index] = car->race.distFromStartLine;
    if (curDistRaced>100.0)
    {
        curDistRaced -= curTrack->length;
    }
    if (curDistRaced<-100.0)
    {
        curDistRaced += curTrack->length;
    }

    distRaced[index] += curDistRaced;
    // /edit daan

    driver[index]->drive(s);

    // edit daan
    oldAccel[index] = car->_accelCmd;
    oldBrake[index] = car->_brakeCmd;
    oldGear[index]  = car->_gearCmd;
    oldSteer[index] = car->_steerCmd;
    oldClutch[index] = car->_clutchCmd;
    oldFocus[index] = car->_focusCmd;//ML

    fs << car->_accelCmd << ";";
    fs << car->_brakeCmd << ";";
    fs << car->_steerCmd << ";";
    fs << angle << ";";
    fs << float(car->_curLapTime) << ";";
    fs << car->race.distFromStartLine << ";";
    fs << float(distRaced[index]) << ";";
    fs << car->_gear << ";";
    fs << float(car->_lastLapTime) << ";";
    fs << car->race.pos << ";";
    fs << car->_enginerpm*10 << ";";
    fs << float(car->_speed_x  * 3.6) << ";";
    fs << float(car->_speed_y  * 3.6) << ";";
    fs << float(car->_speed_z  * 3.6) << ";";
    fs << dist_to_middle << ";";
    fs << car->_pos_Z << ";";
    for (int i=0; i<4; i++) {
        fs << car->_wheelSpinVel(i) << ";";
    }
    if (dist_to_middle<=1.0 && dist_to_middle >=-1.0 ) {
        for (int i=0; i<19; i++) {
            fs << trackSens[index]->getSensorOut(i) << ";";
        }
    } else {
        for (int i=0; i<19; i++) {
            fs << float(-1) << ";";
        }
    }
    for (int i=0; i<36; i++) {
        fs << oppSens[index]->getObstacleSensorOut(i) << ";";
    }
    fs << std::endl;
    // /edit daan
}


// Pitstop callback.
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index]->pitCommand(s);
}


// End of the current race.
static void endRace(int index, tCarElt *car, tSituation *s)
{
    // edit daan
    if (fs.is_open())
        fs.close();
    // /edit daan
    driver[index]->endRace(s);
}


// Called before the module is unloaded.
static void shutdown(int index)
{
    // edit daan
    if (fs.is_open())
        fs.close();
    // /edit daan
    delete driver[index];
}

