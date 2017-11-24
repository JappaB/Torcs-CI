/***************************************************************************

    file                 : manual_override.cpp
    created              : Sat Feb 07 19:53:00 CET 2015
    copyright            : (C) 2015 by Andrew Sumner
    email                : novocas7rian@gmail.com
    version              : $Id: manual_override.cpp,v 1.0 2015/02/07 20:11:49 andrew Exp $

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
#include <string.h>
#include <stdlib.h>

#include <portability.h>

#include "manual_override.h"
#include "vardef.h"

#define SECT_OVERRIDE "overrides"

LManualOverride::LManualOverride(int DivCount, const char *theLabel)
{
    overrideValues = (OverrideValue *)malloc(20 * sizeof(OverrideValue));
    overrideValueCount = 0;
    overrideValueAlloc = 20;
    divCount = DivCount;
    label = theLabel;
}

LManualOverride::~LManualOverride()
{
    free(overrideValues);
}

void LManualOverride::loadFromFile(char **file_contents, int linecount)
{
    char testlabel[64];
    overrideValueCount = 0;

    snprintf(testlabel, 63, "%s<", label);

    for (int line=0; line<linecount; line++)
    {
        char *text = file_contents[line];
        if (isHeading(text))
        {
            if (!strcmp(text, testlabel) && line < linecount-1)
            {
                line++;
                char *workingText = (char *) malloc(strlen(file_contents[line])+1);
                strcpy(workingText, file_contents[line]);
                int sdiv = 0, ediv = 0;
                double value = 0.0;
                if (readValues(workingText, &sdiv, &ediv, &value))
                {
                    if (overrideValueCount >= overrideValueAlloc)
                    {
                        overrideValueAlloc *= 2;
                        overrideValues = (OverrideValue *)realloc(overrideValues, overrideValueAlloc * sizeof(OverrideValue));
                    }
                    overrideValues[overrideValueCount].startdiv = sdiv;
                    overrideValues[overrideValueCount].enddiv = ediv;
                    overrideValues[overrideValueCount].value = value;
                    overrideValueCount++;
                }
                free(workingText);
            }
        }
    }
}

bool LManualOverride::readValues(char *text, int *sdiv, int *ediv, double *value)
{
    bool success = false;
    *value = -1000.0;
    *sdiv = *ediv = -1000;

    char *token = strtok(text, " ");
    if (token)
    {
        *sdiv = atoi(token);
        token = strtok(NULL, " ");
        if (token)
        {
            *ediv = atoi(token);
            token = strtok(NULL, " ");
            if (token)
            {
                *value = atof(token);
                success = true;
            }
        }
    }
    return success;
}

char *LManualOverride::readInteger(char *text, int *value)
{
    char *pstart = text, *pend = text, *plimit = text + (strlen(text)-1);

    while (*pend && *pend != ' ')
        pend++;

    if (pend > pstart)
    {
        *pend = 0;
        *value = atoi(pstart);
        pend++;
    }
    else
    {
        *value = -1000;
        return NULL;
    }

    while (*pend == ' ')
    {
        *pend = 0;
        pend++;
    }

    if (pend > plimit)
        pend = plimit;

    return pend;
}

char *LManualOverride::readDouble(char *text, double *value)
{
    char *pstart = text, *pend = text, *plimit = text + (strlen(text)-1);

    while (*pend && *pend != ' ')
        pend++;

    if (pend > pstart)
    {
        *pend = 0;
        *value = atof(pstart);
        pend++;
    }
    else
    {
        *value = -1000.0;
        return NULL;
    }

    while (*pend == ' ')
    {
        *pend = 0;
        pend++;
    }

    if (pend > plimit)
        pend = plimit;

    return pend;
}

bool LManualOverride::isHeading(char *text)
{
    int len = strlen(text);
    if (len > 0)
    {
        if (text[len-1] == '<')
        {
            return true;
        }
    }
    return false;
}

void LManualOverride::saveToFile(FILE *filepointer)
{
    if (!filepointer)
        return;

    char str[128];

    for (int i=0; i<overrideValueCount; i++)
    {
        snprintf(str, 127, "%s<\n", label);
        fprintf(filepointer, str);
        snprintf(str, 127, "%d %d %.4f\n\n", overrideValues[i].startdiv, overrideValues[i].enddiv, overrideValues[i].value);
        fprintf(filepointer, str);
    }
}

bool LManualOverride::getOverrideValue(int div, double *value)
{
    for (int i=0; i<overrideValueCount; i++)
    {
        if (divInRange(div, overrideValues[i].startdiv, overrideValues[i].enddiv))
        {
            *value = overrideValues[i].value;
            return true;
        }
    }
    return false;
}

void LManualOverride::setOverrideValue(int startdiv, int enddiv, double value)
{
    if (overrideValueCount >= overrideValueAlloc)
    {
        overrideValueAlloc *= 2;
        overrideValues = (OverrideValue *)realloc(overrideValues, overrideValueAlloc * sizeof(OverrideValue));
    }

    overrideValues[overrideValueCount].startdiv = startdiv;
    overrideValues[overrideValueCount].enddiv = enddiv;
    overrideValues[overrideValueCount].value = value;
    overrideValueCount++;
}


bool LManualOverride::divInRange(int div, int startdiv, int enddiv)
{
    if (startdiv < enddiv)
    {
        if (div >= startdiv && div <= enddiv)
            return true;
    }
    else
    {
        if (div >= startdiv || div <= enddiv)
            return true;
    }
    return false;
}

static const char *overrideLabels[64] = { 
    PRV_CORNERSPEED, 
    PRV_CORNERSPEED_MID, 
    PRV_CORNERSPEED_SLOW, 
    PRV_BRAKEDELAY, 
    PRV_BRAKEDELAY_MID, 
    PRV_BRAKEDELAY_SLOW, 
    PRV_RIGHTCORNERSPEED, 
    PRV_LEFTCORNERSPEED, 
    PRV_AVOIDBRAKEDELAY, 
    PRV_RACELINECURVE, 
    PRV_BUMP_CAUTION, 
    PRV_LEFT_BUMP_CAUTION, 
    PRV_RIGHT_BUMP_CAUTION, 
    PRV_AVOID_BUMPCAUTION, 
    PRV_SLOPE_FACTOR, 
    PRV_AVOID_SLOPE, 
    PRV_LEFT_MARGIN, 
    PRV_RL_LEFT_MARGIN, 
    PRV_LEFT_MARGIN_MID, 
    PRV_LEFT_MARGIN_SLOW, 
    PRV_RL_RIGHT_MARGIN,
    PRV_RIGHT_MARGIN,
    PRV_RIGHT_MARGIN_MID,
    PRV_RIGHT_MARGIN_SLOW,
    PRV_MAX_SPEED,
    PRV_OUTSIDE_DAMPENER,
    PRV_PREFERRED_SIDE,
    PRV_LOOKAHEAD,
    PRV_LEFT_OVERTAKE,
    PRV_RIGHT_OVERTAKE,
    PRV_OVERTAKE,
    PRV_TRANSITION_INC,
    PRV_LFT_TRANS_INC,
    PRV_RGT_TRANS_INC,
    PRV_RL_FOR_OVERTAKE,
    PRV_LOOKAHEAD_LEFT,
    PRV_LOOKAHEAD_RIGHT,
    PRV_BRAKE_COEFFICIENT,
    PRV_OVERTAKE_SPD_DIFF,
    PRV_STAY_INSIDE,
    PRV_COLLBRAKE_TIMPACT,
    ""
};

LManualOverrideCollection::LManualOverrideCollection(char *trackname, const char *carname, int DivCount)
{
    int len = strlen(carname);
    carName = (char *) malloc(len+1);
    strcpy(carName, carname);
    len = strlen(trackname);
    trackName = (char *) malloc(len+1);
    strcpy(trackName, trackname);
    if (len > 4)
        trackName[len-4] = 0;
    divCount = DivCount;

    overrides = (LManualOverride **) malloc(OVERRIDE_COUNT * sizeof(LManualOverride *));
    for (int i=0; i<OVERRIDE_COUNT; i++)
        overrides[i] = new LManualOverride(DivCount, overrideLabels[i]);
}

LManualOverrideCollection::~LManualOverrideCollection()
{
    free(trackName);
    free(carName);
    for (int i=0; i<OVERRIDE_COUNT; i++)
        delete overrides[i];
    free(overrides);
}

void LManualOverrideCollection::loadFromFile()
{
    char buffer[1025];
    snprintf(buffer, 1024, "%sdrivers/%s/%s/%s.dat", GetDataDir(), BOT_NAME, carName, trackName); 
    FILE *filepointer = fopen(buffer, "r");

    if (!filepointer)
    {
        fprintf(stderr, "Unable to open data file %s\n", buffer);
        fflush(stderr);
        return;
    }

    int linecount = 1, i;
    while (fgets(buffer, 1022, filepointer) != NULL)
    {
        linecount++;
    }

    if (linecount)
    {
        fseek(filepointer, 0, SEEK_SET);
        char **file_contents = (char **) malloc(linecount * sizeof(char *));

        linecount = 0;
        while (fgets(buffer, 1022, filepointer) != NULL)
        {
            file_contents[linecount] = (char *) malloc(strlen(buffer)+1);
            strcpy(file_contents[linecount], buffer);
            removeNewLineCharacters(file_contents[linecount]);
            linecount++;
            file_contents[linecount] = NULL;
        }

        for (i=0; i<OVERRIDE_COUNT; i++)
        {
            overrides[i]->loadFromFile(file_contents, linecount);
        }

        for (i=0; i<linecount; i++)
            if (file_contents[i])
                free(file_contents[i]);
        free(file_contents);
        fprintf(stderr, "%d lines loaded from data file\n", linecount);
        fflush(stderr);
    }

    fclose(filepointer);
}

void LManualOverrideCollection::saveToFile()
{
    char buffer[1025];
    snprintf(buffer, 1024, "%sdrivers/%s/%s/%s.dat_save", GetDataDir(), BOT_NAME, carName, trackName); 
    FILE *filepointer = fopen(buffer, "w");

    if (filepointer)
    {
        for (int i=0; i<OVERRIDE_COUNT; i++)
        {
            overrides[i]->saveToFile(filepointer);
        }

        fclose(filepointer);
    }
}

LManualOverride *LManualOverrideCollection::getOverrideForLabel(char *label)
{
    if (label)
    {
        for (int i=0; i<OVERRIDE_COUNT; i++)
        {
            if (!strcmp(label, overrideLabels[i]))
                return overrides[i];
        }
    }
    return NULL;
}

void LManualOverrideCollection::removeNewLineCharacters(char *text)
{
    char *p = text + (strlen(text)-1);
    while (p >= text && (*p == 13 || *p == 10 || *p == ' ' || *p == '\t'))
    {
        *p = 0;
        p--;
    }
}

