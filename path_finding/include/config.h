#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include "tinyxml2.h"
#include <string>
#include "gl_const.h"
#include <algorithm>
#include <sstream>


class Config
{
public:
    Config();
    int loglevel;
    int connectedness;
    bool allowanyangle;
    bool planforturns;
    double timelimit;
    int rescheduling;
    double inflatecollisionintervals;
    int initialprioritization;
    double startsafeinterval;
    double additionalwait;
    std::string logfilename;
    std::string logpath;

    bool getConfig(const char* fileName);
};

#endif
