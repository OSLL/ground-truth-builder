#pragma once

#ifndef HEDGEHOG_PROXY_H
#define HEDGEHOG_PROXY_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <unistd.h>
#include <signal.h>
#include <string>

#include <stdio.h>
#include "marvelmind.h"

class HedgehogProxy
{
public:

    HedgehogProxy(std::string ttyFileName = DEFAULT_TTY_FILENAME, uint32_t baudRate = 9600, bool verbose = true);
    ~HedgehogProxy();

    PositionValue get_position();

    bool haveNewValues();
    bool terminationRequired();

private:

    struct MarvelmindHedge* hedge_;
};

#endif // HEDGEHOG_PROXY_H

