#include "hedge_pos_publisher/hedgehog_proxy.h"

HedgehogProxy::HedgehogProxy(std::string ttyFileName, uint32_t baudRate, bool verbose) 
{
    puts("Start proxy...\n");
    
    hedge_ = createMarvelmindHedge();

    if (!hedge_)
    {
        puts ("Error: Unable to create MarvelmindHedge");
        return;
    }

    hedge_->ttyFileName = ttyFileName.c_str();
    hedge_->baudRate = baudRate;
    hedge_->verbose = verbose; // show errors and warnings
    startMarvelmindHedge(hedge_);
}

HedgehogProxy::~HedgehogProxy()
{
    if (hedge_)
    {
        stopMarvelmindHedge(hedge_);
        destroyMarvelmindHedge(hedge_);
        if (hedge_) delete hedge_;
        hedge_ = 0;
    }
}

PositionValue HedgehogProxy::get_position()
{
    struct PositionValue position;
    position.ready = false;

    while (!position.ready) 
    {
        bool onlyNew = true;
       
        if (hedge_->haveNewValues_ || (!onlyNew))
        {        
            uint8_t real_values_count = hedge_->maxBufferedPositions;
            uint8_t addresses[real_values_count];
            uint8_t addressesNum = 0;

            for (uint8_t i = 0; i < real_values_count; ++i)
            {
                uint8_t address = hedge_->positionBuffer[i].address;
                bool alreadyProcessed = false;

                if (addressesNum != 0)
                {
                    for (uint8_t j = 0; j < addressesNum; ++j)
                    {
                        if (address == addresses[j])
                        {
                            alreadyProcessed = true;
                            break;
                        }
                    }
                }

                if (alreadyProcessed)
                    continue;

                addresses[addressesNum++] = address;
                getPositionFromMarvelmindHedgeByAddress(hedge_, &position, address);

                /* coordinates fo debug
                double xm = ((double) position.x) / 1000.0;
                double ym = ((double) position.y) / 1000.0;
                double zm = ((double) position.z) / 1000.0;
                */
                // printf("Address: %d", position.address);
                if (position.ready)
                {   
                    position.address = address;
                    return position;
                }

                hedge_->haveNewValues_=false;
            }
        }
    }
    return position;
}

bool HedgehogProxy::haveNewValues() {
    return hedge_->haveNewValues_;
}

bool HedgehogProxy::terminationRequired()
{ 
    return hedge_->terminationRequired; 
}   


