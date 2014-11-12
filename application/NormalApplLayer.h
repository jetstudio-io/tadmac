/***************************************************************************
 * file:        SensorApplLayer.h
 *
 * author:      Amre El-Hoiydi, Jérôme Rousselot
 *
 * copyright:   (C) 2007 CSEM
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 * description: Generate periodic traffic addressed to a sink
 **************************************************************************/

#ifndef NORMAL_APPL_LAYER_H
#define NORMAL_APPL_LAYER_H

#include <map>

#include "MiXiMDefs.h"
#include "BaseModule.h"
#include "BaseLayer.h"
#include "Packet.h"
#include "SimpleAddress.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <random>
#include <cmath>

class BaseWorldUtility;

/**
 * @brief Test class for the application layer
 *
 * All nodes periodically generate a packet addressed to a sink.
 * This class takes three arguments:
 * - packets: the number of packets to be sent by this application.
 * - trafficType: affects how the time interval between two packets
 * 				is generated. Possible values are "periodic", "uniform",
 * 				and "exponential".
 * - trafficParam: parameter for traffic type. For "periodic" traffic
 * 					this value is the constant time interval, for
 * 					"uniform" traffic this is the maximum time interval
 * 					between two packets and for "exponential" it is
 * 					the mean value. These values are expressed in seconds.
 *
 * @ingroup applLayer
 * @author Amre El-Hoiydi, Jérôme Rousselot
 **/
class MIXIM_API NormalApplLayer : public BaseLayer
{
    private:
        /** @brief Copy constructor is not allowed.
         */
        NormalApplLayer(const NormalApplLayer&);
        /** @brief Assignment operator is not allowed.
         */
        NormalApplLayer& operator=(const NormalApplLayer&);

    public:
        /** @brief Initialization of the module and some variables*/
        virtual void initialize(int);
        virtual void finish();

        virtual ~NormalApplLayer();

        NormalApplLayer() :
                BaseLayer(), delayTimer(NULL), myAppAddr(), destAddr(), sentPackets(0), initializationTime(), firstPacketGeneration(), lastPacketReception(), trafficType(
                        0), trafficParam(0.0), trafficStability(0.1), nbPackets(0), nbPacketsSent(0), nbPacketsReceived(0), stats(false), trace(
                        false), debug(false), broadcastPackets(false), latencies(), latency(), latenciesRaw(), packet(
                        100), headerLength(0), world(NULL), dataOut(0), dataIn(0), ctrlOut(0), ctrlIn(0)
        {
        } // we must specify a packet length for Packet.h

        enum APPL_MSG_TYPES
        {
            SEND_DATA_TIMER, STOP_SIM_TIMER, DATA_MESSAGE
        };

        enum TRAFFIC_TYPES
        {
            UNKNOWN = 0, NORMAL, PERIODIC, UNIFORM, EXPONENTIAL, NB_DISTRIBUTIONS,
        };

    protected:
        cMessage * delayTimer;
        LAddress::L3Type myAppAddr;
        LAddress::L3Type destAddr;
        int sentPackets;
        simtime_t initializationTime;
        simtime_t firstPacketGeneration;
        simtime_t lastPacketReception;
        // parameters:
        int trafficType;
        double trafficParam;
        double trafficStability;
        int nbPackets;
        long nbPacketsSent;
        long nbPacketsReceived;
        bool stats;
        bool trace;
        bool debug;
        bool broadcastPackets;
        std::map<LAddress::L3Type, cStdDev> latencies;
        cStdDev latency;
        cOutVector latenciesRaw;
        Packet packet; // informs the simulation of the number of packets sent and received by this node.
        int headerLength;
        BaseWorldUtility* world;

        std::random_device rd;
        std::mt19937 gen;
        std::normal_distribution<> dist;

    protected:
        // gates
        int dataOut;
        int dataIn;
        int ctrlOut;
        int ctrlIn;

        /** @brief Handle self messages such as timer... */
        virtual void handleSelfMsg(cMessage *);

        /** @brief Handle messages from lower layer */
        virtual void handleLowerMsg(cMessage *);

        virtual void handleLowerControl(cMessage * msg);

        virtual void handleUpperMsg(cMessage * m)
        {
            delete m;
        }

        virtual void handleUpperControl(cMessage * m)
        {
            delete m;
        }

        /** @brief send a data packet to the next hop */
        virtual void sendData();

        /** @brief Recognize distribution name. Redefine this method to add your own distribution. */
        virtual void initializeDistribution(const char*);

        /** @brief calculate time to wait before sending next packet, if required. You can redefine this method in a subclass to add your own distribution. */
        virtual void scheduleNextPacket();

        /**
         * @brief Returns the latency statistics variable for the passed host address.
         * @param hostAddress the address of the host to return the statistics for.
         * @return A reference to the hosts latency statistics.
         */
        cStdDev& hostsLatency(const LAddress::L3Type& hostAddress);
};

#endif
