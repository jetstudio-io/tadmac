//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

/**
 * Version 1.1: support multi senders
 * Version 2.0: WB is broadcast
 */

#include "FTAMacLayer.h"

#include <sstream>
#include <cassert>

#include "FWMath.h"
#include "MacToPhyControlInfo.h"
#include "MacToNetwControlInfo.h"
#include "BaseArp.h"
#include "BaseConnectionManager.h"
#include "PhyUtils.h"
#include "MacToPhyInterface.h"
#include <stdlib.h>
#include <time.h>
#include <MacPkt_m.h>

Define_Module(FTAMacLayer)

/**
 * Initialize method of FTAMacLayer. Init all parameters, schedule timers.
 */
void FTAMacLayer::initialize(int stage) {
    BaseMacLayer::initialize(stage);

    if (stage == 0) {
        srand(time(NULL));
        BaseLayer::catDroppedPacketSignal.initialize();

        /* get sepecific parameters for TADMAC */
        role = static_cast<ROLES>(hasPar("role") ? par("role") : 1);

        wakeupInterval = hasPar("WUIInit") ? par("WUIInit") : 0.1;
        waitCCA = hasPar("waitCCA") ? par("waitCCA") : 0.005;
        waitWB = hasPar("waitWB") ? par("waitWB") : 0.25;
        waitACK = hasPar("waitACK") ? par("waitACK") : 0.01;
        waitDATA = hasPar("waitDATA") ? par("waitDATA") : 0.01;
        sysClock = hasPar("sysClock") ? par("sysClock") : 0.001;
        sysClockFactor = hasPar("sysClockFactor") ? par("sysClockFactor") : 75;
        alpha = hasPar("alpha") ? par("alpha") : 0.5;
        numberSender = hasPar("numberSender") ? par("numberSender") : 1;

        queueLength = hasPar("queueLength") ? par("queueLength") : 8;
        animation = hasPar("animation") ? par("animation") : true;
        bitrate = hasPar("bitrate") ? par("bitrate") : 250000.;
        headerLength = hasPar("headerLength") ? par("headerLength") : 10.;
        txPower = hasPar("txPower") ? par("txPower") : 50.;
        useMacAcks = hasPar("useMACAcks") ? par("useMACAcks") : false;
        maxTxAttempts = hasPar("maxTxAttempts") ? par("maxTxAttempts") : 2;

        stats = par("stats");
        nbTxDataPackets = 0;
        nbTxWB = 0;
        nbRxDataPackets = 0;
        nbRxWB = 0;
        nbMissedAcks = 0;
        nbRecvdAcks = 0;
        nbDroppedDataPackets = 0;
        nbTxAcks = 0;

        txAttempts = 0;
        lastDataPktDestAddr = LAddress::L2BROADCAST;
        lastDataPktSrcAddr = LAddress::L2BROADCAST;

        macState = INIT;

        // init the dropped packet info
        droppedPacket.setReason(DroppedPacket::NONE);
        nicId = getNic()->getId();
        WATCH(macState);
    } else if (stage == 1) {

        if (role == NODE_RECEIVER) {
            /**
             * Initialization of events for recevier
             */
            start = new cMessage("start");
            start->setKind(START);

            wakeup = new cMessage("WAKE_UP");
            wakeup->setKind(WAKE_UP);

            ccaWBTimeout = new cMessage("CCA_WB_TIMEOUT");
            ccaWBTimeout->setKind(CCA_WB_TIMEOUT);

            WBsent = new cMessage("WB_SENT");
            WBsent->setKind(WB_SENT);

            rxDATATimeout = new cMessage("RX_DATA_TIMEOUT");
            rxDATATimeout->setKind(RX_DATA_TIMEOUT);

            DATAreceived = new cMessage("DATA_RECEIVED");
            DATAreceived->setKind(DATA_RECEIVED);

            ccaACKTimeout = new cMessage("CCA_ACK_TIMEOUT");
            ccaACKTimeout->setKind(CCA_ACK_TIMEOUT);

            ACKsent = new cMessage("ACK_SENT");
            ACKsent->setKind(ACK_SENT);

            int nodeIdx = getNode()->getIndex();
            TSR_length = 8;
            // allocate memory & initialize for TSR bank
            TSR_bank = new int*[numberSender+1];
            for (int i = 1; i <= numberSender; i++) {
                TSR_bank[i] = new int[TSR_length];
                for (int j = 0; j < TSR_length; j++) {
                    TSR_bank[i][j] = 0;
                }
            }
            /**
             * Define route table here. Because we don't use high level so we need to fix the network topology
             * node[0] is receiver, mac address is 00:00:00:00:00:00
             * node[1->4] is sender, mac address is from 00:00:00:00:00:01 to 00:00:00:00:00:04
             * node[5] is receiver, mac address is 00:00:00:00:00:05
             * node[6->9] is sender, mac address is from 00:00:00:00:00:06 to 00:00:00:00:00:09
             */
//            routeTable = new LAddress::L2Type[numberSender+1];
//            for (int i = 1; i <= numberSender; i++) {
//                ostringstream converter;
//                converter << "00:00:00:00:00:0" << (i + nodeIdx);
//                routeTable[i].setAddress(converter.str().c_str());
////                routeTable[i] = i;
//            }
            // allocate memory & initialize for nodeWakeupInterval & nextWakeupIntervalTime
            nodeWakeupInterval = new double[numberSender+1];
            nodeWakeupIntervalLock = new double[numberSender+1];
            nodeSumWUInt = new double[numberSender+1];
            nextWakeupTime = new simtime_t[numberSender+1];
            for (int i = 1; i <= numberSender; i++) {
                nodeWakeupInterval[i] = wakeupInterval;
                nodeWakeupIntervalLock[i] = 0.0;
                nextWakeupTime[i] = 0.0;
                nodeSumWUInt[i] = 0.0;
//                nextWakeupTime[i] = (rand() % 1000 + 1) / 1000.0;
//                nextWakeupTime[i] = (100 * i) / 1000.0;
                //cout << nextWakeupTime[i] << endl;
            }

            // allocate memory & initialize for nodeIdle
            nodeIdle = new double*[numberSender+1];
            nodeIndex = new int[numberSender+1];
            nodeNumberWakeup = new int[numberSender+1];
            nodeFirstTime = new int[numberSender+1];
            nodePriority = new int[numberSender+1];
            nodeCollision = new int[numberSender+1];
            nodeChoosen = new int[numberSender+1];
            nodeBroken = new int[numberSender+1];
            for (int i = 1; i <= numberSender; i++) {
                nodePriority[i] = 0;
                nodeIndex[i] = 0;
                nodeNumberWakeup[i] = 0;
                nodeFirstTime[i] = 1;
                nodeIdle[i] = new double[2];
                nodeIdle[i][0] = nodeIdle[i][1] = 0;
                nodeCollision[i] = 0;
                nodeChoosen[i] = 0;
                nodeBroken[i] = 0;
            }
        } else {
            /**
             * Initialization of events for sender
             */
            start = new cMessage("start");
            start->setKind(START);

            wakeupDATA = new cMessage("WAKE_UP_DATA");
            wakeupDATA->setKind(WAKE_UP_DATA);

            rxWBTimeout = new cMessage("RX_WB_TIMEOUT");
            rxWBTimeout->setKind(RX_WB_TIMEOUT);

            WBreceived = new cMessage("WB_RECEIVED");
            WBreceived->setKind(WB_RECEIVED);

            ccaDATATimeout = new cMessage("CCA_DATA_TIMEOUT");
            ccaDATATimeout->setKind(CCA_DATA_TIMEOUT);

            DATAsent = new cMessage("DATA_SENT");
            DATAsent->setKind(DATA_SENT);

            waitACKTimeout = new cMessage("WAIT_ACK_TIMEOUT");
            waitACKTimeout->setKind(WAIT_ACK_TIMEOUT);

            ACKreceived = new cMessage("ACK_RECEIVED");
            ACKreceived->setKind(ACK_RECEIVED);
        }
        lastWakeup = 0;
        iwuVec.setName("Iwu");
        nbCollision = 0;
        numberWakeup = 0;
        scheduleAt(0.0, start);
    }
}

FTAMacLayer::~FTAMacLayer() {

    if (role == NODE_RECEIVER) {
        cancelAndDelete(start);
        cancelAndDelete(wakeup);
        cancelAndDelete(ccaWBTimeout);
        cancelAndDelete(WBsent);
        cancelAndDelete(rxDATATimeout);
        cancelAndDelete(DATAreceived);
        cancelAndDelete(ccaACKTimeout);
        cancelAndDelete(ACKsent);
    } else {
        cancelAndDelete(start);
        cancelAndDelete(wakeupDATA);
        cancelAndDelete(rxWBTimeout);
        cancelAndDelete(WBreceived);
        cancelAndDelete(ccaDATATimeout);
        cancelAndDelete(DATAsent);
        cancelAndDelete(waitACKTimeout);
        cancelAndDelete(ACKreceived);
    }

    MacQueue::iterator it;
    for (it = macQueue.begin(); it != macQueue.end(); ++it) {
        delete (*it);
    }
    macQueue.clear();
}

void FTAMacLayer::finish() {
    BaseMacLayer::finish();

    // record stats
    if (stats) {
        recordScalar("nbTxDataPackets", nbTxDataPackets);
        recordScalar("nbTxPreambles", nbTxWB);
        recordScalar("nbRxDataPackets", nbRxDataPackets);
        recordScalar("nbRxPreambles", nbRxWB);
        recordScalar("nbMissedAcks", nbMissedAcks);
        recordScalar("nbRecvdAcks", nbRecvdAcks);
        recordScalar("nbTxAcks", nbTxAcks);
        recordScalar("numberWakeup", numberWakeup);
        recordScalar("nbCollision", nbCollision);
        if (role == NODE_RECEIVER) {
        }
    }
}

/**
 * Attaches a "control info" (MacToNetw) structure (object) to the message pMsg.
 */
cObject* FTAMacLayer::setUpControlInfo(cMessage * const pMsg, const LAddress::L2Type& pSrcAddr)
{
    return MacToNetwControlInfo::setControlInfo(pMsg, pSrcAddr);
}

bool FTAMacLayer::addToQueue(cMessage * msg) {
    if (macQueue.size() >= queueLength) {
        // queue is full, message has to be deleted
        debugEV << simTime() << ":New packet arrived, but queue is FULL, so new packet is"
                  " deleted\n";
        msg->setName("MAC ERROR");
        msg->setKind(PACKET_DROPPED);
        sendControlUp(msg);
        droppedPacket.setReason(DroppedPacket::QUEUE);
        emit(BaseLayer::catDroppedPacketSignal, &droppedPacket);
        nbDroppedDataPackets++;

        return false;
    }

    macpkt_ptr_t macPkt = new MacPkt(msg->getName());
    macPkt->setBitLength(headerLength);
    cObject *const cInfo = msg->removeControlInfo();
    //EV<<"CSMA received a message from upper layer, name is "
    //  << msg->getName() <<", CInfo removed, mac addr="
    //  << cInfo->getNextHopMac()<<endl;
    macPkt->setDestAddr(getUpperDestinationFromControlInfo(cInfo));
    delete cInfo;
    macPkt->setSrcAddr(myMacAddr);

    assert(static_cast<cPacket*>(msg));
    macPkt->encapsulate(static_cast<cPacket*>(msg));

    macQueue.push_back(macPkt);
    return true;
}

/**
 * Receipt data from upper layer -> stock in queue -> wake up to send data if in sleep state
 * If already awake, change to Tx state -> send data
 */
void FTAMacLayer::handleUpperMsg(cMessage *msg) {
    // If this is receiver, we dont need to handle upper layer packet
    if (role == NODE_RECEIVER) {
        delete msg;
        return;
    }
    addToQueue(msg);
    // force wakeup now
    if (macState == SLEEP)
    {
        if (wakeupDATA->isScheduled()) {
            cancelEvent(wakeupDATA);
        }
        scheduleAt(simTime(), wakeupDATA);
    }
}

/**
 * /!\ NOTE: This function is used only in recevier
 * Find next wakeup time
 * Change physic layer state
 * Change node color
 * Change state to SLEEP
 */
void FTAMacLayer::scheduleNextWakeup() {
    simtime_t nextWakeup = 100000;
    for (int i = 1; i <= numberSender; i++) {
        // if already pass the wakeup moment for this node
        // -> calculate nextwakeup for this node with 0 in TSR
        if (nextWakeupTime[i] < simTime()) {
            calculateNextInterval();
        }
        if (nextWakeupTime[i] < nextWakeup) {
            nextWakeup = nextWakeupTime[i];
            currentNode = i;
        }
    }
    scheduleAt(nextWakeup, wakeup);
}

void FTAMacLayer::writeLog() {
}

void FTAMacLayer::changeMACState() {
    switch (macState) {
        case SLEEP:
            // change icon to black light -> note is inactive
            changeDisplayColor(BLACK);
            // Change antenna to sleep state
            phy->setRadioState(MiximRadio::SLEEP);
            break;
        case WAIT_WB:
        case CCA_DATA:
        case WAIT_ACK:
        case CCA_WB:
        case WAIT_DATA:
        case CCA_ACK:
            // change icon to green light -> note is wait for sign
            changeDisplayColor(GREEN);
            // set antenna to receiving sign state
            phy->setRadioState(MiximRadio::RX);
            break;
        case SENDING_DATA:
        case SENDING_WB:
        case SENDING_ACK:
            changeDisplayColor(YELLOW);
            // set antenna to sending sign state
            phy->setRadioState(MiximRadio::TX);
    }
}

/**
 * Handle message in sender
 *
 */
void FTAMacLayer::handleSelfMsgSender(cMessage *msg) {
    switch (macState) {
        // Call at first time after initialize the note
        case INIT:
            if (msg->getKind() == START) {
                macState = SLEEP;
                changeMACState();
                lastWakeup = simTime();
                wbMiss = 0;
                // Sender don't need to schedule wakeup because it wakeup to send data
                return;
            }
            break;
        // This node is sleeping & have data to send
        case SLEEP:
            if (msg->getKind() == WAKE_UP_DATA) {
//                cout << simTime() << ":SLEEP & WAKE_UP_DATA" << endl;
                // MAC state is CCA
                macState = WAIT_WB;
                changeMACState();
                // schedule the event wait WB timeout
                scheduleAt(simTime() + waitWB, rxWBTimeout);
                // store the moment that this node is wake up
                startWake = simTime();
                // reset number resend data
                txAttempts = 0;
                numberWakeup++;
                iwuVec.recordWithTimestamp(simTime(), simTime() - lastWakeup);
                lastWakeup = simTime();
                return;
            }
            break;
        // The sender is in state WAIT_WB & receive a message
        case WAIT_WB:
            // If this message is event
            if (msg->getKind() == RX_WB_TIMEOUT) {
                // Turn back to SLEEP state
                macState = SLEEP;
                changeMACState();
                wbMiss++;
                return;
            }
            // duration the WAIT_WB, received the WB message -> change to CCA state & schedule the timeout event
            if (msg->getKind() == WB) {
                macpkt_ptr_t            mac  = static_cast<macpkt_ptr_t>(msg);
                const LAddress::L2Type& dest = mac->getDestAddr();
                // Do nothing if receive WB for other node
                if (dest != myMacAddr) {
                    mac = NULL;
                    // Drop this message
                    delete msg;
                    return;
                }
                //cout << "sender receipt wb" << endl;
                // Receiver is the node which send WB packet
                receiverAddress = mac->getSrcAddr();
                nbRxWB++;
                macState = CCA_DATA;
                changeMACState();
                // Don't need to call the event to handle WB timeout
                cancelEvent (rxWBTimeout);
                // schedule the CCA timeout event
                scheduleAt(simTime() + waitCCA, ccaDATATimeout);
                // log the time wait for WB
                timeWaitWB = simTime() - startWake;
                // reset ccaAttempts
                ccaAttempts = 0;
                mac = NULL;
                // Drop this message
                delete msg;
                msg = NULL;
                return;
            }
            break;

        case CCA_DATA:
            if (msg->getKind() == CCA_DATA_TIMEOUT) {
                macState = SENDING_DATA;
                changeMACState();
                // change mac state to send data
                // in this case, we don't need to schedule the event to handle when data is sent
                // this event will be call when the physic layer finished
                return;
            }

            break;

        case SENDING_DATA:
            // Finish send data to receiver
            if (msg->getKind() == DATA_SENT) {
                macState = WAIT_ACK;
                changeMACState();
                // schedule the event wait WB timeout
                scheduleAt(simTime() + waitACK, waitACKTimeout);

                return;
            }
            break;

        case WAIT_ACK:
            if (msg->getKind() == WAIT_ACK_TIMEOUT) {
                // if the number resend data is not reach max time
                if (txAttempts < maxTxAttempts) {
                    macState = WAIT_WB;
                    changeMACState();
                    txAttempts++;

                    // schedule the event wait WB timeout
                    scheduleAt(simTime() + waitWB, rxWBTimeout);
                    // store the moment to wait WB
                    timeWaitWB = simTime();
                    nbMissedAcks++;
                } else {
                    macState = SLEEP;
                    changeMACState();
                    nbMissedAcks++;
                }
                return;
            }
            // received ACK -> change to sleep, schedule next wakeup time
            if (msg->getKind() == ACK_RECEIVED || msg->getKind() == ACK) {
                //cout << "sender receipt ack -> sleep" << endl;
                macState = SLEEP;
                changeMACState();
                //remove event wait ack timeout
                cancelEvent(waitACKTimeout);
                // Remove packet in queue
                delete macQueue.front();
                macQueue.pop_front();
                //Delete ACK
                delete msg;
                msg = NULL;
                return;
            }
            if (msg->getKind() == DATA || msg->getKind() == WB) {
                delete msg;
                msg = NULL;
                return;
            }
            break;
    }
    opp_error("Undefined event of type %d in state %d (Radio state %d)!",
            msg->getKind(), macState, phy->getRadioState());
}

/**
 * Handle message in receiver
 *
 */
void FTAMacLayer::handleSelfMsgReceiver(cMessage *msg) {
    switch (macState) {
        // Call at first time after initialize the note
        case INIT:
            if (msg->getKind() == START) {
                macState = SLEEP;
                changeMACState();
                scheduleNextWakeup();
                return;
            }
            break;
        // This node is sleeping & time to wakeup
        case SLEEP:
            if (msg->getKind() == WAKE_UP) {
                macState = CCA_WB;
                changeMACState();
                // schedule the event wait WB timeout
                startWake = simTime();
                // reset CCA attempts
                ccaAttempts = 0;
                scheduleAt(startWake + waitCCA, ccaWBTimeout);
                numberWakeup++;

                nodeNumberWakeup[currentNode]++;
                iwuVec.recordWithTimestamp(simTime(), nodeWakeupInterval[currentNode]);
                return;
            }
            break;
        //
        case CCA_WB:
            if (msg->getKind() == CCA_WB_TIMEOUT) {
                // Send WB to sender - this action will be executed when radio change state successful
                macState = SENDING_WB;
                changeMACState();
                // Don't need to schedule sent wb event here, this event will be call when physical layer finish
                return;
            }
            break;

        case SENDING_WB:
            if (msg->getKind() == WB_SENT) {
                // change mac state to send data
                macState = WAIT_DATA;
                changeMACState();
                // Schedule wait data timeout event
                scheduleAt(simTime() + waitDATA, rxDATATimeout);
                return;
            }
            break;

        case WAIT_DATA:
            // if wait data timeout -> go to sleep, calculate the next wakeup interval
            if (msg->getKind() == RX_DATA_TIMEOUT) {
                macState = SLEEP;
                changeMACState();

                // calculate next wakeup interval for current node
                calculateNextInterval();
                // schedule for next wakeup time
                scheduleNextWakeup();

                return;
            }
            // Receive data -> wait CCA to send ACK
            if (msg->getKind() == DATA) {
                nbRxDataPackets++;
                macpkt_ptr_t            mac  = static_cast<macpkt_ptr_t>(msg);
                const LAddress::L2Type& dest = mac->getDestAddr();
                const LAddress::L2Type& src  = mac->getSrcAddr();
                // If this data packet destination is not for this receiver
                // wait for right data packet
                if (dest != myMacAddr) {
                    delete msg;
                    msg = NULL;
                    mac = NULL;
                    return;
                }
                // cacel event
                cancelEvent(rxDATATimeout);
                // Calculate next wakeup interval
//                cout << "before calculateNextInterval" << endl;
                calculateNextInterval(msg);
                // send mac packet to upper layer
                sendUp(decapsMsg(mac));
                // if use ack
                if (useMacAcks) {
                    //cout << "receiver have data, will send ack" << endl;
                    macState = CCA_ACK;
                    changeMACState();

                    lastDataPktSrcAddr = src;
                    // reset cca attempt number
                    ccaAttempts = 0;
                    scheduleAt(simTime() + waitCCA, ccaACKTimeout);
                } else {
                    // schedule for next wakeup time
                    scheduleNextWakeup();
                }
                return;
            }
            break;

        case CCA_ACK:
            if (msg->getKind() == CCA_ACK_TIMEOUT) {
                // Radio is free to send ACK
                macState = SENDING_ACK;
                changeMACState();
                return;
            }
            break;
        case SENDING_ACK:
            if (msg->getKind() == ACK_SENT) {
                macState = SLEEP;
                changeMACState();
                // schedule for next wakeup time
                scheduleNextWakeup();
                return;
            }
            break;
    }
    opp_error("Undefined event of type %d in state %d (Radio state %d)!",
            msg->getKind(), macState, phy->getRadioState());
}

void FTAMacLayer::updateTSR(int nodeId, int value) {
    for (int i = 0; i < TSR_length - 1; i++) {
        TSR_bank[nodeId][i] = TSR_bank[nodeId][i + 1];
    }
    TSR_bank[nodeId][TSR_length - 1] = value;
}

/**
 * Calculate next wakeup interval for current node
 */
void FTAMacLayer::calculateNextInterval(cMessage *msg) {
    int n0 = 0;
    // Move the array TSR to left to store the new value in TSR[TSR_lenth - 1]
    updateTSR(currentNode, (msg == NULL) ? 0 : 1);
    // Calculate n0;
    for (int i = 0; i < TSR_length; i++) {
        if (TSR_bank[currentNode][i] == 0) {
            n0++;
        }
    }
    /**
     * Use new adaptive function
     */
    nodeSumWUInt[currentNode] += nodeWakeupInterval[currentNode];
    if (msg != NULL) {
        macpktfta_ptr_t mac  = static_cast<macpktfta_ptr_t>(msg);
        double idle = double(mac->getIdle()) / 1000.0;
        int wbMiss = mac->getWbMiss();
        mac = NULL;
        nodeIdle[currentNode][1] = idle;
        // If this is not first packet received
        if (nodeIdle[currentNode][0] != 0 && nodeIdle[currentNode][1] != 0) {
            // Calculate the TxInt of current node
            nodeWakeupIntervalLock[currentNode] = (nodeSumWUInt[currentNode] + nodeIdle[currentNode][0] - nodeIdle[currentNode][1]) / (wbMiss + 1);
            // Next WUInt
            nodeWakeupInterval[currentNode] = nodeWakeupIntervalLock[currentNode] - idle + sysClock;
        } else {
            nodeWakeupInterval[currentNode] += n0 * sysClockFactor * sysClock;
            nodeWakeupInterval[currentNode] = round(nodeWakeupInterval[currentNode] * 1000.0) / 1000.0;
            if (nodeWakeupInterval[currentNode] < 0.02) {
                nodeWakeupInterval[currentNode] = 0.02;
            }
        }
        // re-calculate the total WUInt between 2 times receipt data packet
        nodeSumWUInt[currentNode] = 0;
        nodeIdle[currentNode][0] = idle;
        nodeIdle[currentNode][1] = 0;
    } else {
        // If we already calculated the WUInt convergent -> use this, don't need to calculate
        if (nodeWakeupIntervalLock[currentNode] > 0) {
            nodeWakeupInterval[currentNode] = nodeWakeupIntervalLock[currentNode];
            //
            nodeWakeupIntervalLock[currentNode] = 0;
        } else {
            nodeWakeupInterval[currentNode] += n0 * sysClockFactor * sysClock;
            nodeWakeupInterval[currentNode] = round(nodeWakeupInterval[currentNode] * 1000.0) / 1000.0;
            if (nodeWakeupInterval[currentNode] < 0.02) {
                nodeWakeupInterval[currentNode] = 0.02;
            }
        }
    }
    nextWakeupTime[currentNode] += nodeWakeupInterval[currentNode];
}

/**
 * Handle TADMAC preambles and received data packets.
 */
void FTAMacLayer::handleLowerMsg(cMessage *msg) {
    handleSelfMsg(msg);
}

void FTAMacLayer::handleSelfMsg(cMessage *msg) {
    // simply pass the massage as self message, to be processed by the FSM.
    // Check role of this node
    if (role == NODE_SENDER) {
        handleSelfMsgSender(msg);
    } else {
        handleSelfMsgReceiver(msg);
    }
}

/**
 * Handle transmission over messages: either send another preambles or the data
 * packet itself.
 */
void FTAMacLayer::handleLowerControl(cMessage *msg) {
    // Transmission of one packet is over
    if (msg->getKind() == MacToPhyInterface::TX_OVER) {
        if (macState == SENDING_DATA) {
            scheduleAt(simTime(), DATAsent);
        }
        if (macState == SENDING_WB) {
            scheduleAt(simTime(), WBsent);
        }
        if (macState == SENDING_ACK) {
            scheduleAt(simTime(), ACKsent);
        }
    }
    // Radio switching (to RX or TX) ir over, ignore switching to SLEEP.
    else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {
//        // we just switched to TX after CCA, so simply send the first
//        // sendPremable self message
        if ((macState == SENDING_WB)
                && (phy->getRadioState() == MiximRadio::TX)) {
            // Call action to send WB packet
            sendWB();
        }
        if ((macState == SENDING_ACK)
                && (phy->getRadioState() == MiximRadio::TX)) {
            sendMacAck();
        }
//        // we were waiting for acks, but none came. we switched to TX and now
//        // need to resend data
        if ((macState == SENDING_DATA)
                && (phy->getRadioState() == MiximRadio::TX)) {
            sendDataPacket();
        }

    } else {
        debugEV << "control message with wrong kind -- deleting\n";
    }
    delete msg;
}

/**
 * Send wakeup beacon from receiver to sender
 */
void FTAMacLayer::sendWB() {
    /**
     * For multi sender, the WB packet must send to exactly sender, cannot broadcast
     */
    macpkt_ptr_t wb = new MacPkt();
    wb->setSrcAddr(myMacAddr);
    wb->setDestAddr(LAddress::L2BROADCAST);
//    wb->setDestAddr(routeTable[currentNode]);
    wb->setName("WB");
    wb->setKind(WB);
    wb->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(wb);
    sendDown(wb);
    nbTxWB++;
}

/**
 * Send one short preamble packet immediately.
 */
void FTAMacLayer::sendMacAck() {
    macpkt_ptr_t ack = new MacPkt();
    ack->setSrcAddr(myMacAddr);
    ack->setDestAddr(lastDataPktSrcAddr);
    ack->setName("ACK");
    ack->setKind(ACK);
    ack->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(ack);
    sendDown(ack);
    nbTxAcks++;
    //endSimulation();
}


void FTAMacLayer::sendDataPacket() {
    nbTxDataPackets++;
    macpkt_ptr_t tmp = macQueue.front()->dup();
    macpktfta_ptr_t pkt = new MacPktFTA();
    pkt->setDestAddr(tmp->getDestAddr());
    pkt->setSrcAddr(tmp->getSrcAddr());
    pkt->encapsulate(tmp->decapsulate());

    lastDataPktDestAddr = pkt->getDestAddr();
    pkt->setName("DATA");
    pkt->setKind(DATA);
    pkt->setByteLength(16);
    pkt->setIdle(int(timeWaitWB.dbl() * 1000));
    pkt->setWbMiss(wbMiss);
    attachSignal(pkt);
    sendDown(pkt);
}

void FTAMacLayer::attachSignal(macpkt_ptr_t macPkt) {
    //calc signal duration
    simtime_t duration = macPkt->getBitLength() / bitrate;
    //create and initialize control info with new signal
    setDownControlInfo(macPkt,createSignal(simTime(), duration, txPower, bitrate));
}

/**
 * Change the color of the node for animation purposes.
 */

void FTAMacLayer::changeDisplayColor(COLOR color) {
    if (!animation)
        return;
    cDisplayString& dispStr = findHost()->getDisplayString();
    //b=40,40,rect,black,black,2"
    if (color == GREEN)
        dispStr.setTagArg("b", 3, "green");
    //dispStr.parse("b=40,40,rect,green,green,2");
    if (color == BLUE)
        dispStr.setTagArg("b", 3, "blue");
    //dispStr.parse("b=40,40,rect,blue,blue,2");
    if (color == RED)
        dispStr.setTagArg("b", 3, "red");
    //dispStr.parse("b=40,40,rect,red,red,2");
    if (color == BLACK)
        dispStr.setTagArg("b", 3, "black");
    //dispStr.parse("b=40,40,rect,black,black,2");
    if (color == YELLOW)
        dispStr.setTagArg("b", 3, "yellow");
    //dispStr.parse("b=40,40,rect,yellow,yellow,2");
}