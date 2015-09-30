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

#include "Ricer3bLayer.h"

#include <cassert>

#include "FWMath.h"
#include "MacToPhyControlInfo.h"
#include "BaseArp.h"
#include "BaseConnectionManager.h"
#include "PhyUtils.h"
#include "MacPkt_m.h"
#include "MacToPhyInterface.h"

Define_Module(Ricer3bLayer)

/**
 * Initialize method of Ricer3bLayer. Init all parameters, schedule timers.
 */
void Ricer3bLayer::initialize(int stage) {
    BaseMacLayer::initialize(stage);

    if (stage == 0) {
        coeff = par("coeff");
        queueLength = hasPar("queueLength") ? par("queueLength") : 10;
        animation = hasPar("animation") ? par("animation") : true;
        slotDuration = hasPar("slotDuration") ? par("slotDuration") : 0.09;
        numNodes = par("numNodes");
        if (myMacAddr == sinkAddr) {
            slotDuration = slotDuration / coeff;

        }
        numNodes = par("numNodes");
        bitrate = hasPar("bitrate") ? par("bitrate") : 15360;
        headerLength = hasPar("headerLength") ? par("headerLength") : 10;
        checkInterval = hasPar("checkInterval") ? par("checkInterval") : 0.01;
        buzzduration = headerLength / bitrate;
        dataduration = 3 * headerLength / bitrate;

        txPower = hasPar("txPower") ? par("txPower") : 50;
        useMacAcks = hasPar("useMACAcks") ? par("useMACAcks") : false;
        maxTxAttempts = par("macMaxFrameRetries");
        debugEV << "headerLength: " << headerLength << ", bitrate: " << bitrate
                       << endl;
        stats = par("stats");
        nbTxDataPackets = 0;
        nbTxBeacons = 0;
        nbRxDataPackets = 0;
        nbRxBeacons = 0;
        nbMissedAcks = 0;
        nbRecvdAcks = 0;
        nbDroppedDataPackets = 0;
        nbTxAcks = 0;
        nbPacketDrop = 0;
        nbTxRelayData = 0;
        nbRxRelayData = 0;
        flagack = 0;
        flagsendack = 0;

        nbTxBuzz = 0;
        txAttempts = 0;
        lastDataPktDestAddr = LAddress::L2BROADCAST;
        lastDataPktSrcAddr = LAddress::L2BROADCAST;
        sinkAddr.setAddress("00:00:00:00:00:00");

        macState = INIT;

        // init the dropped packet info
        droppedPacket.setReason(DroppedPacket::NONE);
        nicId = getNic()->getId();
        WATCH(macState);
    }

    else if (stage == 1) {

        wakeup = new cMessage("wakeup");
        wakeup->setKind(Ricer_WAKE_UP);

        data_timeout = new cMessage("data_timeout");
        data_timeout->setKind(Ricer_DATA_TIMEOUT);
        data_timeout->setSchedulingPriority(100);

        relay_timeout = new cMessage("relay_timeout");
        relay_timeout->setKind(Ricer_RELAY_TIMEOUT);

        buzz_timeout = new cMessage("buzz_timeout");
        buzz_timeout->setKind(Ricer_BUZZ_TIMEOUT);

        data_tx_over = new cMessage("data_tx_over");
        data_tx_over->setKind(Ricer_DATA_TX_OVER);

        wait_over = new cMessage("wait_over");
        wait_over->setKind(Ricer_WAIT_OVER);

        send_beacon = new cMessage("send_beacon");
        send_beacon->setKind(Ricer_SEND_BEACON);

        send_buzz = new cMessage("send_buzz");
        send_buzz->setKind(Ricer_SEND_BUZZ);

        ack_tx_over = new cMessage("ack_tx_over");
        ack_tx_over->setKind(Ricer_ACK_TX_OVER);

        cca_timeout = new cMessage("cca_timeout");
        cca_timeout->setKind(Ricer_CCA_TIMEOUT);
        cca_timeout->setSchedulingPriority(100);

        send_ack = new cMessage("send_ack");
        send_ack->setKind(Ricer_SEND_ACK);

        start_Ricer = new cMessage("start_Ricer");
        start_Ricer->setKind(Ricer_START_Ricer);

        ack_timeout = new cMessage("ack_timeout");
        ack_timeout->setKind(Ricer_ACK_TIMEOUT);

        resend_data = new cMessage("resend_data");
        resend_data->setKind(Ricer_RESEND_DATA);
        resend_data->setSchedulingPriority(100);

        scheduleAt(0.0, start_Ricer);
    }
}

Ricer3bLayer::~Ricer3bLayer() {
    cancelAndDelete(wakeup);
    cancelAndDelete(data_timeout);
    cancelAndDelete(data_tx_over);
    cancelAndDelete(wait_over);
    cancelAndDelete(send_beacon);
    cancelAndDelete(ack_tx_over);
    cancelAndDelete(cca_timeout);
    cancelAndDelete(send_ack);
    cancelAndDelete(start_Ricer);
    cancelAndDelete(ack_timeout);
    cancelAndDelete(resend_data);

    MacQueue::iterator it;
    for (it = macQueue.begin(); it != macQueue.end(); ++it) {
        delete (*it);
    }
    macQueue.clear();
}

void Ricer3bLayer::finish() {
    BaseMacLayer::finish();

    // record stats
    if (stats) {
        recordScalar("nbRxRelayData", nbRxRelayData);
        recordScalar("nbTxRelayData", nbTxRelayData);
        recordScalar("nbPacketDrop", nbPacketDrop);
        recordScalar("nbTxDataPackets", nbTxDataPackets);
        recordScalar("nbTxBeacons", nbTxBeacons);
        recordScalar("nbRxDataPackets", nbRxDataPackets);
        recordScalar("nbRxBeacons", nbRxBeacons);
        recordScalar("nbMissedAcks", nbMissedAcks);
        recordScalar("nbRecvdAcks", nbRecvdAcks);
        recordScalar("nbTxAcks", nbTxAcks);
        recordScalar("nbDroppedDataPackets", nbDroppedDataPackets);
        //recordScalar("timeSleep", timeSleep);
        //recordScalar("timeRX", timeRX);
        //recordScalar("timeTX", timeTX);
    }
}

/**
 * Check whether the queue is not full: if yes, print a warning and drop the
 * packet. Then initiate sending of the packet, if the node is sleeping. Do
 * nothing, if node is working.
 */
void Ricer3bLayer::handleUpperMsg(cMessage *msg) {
    bool pktAdded = addToQueue(msg);
    if (!pktAdded)
        return;
    // force wakeup now
    if (wakeup->isScheduled() && (macState == SLEEP)) {
        cancelEvent(wakeup);
        scheduleAt(simTime() + dblrand() * 0.1f, wakeup);
    }
}

/**
 * Send one short beacon packet immediately.
 */
void Ricer3bLayer::sendBeacon() {
    MacPkt* beacon = new MacPkt();
    beacon->setSrcAddr(myMacAddr);
    beacon->setDestAddr(LAddress::L2BROADCAST);
    beacon->setKind(Ricer_BEACON);
    beacon->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(beacon);
    sendDown(beacon);
    nbTxBeacons++;
}
void Ricer3bLayer::sendRelayData() {
    cPacket *m = dupdata->decapsulate();
    MacPkt *macPkt = new MacPkt(dupdata->getName());
    macPkt->setBitLength(headerLength);
    macPkt->setKind(Ricer_RELAYDATA);
    macPkt->setDestAddr(sinkAddr);
    macPkt->setSrcAddr(myMacAddr);
    macPkt->encapsulate(m);
    attachSignal(macPkt);
    sendDown(macPkt);
    nbTxRelayData++;

}
void Ricer3bLayer::sendBuzz() {
    MacPkt* buzz = new MacPkt();
    buzz->setSrcAddr(myMacAddr);
    buzz->setDestAddr(lastDataPktDestAddr);
    buzz->setKind(Ricer_BUZZ);
    buzz->setBitLength(headerLength);
    buzz->setSequenceId(buzztx);
    //attach signal and send down
    attachSignal(buzz);
    sendDown(buzz);
    nbTxBuzz++;
}
/**
 * Send one short beacon packet immediately.
 */
void Ricer3bLayer::sendMacAck() {
    MacPkt* ack = new MacPkt();
    ack->setSrcAddr(myMacAddr);
    ack->setDestAddr(lastDataPktSrcAddr);
    ack->setKind(Ricer_ACK);
    ack->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(ack);
    sendDown(ack);
    nbTxAcks++;
    //endSimulation();
}

/**
 * Handle own messages:
 * Ricer_WAKEUP: wake up the node, check the channel for some time.
 * Ricer_CHECK_CHANNEL: if the channel is free, check whether there is something
 * in the queue and switch the radio to TX. When switched to TX, the node will
 * start sending beacons for a full slot duration. If the channel is busy,
 * stay awake to receive message. Schedule a timeout to handle false alarms.
 * Ricer_SEND_BEACONS: sending of beacons over. Next time the data packet
 * will be send out (single one).
 * Ricer_TIMEOUT_DATA: timeout the node after a false busy channel alarm. Go
 * back to sleep.
 */
void Ricer3bLayer::handleSelfMsg(cMessage *msg) {
    switch (macState) {
    case INIT:
        if (msg->getKind() == Ricer_START_Ricer) {
            debugEV << "State INIT, message Ricer_START, new state SLEEP"
                           << endl;
            changeDisplayColor(BLACK);
            phy->setRadioState(MiximRadio::SLEEP);
            macState = SLEEP;
            scheduleAt(simTime() + dblrand() * slotDuration, wakeup);
            return;
        }
        break;
    case SLEEP:
        if (msg->getKind() == Ricer_WAKE_UP) {
            debugEV
                           << "State SLEEP, message Ricer_WAKEUP, new state CCA-----------------"
                           << numNodes << "----" << endl;
            scheduleAt(simTime() + buzzduration, cca_timeout);
            macState = CCA;
            phy->setRadioState(MiximRadio::RX);
            changeDisplayColor(GREEN);
            return;
        }
        if ((msg->getKind() == Ricer_DATA) || (msg->getKind() == 22100)) {
            if (myMacAddr == sinkAddr) {
                cancelEvent(wakeup);
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(GREEN);
                macState = WAIT_DATA;
                scheduleAt(simTime() + dataduration, data_timeout);
                scheduleAt(simTime(), msg);
            }
            return;
        }
        if (msg->getKind() == Ricer_BUZZ) {
            MacPkt* buzz = static_cast<MacPkt *>(msg);
            const LAddress::L2Type& dest = buzz->getDestAddr();
            if (dest == myMacAddr) {
                debugEV
                               << "State WAITBUZZ, message Ricer_BUZZ, new state WAITDATA"
                               << endl;
                cancelEvent(wakeup);
                scheduleAt(simTime() + 2 * dataduration, data_timeout);
                macState = WAIT_DATA;
                phy->setRadioState(MiximRadio::RX);
                changeDisplayColor(GREEN);
            } else
                delete msg;
            return;
        } else {
            return;
        }
        break;

    case WAIT_BEACON:
        if (msg->getKind() == Ricer_WAIT_OVER) {
            //                      delete macQueue.front();
            //                      macQueue.pop_front();
            // if something in the queue, wakeup soon.
            debugEV << "State WAIT_BEACON, message WAIT_OVER, new state"
                    " SLEEP" << endl;
            scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            return;
        }
        if (msg->getKind() == Ricer_BEACON) {
            nbRxBeacons++;
            if (macQueue.size() > 0) {
                MacPkt *pkt = macQueue.front()->dup();
                lastDataPktDestAddr = pkt->getDestAddr();
                MacPkt* beacon = static_cast<MacPkt *>(msg);
                const LAddress::L2Type& src = beacon->getSrcAddr();
                if (src == lastDataPktDestAddr) {
                    debugEV
                                   << "State WAITBEACON, message Ricer_BEACON received, new state"
                                           " SEND_BUZZ" << endl;
                    buzztx = 1;
                    macState = SEND_BUZZ;
                    phy->setRadioState(MiximRadio::TX);
                    changeDisplayColor(YELLOW);
                    cancelEvent(wait_over);
                    txAttempts = 1;
                }
            }
            return;
        }
        if ((msg->getKind() == Ricer_DATA) || (msg->getKind() == 22100)
                || (msg->getKind() == Ricer_BUZZ)) {
            cancelEvent(wait_over);
            scheduleAt(simTime() + slotDuration + dblrand() * checkInterval,
                    wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            debugEV
                           << "State CCA, message Ricer_DATA_BROADCAST, new state WAIT_BEACON"
                           << endl;
            return;
        }
        //in case we get an ACK, we simply dicard it, because it means the end
        //of another communication
        else {
            return;
        }
        break;
    case CCA:
        if (msg->getKind() == Ricer_CCA_TIMEOUT) {
//          if (phy->getChannelState().getRSSI()>2*1e-10)
//            {
//                if (macQueue.size() > 0)
//                scheduleAt(simTime() + dblrand()*checkInterval, wakeup);
//                else
//                scheduleAt(simTime() + slotDuration+dblrand()*checkInterval, wakeup);
//                macState = SLEEP;
//                phy->setRadioState(MiximRadio::SLEEP);
//                changeDisplayColor(BLACK);
//            }
//           else {
            if (macQueue.size() > 0) {
                debugEV << "State CCA, message CCA_TIMEOUT, new state"
                        " WAIT_BEACON" << endl;
                macState = WAIT_BEACON;
                scheduleAt(simTime() + slotDuration, wait_over);
            }
            // if not, transmit wakeup beacon
            else {
                debugEV
                               << "State CCA, message CCA_TIMEOUT new state SEND_BEACON"
                               << endl;
                macState = SEND_BEACON;
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(YELLOW);
            }
//            }
            return;
        }
        if (msg->getKind() == Ricer_BEACON) {
            nbRxBeacons++;
            if (macQueue.size() > 0) {
                MacPkt *pkt = macQueue.front()->dup();
                lastDataPktDestAddr = pkt->getDestAddr();
                MacPkt* beacon = static_cast<MacPkt *>(msg);
                const LAddress::L2Type& src = beacon->getSrcAddr();
                if (src == lastDataPktDestAddr) {
                    debugEV
                                   << "State CCA, message Ricer_BEACON received, new state"
                                           " SEND_BUZZ" << endl;
                    buzztx = 1;
                    macState = SEND_BUZZ;
                    phy->setRadioState(MiximRadio::TX);
                    changeDisplayColor(YELLOW);
                    txAttempts = 1;
                    cancelEvent(wait_over);
                    cancelEvent(cca_timeout);
                }
            }
            return;
        }
        if ((msg->getKind() == 22100) || (msg->getKind() == Ricer_BUZZ)
                || (msg->getKind() == Ricer_DATA)) {
            cancelEvent(cca_timeout);
            scheduleAt(simTime() + slotDuration + dblrand() * checkInterval,
                    wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            return;
        } else {
            return;
        }
        break;

    case SEND_BUZZ:
        if (msg->getKind() == Ricer_SEND_BUZZ) {
            debugEV << "State SENDBUZZ, new state WAIT_RELAY" << endl;
            sendBuzz();
            macState = SEND_BUZZ;
            return;
        } else {
            return;
        }
        break;
    case WAIT_BUZZ:
        if (msg->getKind() == Ricer_BUZZ) {
            MacPkt* buzz = static_cast<MacPkt *>(msg);
            const LAddress::L2Type& dest = buzz->getDestAddr();
            if (dest == myMacAddr) {
                debugEV
                               << "State WAITBUZZ, message Ricer_BUZZ, new state WAITDATA"
                               << endl;
                cancelEvent(buzz_timeout);
                scheduleAt(simTime() + 4 * dataduration, data_timeout);
                macState = WAIT_DATA;
            } else
                delete msg;
            return;
        }
        if (msg->getKind() == Ricer_BUZZ_TIMEOUT) {
            debugEV
                           << "State WAIT_BUZZ, message Ricer_BUZZ TIMEOUT, new state SLEEP"
                           << endl;
            if (macQueue.size() > 0)
                scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
            else
                scheduleAt(simTime() + slotDuration + dblrand() * checkInterval,
                        wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            return;
        } else {
            return;
        }
        break;
    case SEND_BEACON:
        if (msg->getKind() == Ricer_SEND_BEACON) {
            debugEV << "State SEND_BEACON, message Ricer_SEND_BEACON, new"
                    " state WAIT_BUZZ" << endl;
            sendBeacon();
            macState = SEND_BEACON;
            return;
        }

        else {
            return;
        }
        break;

    case SEND_DATA:
        if (msg->getKind() == Ricer_RESEND_DATA) {
            debugEV << "State SEND_DATA, message "
                    " Ricer_SEND_DATA, new state WAIT_TX_DATA_OVER" << endl;
            // send the data packet
            sendDataPacket();
            flagack = 1;
            macState = WAIT_TX_DATA_OVER;
            return;
        } else {
            return;
        }
        break;
//
    case WAIT_TX_DATA_OVER:
        if (msg->getKind() == Ricer_DATA_TX_OVER) {
            debugEV << "State WAIT_TX_DATA_OVER, message Ricer_DATA_TX_OVER,"
                    " new state WAIT_ACK" << endl;
            macState = WAIT_ACK;
            phy->setRadioState(MiximRadio::RX);
            changeDisplayColor(GREEN);
            scheduleAt(simTime() + 2 * dataduration + 3 * buzzduration,
                    ack_timeout);
            return;
        } else {
            return;
        }
        break;
    case WAIT_ACK:
        if (msg->getKind() == Ricer_ACK_TIMEOUT) {
            if (txAttempts < maxTxAttempts) {
                debugEV
                               << "State WAIT_ACK, message Ricer_ACK_TIMEOUT, new state"
                                       " SEND_DATA" << endl;
                txAttempts++;
                macState = SEND_DATA;
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(YELLOW);
//				macState = WAIT_BEACON;
//				scheduleAt(simTime() + slotDuration+checkInterval, wait_over);
            } else {
                debugEV
                               << "State WAIT_ACK, message Ricer_ACK_TIMEOUT, new state"
                                       " SLEEP" << endl;
                if (flagack == 1) {
                    delete macQueue.front();
                    macQueue.pop_front();
                }
                if (macQueue.size() > 0)
                    scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
                else
                    scheduleAt(
                            simTime() + slotDuration
                                    + dblrand() * checkInterval, wakeup);
                macState = SLEEP;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                nbMissedAcks++;
            }
            return;
        }
        if (msg->getKind() == Ricer_ACK) {
            debugEV << "State WAIT_ACK, message Ricer_ACK" << endl;
            MacPkt* mac = static_cast<MacPkt *>(msg);
            const LAddress::L2Type& src = mac->getSrcAddr();
            // the right ACK is received..
            debugEV << "We are waiting for ACK from : " << lastDataPktDestAddr
                           << ", and ACK came from : " << src << endl;
            if (src == lastDataPktDestAddr) {
                debugEV << "New state SLEEP" << endl;
                nbRecvdAcks++;
                lastDataPktDestAddr = LAddress::L2BROADCAST;
                cancelEvent(ack_timeout);
                if (flagack == 1) {
                    delete macQueue.front();
                    macQueue.pop_front();
                }
                // if something in the queue, wakeup soon.
                if (macQueue.size() > 0)
                    scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
                else
                    scheduleAt(simTime() + slotDuration, wakeup);
                macState = SLEEP;
                phy->setRadioState(MiximRadio::SLEEP);
                changeDisplayColor(BLACK);
                lastDataPktDestAddr = LAddress::L2BROADCAST;
            }
            return;
        } else {
            return;
        }
        break;
    case WAIT_DATA:
//	    if(msg->getKind()==22100)
//	    {
//	        MacPkt* mac  = static_cast<MacPkt *>(msg);
//	        cancelEvent(data_timeout);
//	        scheduleAt(simTime() + dataduration+2*buzzduration, data_timeout);
//	        delete msg;
//	        return;
//	    }
        if (msg->getKind() == Ricer_DATA) {
            cancelEvent(data_timeout);
            nbRxDataPackets++;
            MacPkt* mac = static_cast<MacPkt *>(msg);
            const LAddress::L2Type& dest = mac->getDestAddr();
            const LAddress::L2Type& src = mac->getSrcAddr();
            if (dest == myMacAddr) {
                sendUp(decapsMsg(mac));
                debugEV << "State WAIT_DATA, message Ricer_DATA, new state"
                        " SEND_ACK" << endl;
                flagsendack = 1;
                macState = SEND_ACK;
                lastDataPktSrcAddr = src;
                phy->setRadioState(MiximRadio::TX);
                changeDisplayColor(YELLOW);
            }
            return;
        }
        if (msg->getKind() == Ricer_DATA_TIMEOUT) {
            debugEV << "State WAIT_DATA, message Ricer_DATA_TIMEOUT, new state"
                    " SLEEP" << endl;
            // if something in the queue, wakeup soon.
            if (macQueue.size() > 0)
                scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
            else
                scheduleAt(simTime() + slotDuration, wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            return;
        } else {
            return;
        }
        break;
    case SEND_ACK:
        if (msg->getKind() == Ricer_SEND_ACK) {
            debugEV << "State SEND_ACK, message Ricer_SEND_ACK, new state"
                    " WAIT_ACK_TX" << endl;
            // send now the ack packet
            sendMacAck();
            macState = WAIT_ACK_TX;
            return;
        } else {
            return;
        }
        break;
    case WAIT_ACK_TX:
        if (msg->getKind() == Ricer_ACK_TX_OVER) {
            debugEV << "State WAIT_ACK_TX, message Ricer_ACK_TX_OVER, new state"
                    " SLEEP" << endl;
            // ack sent, go to sleep now.
            // if something in the queue, wakeup soon.
            if (macQueue.size() > 0)
                scheduleAt(simTime() + dblrand() * checkInterval, wakeup);
            else
                scheduleAt(simTime() + slotDuration, wakeup);
            macState = SLEEP;
            phy->setRadioState(MiximRadio::SLEEP);
            changeDisplayColor(BLACK);
            lastDataPktSrcAddr = LAddress::L2BROADCAST;
            return;
        } else {
            return;
        }
        break;
    }
    opp_error("Undefined event of type %d in state %d (MiximRadio state %d)!",
            msg->getKind(), macState, phy->getRadioState());
}

/**
 * Handle Ricer beacons and received data packets.
 */
void Ricer3bLayer::handleLowerMsg(cMessage *msg) {
    // simply pass the massage as self message, to be processed by the FSM.
    handleSelfMsg(msg);

}

void Ricer3bLayer::sendDataPacket() {
    nbTxDataPackets++;
    MacPkt *pkt = macQueue.front()->dup();
    attachSignal(pkt);
    lastDataPktDestAddr = pkt->getDestAddr();
    pkt->setKind(Ricer_DATA);
    sendDown(pkt);
}

/**
 * Handle transmission over messages: either send another beacons or the data
 * packet itself.
 */
void Ricer3bLayer::handleLowerControl(cMessage *msg) {
//    if(msg->getKind()==22100)
//        {
//            nbPacketDrop++;
//            cMessage *msg2=msg->dup();
//            handleSelfMsg(msg2);
//        }
//        else if (msg->getKind()==22102)
//        {
//             cMessage *msg2=msg->dup();
//               msg2->setKind(192);
//               handleSelfMsg(msg2);
//        }
//        else if (msg->getKind()==22103)
//            {
//                cMessage *msg2=msg->dup();
//                 msg2->setKind(193);
//                 handleSelfMsg(msg2);
//            }
//        else if (msg->getKind()==22104)
//        {
//            cMessage *msg2=msg->dup();
//             msg2->setKind(194);
//             handleSelfMsg(msg2);
//        }

    // Transmission of one packet is over
    if (msg->getKind() == MacToPhyInterface::TX_OVER) {
        if (macState == SEND_BEACON) {
            macState = WAIT_BUZZ;
            phy->setRadioState(MiximRadio::RX);
            changeDisplayColor(GREEN);
            scheduleAt(simTime() + 2 * buzzduration, buzz_timeout);
        }
        if (macState == SEND_BUZZ) {
            macState = SEND_DATA;
            scheduleAt(simTime(), resend_data);
        }
        if (macState == WAIT_TX_DATA_OVER) {
            scheduleAt(simTime(), data_tx_over);
        }
        if (macState == WAIT_ACK_TX) {
            scheduleAt(simTime(), ack_tx_over);
        }
    }
    // MiximRadio switching (to RX or TX) ir over, ignore switching to SLEEP.
    else if (msg->getKind() == MacToPhyInterface::RADIO_SWITCHING_OVER) {

        // we just switched to TX after CCA, so simply send the first
        // sendPremable self message
        if ((macState == SEND_BEACON)
                && (phy->getRadioState() == MiximRadio::TX)) {
            scheduleAt(simTime(), send_beacon);
        }
        if ((macState == SEND_BUZZ)
                && (phy->getRadioState() == MiximRadio::TX)) {
            scheduleAt(simTime(), send_buzz);
        }
        if ((macState == SEND_ACK)
                && (phy->getRadioState() == MiximRadio::TX)) {
//    	    if(flagsendack==1)
//    		scheduleAt(simTime()+checkInterval, send_ack);
//    	    else
            scheduleAt(simTime() + 2 * dataduration, send_ack);
        }
        // we were waiting for acks, but none came. we switched to TX and now
        // need to resend data
        if ((macState == SEND_DATA)
                && (phy->getRadioState() == MiximRadio::TX)) {
            scheduleAt(simTime(), resend_data);
        }

        if ((macState == SEND_RELAYDATA)
                && (phy->getRadioState() == MiximRadio::TX)) {
            scheduleAt(simTime(), resend_data);
        }
    } else {
        debugEV << "control message with wrong kind -- deleting\n";
    }
    delete msg;
}

/**
 * Encapsulates the received network-layer packet into a MacPkt and set all
 * needed header fields.
 */
bool Ricer3bLayer::addToQueue(cMessage *msg) {
    if (macQueue.size() >= queueLength) {
        // queue is full, message has to be deleted
        debugEV << "New packet arrived, but queue is FULL, so new packet is"
                " deleted\n";
        msg->setName("MAC ERROR");
        msg->setKind(PACKET_DROPPED);
        sendControlUp(msg);
        droppedPacket.setReason(DroppedPacket::QUEUE);
        emit(BaseLayer::catDroppedPacketSignal, &droppedPacket);
        nbDroppedDataPackets++;

        return false;
    }

    MacPkt *macPkt = new MacPkt(msg->getName());
    macPkt->setBitLength(headerLength);
    cObject * const cInfo = msg->removeControlInfo();
    //EV<<"CSMA received a message from upper layer, name is "
    //  << msg->getName() <<", CInfo removed, mac addr="
    //  << cInfo->getNextHopMac()<<endl;
    macPkt->setDestAddr(getUpperDestinationFromControlInfo(cInfo));
    delete cInfo;
    macPkt->setSrcAddr(myMacAddr);

    assert(static_cast<cPacket*>(msg));
    macPkt->encapsulate(static_cast<cPacket*>(msg));

    macQueue.push_back(macPkt);
    debugEV << "Max queue length: " << queueLength << ", packet put in queue"
            "\n  queue size: " << macQueue.size() << " macState: " << macState
                   << endl;

    return true;
}

void Ricer3bLayer::attachSignal(MacPkt *macPkt) {
    //calc signal duration

    simtime_t duration = macPkt->getBitLength() / bitrate;
//	debugEV <<"length" <<macPkt->getName()<<endl;
    //create and initialize control info with new signal
    setDownControlInfo(macPkt, createSignal(simTime(), duration, txPower, bitrate));
}

/**
 * Change the color of the node for animation purposes.
 */

void Ricer3bLayer::changeDisplayColor(Ricer_COLORS color) {
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

/*void Ricer3bLayer::changeMacState(States newState)
 {
 switch (macState)
 {
 case RX:
 timeRX += (simTime() - lastTime);
 break;
 case TX:
 timeTX += (simTime() - lastTime);
 break;
 case SLEEP:
 timeSleep += (simTime() - lastTime);
 break;
 case CCA:
 timeRX += (simTime() - lastTime);
 }
 lastTime = simTime();

 switch (newState)
 {
 case CCA:
 changeDisplayColor(GREEN);
 break;
 case TX:
 changeDisplayColor(BLUE);
 break;
 case SLEEP:
 changeDisplayColor(BLACK);
 break;
 case RX:
 changeDisplayColor(YELLOW);
 break;
 }

 macState = newState;
 }*/
