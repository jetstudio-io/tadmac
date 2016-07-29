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

#include "PWMacLayer.h"

#include <cassert>

#include "FWMath.h"
#include "MacToPhyControlInfo.h"
#include "BaseArp.h"
#include "BaseConnectionManager.h"
#include "PhyUtils.h"
#include "MacPkt_m.h"
#include "MacToPhyInterface.h"

Define_Module(PWMacLayer)

/**
 * Initialize method of PWMacLayer. Init all parameters, schedule timers.
 */
void PWMacLayer::initialize(int stage) {
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

        resend_data->setSchedulingPriority(100);

        scheduleAt(0.0, start);
    }
}

PWMacLayer::~PWMacLayer() {
    cancelAndDelete(wakeup);
    cancelAndDelete(data_timeout);
    cancelAndDelete(data_tx_over);
    cancelAndDelete(wait_over);
    cancelAndDelete(send_beacon);
    cancelAndDelete(ack_tx_over);
    cancelAndDelete(cca_timeout);
    cancelAndDelete(send_ack);
    cancelAndDelete(start);
    cancelAndDelete(ack_timeout);
    cancelAndDelete(resend_data);

    MacQueue::iterator it;
    for (it = macQueue.begin(); it != macQueue.end(); ++it) {
        delete (*it);
    }
    macQueue.clear();
}

void PWMacLayer::finish() {
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
void PWMacLayer::handleUpperMsg(cMessage *msg) {
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
void PWMacLayer::sendBeacon() {
    MacPkt* beacon = new MacPkt();
    beacon->setSrcAddr(myMacAddr);
    beacon->setDestAddr(LAddress::L2BROADCAST);
    beacon->setKind(PW_BEACON);
    beacon->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(beacon);
    sendDown(beacon);
    nbTxBeacons++;
}
void PWMacLayer::sendRelayData() {
    cPacket *m = dupdata->decapsulate();
    MacPkt *macPkt = new MacPkt(dupdata->getName());
    macPkt->setBitLength(headerLength);
    macPkt->setKind(PW_RELAYDATA);
    macPkt->setDestAddr(sinkAddr);
    macPkt->setSrcAddr(myMacAddr);
    macPkt->encapsulate(m);
    attachSignal(macPkt);
    sendDown(macPkt);
    nbTxRelayData++;

}
void PWMacLayer::sendBuzz() {
    MacPkt* buzz = new MacPkt();
    buzz->setSrcAddr(myMacAddr);
    buzz->setDestAddr(lastDataPktDestAddr);
    buzz->setKind(PW_BUZZ);
    buzz->setBitLength(headerLength);
    buzz->setSequenceId(buzztx);
    //attach signal and send down
    attachSignal(buzz);
    sendDown(buzz);
}
/**
 * Send one short beacon packet immediately.
 */
void PWMacLayer::sendMacAck() {
    MacPkt* ack = new MacPkt();
    ack->setSrcAddr(myMacAddr);
    ack->setDestAddr(lastDataPktSrcAddr);
    ack->setKind(PW_ACK);
    ack->setBitLength(headerLength);

    //attach signal and send down
    attachSignal(ack);
    sendDown(ack);
    nbTxAcks++;
    //endSimulation();
}

/**
 * Handle own messages:
 * PW_WAKEUP: wake up the node, check the channel for some time.
 * PW_CHECK_CHANNEL: if the channel is free, check whether there is something
 * in the queue and switch the radio to TX. When switched to TX, the node will
 * start sending beacons for a full slot duration. If the channel is busy,
 * stay awake to receive message. Schedule a timeout to handle false alarms.
 * PW_SEND_BEACONS: sending of beacons over. Next time the data packet
 * will be send out (single one).
 * PW_TIMEOUT_DATA: timeout the node after a false busy channel alarm. Go
 * back to sleep.
 */
void PWMacLayer::handleSelfMsg(cMessage *msg) {
    opp_error("Undefined event of type %d in state %d (MiximRadio state %d)!",
            msg->getKind(), macState, phy->getRadioState());
}

/**
 * Handle PW beacons and received data packets.
 */
void PWMacLayer::handleLowerMsg(cMessage *msg) {
    // simply pass the massage as self message, to be processed by the FSM.
    handleSelfMsg(msg);

}

void PWMacLayer::sendDataPacket() {
    nbTxDataPackets++;
    MacPkt *pkt = macQueue.front()->dup();
    attachSignal(pkt);
    lastDataPktDestAddr = pkt->getDestAddr();
    pkt->setKind(PW_DATA);
    sendDown(pkt);
}

/**
 * Handle transmission over messages: either send another beacons or the data
 * packet itself.
 */
void PWMacLayer::handleLowerControl(cMessage *msg) {
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
bool PWMacLayer::addToQueue(cMessage *msg) {
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

void PWMacLayer::attachSignal(MacPkt *macPkt) {
    //calc signal duration

    simtime_t duration = macPkt->getBitLength() / bitrate;
//	debugEV <<"length" <<macPkt->getName()<<endl;
    //create and initialize control info with new signal
    setDownControlInfo(macPkt, createSignal(simTime(), duration, txPower, bitrate));
}

/**
 * Change the color of the node for animation purposes.
 */

void PWMacLayer::changeDisplayColor(PW_COLORS color) {
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

/*void PWMacLayer::changeMacState(States newState)
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
