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
 * Version 1.0: WB is broadcast
 */

#ifndef FTAMACLAYER_H_
#define FTAMACLAYER_H_

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <fstream>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include <DroppedPacket.h>
#include <MacPktFTA_m.h>

using namespace std;

class MacPktTAD;

/**
 * @class FTAMacLayer
 * @ingroup macLayer
 * @author Nguyen Van Thiep
 */
class MIXIM_API FTAMacLayer: public BaseMacLayer {
private:
    /** @brief Copy constructor is not allowed.
     */
    FTAMacLayer(const FTAMacLayer&);
    /** @brief Assignment operator is not allowed.
     */
    FTAMacLayer& operator=(const FTAMacLayer&);

public:
    FTAMacLayer() :
            BaseMacLayer(), macQueue(),
                nbTxDataPackets(0), nbTxWB(0), nbRxDataPackets(0), nbRxWB(0), nbMissedAcks(0), nbRecvdAcks(0), nbDroppedDataPackets(0), nbTxAcks(0),
                TSR_length(16), wakeupInterval(0.5), waitCCA(0.1), waitWB(0.3),
                waitACK(0.3), waitDATA(0.3), sysClock(0.001), alpha(0.5),
                macState(INIT),
                start(NULL), rxWBTimeout(NULL), WBreceived(NULL), ccaDATATimeout(NULL), DATAsent(NULL), waitACKTimeout(NULL), ACKreceived(NULL),
                wakeup(NULL), ccaWBTimeout(NULL), WBsent(NULL), rxDATATimeout(NULL), DATAreceived(NULL), ccaACKTimeout(NULL), ACKsent(NULL),
                lastDataPktSrcAddr(), lastDataPktDestAddr(),
                txAttempts(0), droppedPacket(), nicId(-1), queueLength(0), animation(false),
                bitrate(0), txPower(0),
                useMacAcks(0), maxTxAttempts(0), stats(false), wakeupIntervalLook(0),
                numberWakeup(0), sysClockFactor(75), numberSender(1)
    {}

    typedef MacPktFTA* macpktfta_ptr_t;

    virtual ~FTAMacLayer();

    /** @brief Initialization of the module and some variables*/
    virtual void initialize(int);

    /** @brief Delete all dynamically allocated objects of the module*/
    virtual void finish();

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(cMessage*);

    /** @brief Handle messages from upper layer */
    virtual void handleUpperMsg(cMessage*);

    /** @brief Handle self message */
    virtual void handleSelfMsg(cMessage*);

    /** @brief Handle self messages such as timers used by sender */
    virtual void handleSelfMsgSender(cMessage*);

    /** @brief Handle self messages such as timers used by receiver */
    virtual void handleSelfMsgReceiver(cMessage*);

    /** @brief Handle control messages from lower layer */
    virtual void handleLowerControl(cMessage *msg);

protected:
    typedef std::list<macpkt_ptr_t> MacQueue;

    /** @brief A queue to store packets from upper layer in case another
     packet is still waiting for transmission.*/
    MacQueue macQueue;

    /** @name Different tracked statistics.*/
    /*@{*/
    long nbTxDataPackets;
    long nbTxWB;
    long nbRxDataPackets;
    long nbRxWB;
    long nbMissedAcks;
    long nbRecvdAcks;
    long nbDroppedDataPackets;
    long nbTxAcks;

    int numWUConvergent;
    /*@}*/

    // Note type
    enum ROLES {
        NODE_RECEIVER,      // 0
        NODE_SENDER,        // 1
        NODE_TRANSMITER     // 2
    };
    ROLES role;

    int TSR[16];
    int TSR_length;
    /** @brief store the moment wakeup, will be used to calculate the rest time */
    simtime_t startWake;
    /** store the moment the sender wait for WB */
    double timeWaitWB;

    double wakeupInterval;
    double waitCCA;
    double maxCCA;
    double waitWB;
    double waitACK;
    double waitDATA;
    double sysClock;
    double alpha;
    double sigma;

    /** @brief MAC states */
    enum States {
        INIT,	        //0
        SLEEP,	        //1
        // The stages for sender
        WAIT_WB,        //2
        CCA_DATA,       //3
        SENDING_DATA,   //4
        WAIT_ACK,		//5
        // The stages for receiver
        CCA_WB,         //6
        SENDING_WB,     //7
        WAIT_DATA,      //8
        CCA_ACK,        //9
        SENDING_ACK,    //10
    };
    /** @brief The current state of the protocol */
    States macState;

    /** @brief Types of messages (self messages and packets) the node can process **/
    enum TYPES {
        START,           //0
        // The messages used by sender
        WAKE_UP_DATA,    //1    // Current state SLEEP, called when received DATA packet from upper layer network
        RX_WB_TIMEOUT,   //2    // Current state WAIT_WB, next state SLEEP
        WB_RECEIVED,     //3    // This event called when received WB - Current state WAIT_WB, next state CCA_DATA
        CCA_DATA_TIMEOUT,//4    // Current state CCA_DATA, next state SENDING_DATA
        DATA_SENT,       //5    // current state SENDING_DATA, next state WAIT_ACK
        WAIT_ACK_TIMEOUT,//6    // current state WAIT_ACK, next state WAIT_WB
        ACK_RECEIVED,    //7    // This event called when received ACK
        // The message used by receiver
        WAKE_UP,         //8    // Current state SLEEP, next state CCA_WB
        CCA_WB_TIMEOUT,  //9    // current state CCA_WB, next state SENDING_WB
        WB_SENT,         //10   // current state SENDING_WB, next state WAIT_DATA
        RX_DATA_TIMEOUT, //11   // current state WAIT_DATA, next state SLEEP
        DATA_RECEIVED,   //12   // This event called when receive DATA packet, current state WAIT_DATA, next state CCA_ACK
        CCA_ACK_TIMEOUT, //13   // current state CCA_ACK, next state SENDING_ACK
        ACK_SENT,        //14   // current state SENDING_ACK, next state SLEEP
        // The message used to transmit between the node
        WB,              //15   // WB packet
        DATA,            //16   // DATA packet received from network upper layer or physical lower layer
        ACK              //17   // ACK packet
    };

    // The messages used as events
    cMessage *start;            // call to start protocol TADMAC
    // The messages events used for sender
    cMessage *wakeupDATA;       // Type WAKE_UP_DATA
    cMessage *rxWBTimeout;      // Type RX_WB_TIMEOUT
    cMessage *WBreceived;       // Type WB_RECEIVED
    cMessage *ccaDATATimeout;   // Type CCA_DATA_TIMEOUT
    cMessage *DATAsent;         // Type DATA_SENT
    cMessage *waitACKTimeout;   // Type WAIT_ACK_TIMEOUT
    cMessage *ACKreceived;      // Type ACK_RECEIVED
    // The messages events used for receiver
    cMessage *wakeup;           // Type WAKE_UP
    cMessage *ccaWBTimeout;     // Type CCA_WB_TIMEOUT
    cMessage *WBsent;           // Type WB_SENT
    cMessage *rxDATATimeout;    // Type RX_DATA_TIMEOUT
    cMessage *DATAreceived;     // Type DATA_RECEIVED
    cMessage *ccaACKTimeout;    // Type CCA_ACK_TIMEOUT
    cMessage *ACKsent;          // Type ACK_SENT

    /** @name Help variables for the acknowledgment process. */
    /*@{*/
    LAddress::L2Type lastDataPktSrcAddr;
    LAddress::L2Type lastDataPktDestAddr;
    int txAttempts;
    /*@}*/

    /** @brief Inspect reasons for dropped packets */
    DroppedPacket droppedPacket;

    /** @brief publish dropped packets nic wide */
    int nicId;

    /** @brief The maximum length of the queue */
    unsigned int queueLength;
    /** @brief Animate (colorize) the nodes.
     *
     * The color of the node reflects its basic status (not the exact state!)
     * BLACK - node is sleeping
     * GREEN - node is receiving
     * YELLOW - node is sending
     */
    bool animation;

    /** @brief The bitrate of transmission */
    double bitrate;
    /** @brief Transmission power of the node */
    double txPower;
    /** @brief Use MAC level acks or not */
    bool useMacAcks;
    /** @brief Maximum transmission attempts per data packet, when ACKs are
     * used */
    int maxTxAttempts;
    /** @brief Gather stats at the end of the simulation */
    bool stats;

    /** @brief Possible colors of the node for animation */
    enum COLOR {
        GREEN = 1, BLUE = 2, RED = 3, BLACK = 4, YELLOW = 5
    };

    /**
     * These variables used for calculate the error correlator.
     */
    int nodeIdx;
    double wakeupIntervalLook;

    bool useCorrection;
    bool usePriority;
    bool useWBMiss;
    int numberWakeup;
    int sysClockFactor;

    /** @brief Ouput vector tracking the wakeup interval.*/
    cOutVector *iwuVec;
    simtime_t lastWakeup;

    /**
     * Define variable for multi sender
     */
    int numberSender;
    int currentNode;
    double *nodeWakeupInterval;
    double *nodeWakeupIntervalLock;
    // used in new adaptive function
    double *nodeSumWUInt;
    double *nextWakeupTime;
    double *sentWB;
    double globalSentWB;
    double **nodeIdle;
    int *nodeIndex;
    int **TSR_bank;
    int *nodeNumberWakeup;
    int *nbRxData;
    LAddress::L2Type *routeTable;
    LAddress::L2Type receiverAddress;

    static const int maxCCAattempts = 2;
    int ccaAttempts;
    int wbMiss;

    int nbCollision;
    int *nodeCollision;
    int *nodeChosen;
    int *nodeBroken;

    /** @brief Change MAC state */
    void changeMACState();

    /** @brief Internal function to change the color of the node */
    void changeDisplayColor(COLOR color);

    /** @brief Internal function to send the first packet in the queue */
    void sendDataPacket();

    /** @brief Internal function to send an ACK */
    void sendMacAck();

    /** @brief Internal function to send one WB */
    void sendWB();

    /** @brief Internal function to attach a signal to the packet */
    void attachSignal(macpkt_ptr_t macPkt);

    /** @brief Internal function to add a new packet from upper to the queue */
    bool addToQueue(cMessage *msg);

    void handleDataPacket(cMessage *msg);
    double getCCA();

    /** @brief Calculate the next wakeup interval*/
    void calculateNextInterval(int nodeId, macpktfta_ptr_t mac=NULL);

    void scheduleNextWakeup();
    void writeLog(int nodeId = 0);
    void updateTSR(int nodeId, int value);


    virtual cObject* setUpControlInfo(cMessage *const pMsg, const LAddress::L2Type& pSrcAddr);
};

#endif /* FTAMacLayer_H_ */
