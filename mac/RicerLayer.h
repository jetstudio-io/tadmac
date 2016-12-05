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

#ifndef RicerLAYER_H_
#define RicerLAYER_H_

#include <string>
#include <sstream>
#include <vector>
#include <list>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include <DroppedPacket.h>
#include "DATAPkt_m.h"

class DATAPkt;


class MIXIM_API RicerLayer : public BaseMacLayer
{
  private:
    /** @brief Copy constructor is not allowed.
     */
    RicerLayer(const RicerLayer&);
    /** @brief Assignment operator is not allowed.
     */
    RicerLayer& operator=(const RicerLayer&);

  public:
    RicerLayer()
        : BaseMacLayer()
        , macQueue()
        , nbTxDataPackets(0), nbTxBeacons(0), nbRxDataPackets(0), nbRxBeacons(0)
        , nbMissedAcks(0), nbRecvdAcks(0), nbDroppedDataPackets(0), nbTxAcks(0), nbTxRelayData(0)
        , macState(INIT)
        , wakeup(NULL), wakeup_data(NULL), data_timeout(NULL), data_tx_over(NULL)
        , beacon_tx_over(NULL), beacon_timeout(NULL), ack_tx_over(NULL), cca_timeout(NULL)
        , ack_timeout(NULL), start(NULL)
        , lastDataPktSrcAddr()
        , lastDataPktDestAddr()
        , forwardAddr()
        , txAttempts(0)
        , droppedPacket()
        , nicId(-1)
        , queueLength(0)
        , animation(false)
        , slotDuration(0), bitrate(0), headerLength(0), checkInterval(0), iwu(0), txPower(1.0)
        , useMacAcks(0)
        , maxTxAttempts(0)
        , stats(false)
        , wakeupTime(0)
    {}
    virtual ~RicerLayer();

    /** @brief Initialization of the module and some variables*/
    virtual void initialize(int);

    /** @brief Delete all dynamically allocated objects of the module*/
    virtual void finish();

    /** @brief Handle messages from lower layer */
    virtual void handleLowerMsg(cMessage*);

    /** @brief Handle messages from upper layer */
    virtual void handleUpperMsg(cMessage*);

    /** @brief Handle self messages such as timers */
    virtual void handleSelfMsg(cMessage*);

    /** @brief Handle control messages from lower layer */
    virtual void handleLowerControl(cMessage *msg);

  protected:
    typedef DATAPkt* dataPkt_prt_t;
    typedef std::list<dataPkt_prt_t> MacQueue;

    /** @brief A queue to store packets from upper layer in case another
    packet is still waiting for transmission.*/
    MacQueue macQueue;

    /** @name Different tracked statistics.*/
    /*@{*/
    long nbTxDataPackets;
    long nbTxBeacons;
    long nbRxDataPackets;
    long nbRxBeacons;
    long nbMissedAcks;
    long nbRecvdAcks;
    long nbDroppedDataPackets;
    long nbTxAcks;
    long nbTxRelayData;

    /** @brief Ouput vector tracking the idle listening interval.*/
    cOutVector idleVec;

    int ccaAttempts;
    /*@}*/

    // Note type
    enum ROLES {
        NODE_RECEIVER,      // 0
        NODE_SENDER,        // 1
        NODE_TRANSMITER     // 2
    };
    ROLES role;

    /** @brief MAC states
    *
    *  The MAC states help to keep track what the MAC is actually
    *  trying to do.
    *  INIT -- node has just started and its status is unclear
    *  SLEEP -- node sleeps, but accepts packets from the network layer
    *  CCA -- Clear Channel Assessment - MAC checks
    *         whether medium is busy
    *  SEND_BEACON -- node sends beacons to wake up all nodes
    *  WAIT_DATA -- node has received at least one beacon from another node
    *               and wiats for the actual data packet
    *  SEND_DATA -- node has sent enough beacons and sends the actual data
    *               packet
    *  WAIT_TX_DATA_OVER -- node waits until the data packet sending is ready
    *  WAIT_ACK -- node has sent the data packet and waits for ack from the
    *              receiving node
    *  SEND_ACK -- node send an ACK back to the sender
    *  WAIT_ACK_TX -- node waits until the transmission of the ack packet is
    *                 over
    */
    enum States {
        INIT,               //0
        SLEEP,              //1
        CCA,                //2
        SEND_BEACON,        //3
        WAIT_DATA,          //4
        SEND_DATA,          //5
        WAIT_TX_DATA_OVER,  //6
        WAIT_ACK,           //7
        SEND_ACK,           //8
        WAIT_TX_ACK_OVER,   //9
        WAIT_BEACON,        //10
        WAIT_TX_BEACON_OVER //11
      };
    /** @brief The current state of the protocol */
    States macState;

    /** @brief Types of messages (self messages and packets) the node can
     * process **/
    enum TYPES {
        // packet types
        Ricer_DATA = 190,
        Ricer_BEACON,       //191
        Ricer_ACK,          //192
        Ricer_DATA_AGG,     //193
        // self message types
        Ricer_START = 200,      //200
        Ricer_WAKE_UP,          //201
        Ricer_WAKE_UP_DATA,     //202
        Ricer_CCA_TIMEOUT,      //203
        Ricer_DATA_TX_OVER,     //204
        Ricer_BEACON_TX_OVER,   //205
        Ricer_ACK_TIMEOUT,      //206
        Ricer_ACK_TX_OVER,      //207
        Ricer_BEACON_TIMEOUT,   //208
        Ricer_DATA_TIMEOUT,     //209
    };

    // messages used as the events
    cMessage *wakeup;           // wake up event - used in destination & relay: wake up to send WB
    cMessage *wakeup_data;      // wake up to send data event - used in sources
    cMessage *data_timeout;     // wait DATA time out event
    cMessage *data_tx_over;     // sending data finished event
    cMessage *beacon_tx_over;   // sending WB finished event
    cMessage *beacon_timeout;   // wait WB timeout event
    cMessage *ack_tx_over;      // sending ACK finished event
    cMessage *cca_timeout;      // finish channel check event
    cMessage *ack_timeout;      // wait ACK time out event
    cMessage *start;            // start protocol event

    /** @name Help variables for the acknowledgment process. */
    /*@{*/
    LAddress::L2Type lastDataPktSrcAddr;
    LAddress::L2Type lastDataPktDestAddr;
    LAddress::L2Type forwardAddr;
    int txAttempts;
    /*@}*/


    /** @brief Inspect reasons for dropped packets */
    DroppedPacket droppedPacket;

    /** @brief publish dropped packets nic wide */
    int nicId;
    /** @brief The maximum length of the queue */
    double queueLength;
    /** @brief Animate (colorize) the nodes.
     *
     * The color of the node reflects its basic status (not the exact state!)
     * BLACK - node is sleeping
     * GREEN - node is receiving
     * YELLOW - node is sending
     */
    bool animation;
    /** @brief The duration of the slot in secs. */
    double slotDuration;
    /** @brief The bitrate of transmission */
    double bitrate;
    /** @brief The length of the MAC header */
    double headerLength;
    /** @brief The duration of CCA */
    double checkInterval;
    double iwu;

    int nbSlot;
    /** @brief Transmission power of the node */
    double txPower;
    /** @brief Use MAC level acks or not */
    bool useMacAcks;
    /** @brief Maximum transmission attempts per data packet, when ACKs are
     * used */
    int maxTxAttempts;
    /** @brief Gather stats at the end of the simulation */
    bool stats;

    bool packetError;
    int nbPacketError;

    /** @brief Possible colors of the node for animation */
    enum Ricer_COLORS {
        GREEN = 1,
        BLUE = 2,
        RED = 3,
        BLACK = 4,
        YELLOW = 5
    };

    /**
     * variable used to calculate in protocol
     */
    double wakeupTime;

    /** @brief Internal function to change the color of the node */
    void changeDisplayColor(Ricer_COLORS color);

    /** @brief Internal function to send the first packet in the queue */
    void sendDataPacket();

    /** @brief Internal function to send an ACK */
    void sendMacAck();

    /** @brief Internal function to send one beacon */
    void sendBeacon();
    void sendRelayData();
    /** @brief Internal function to attach a signal to the packet */
    void attachSignal(MacPkt *macPkt);

    /** @brief Internal function to add a new packet from upper to the queue */
    bool addToQueue(cMessage * msg);
};

#endif /* RicerLAYER_H_ */
