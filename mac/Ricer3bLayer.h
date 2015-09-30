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

#ifndef Ricer3bLAYER_H_
#define Ricer3bLAYER_H_

#include <string>
#include <sstream>
#include <vector>
#include <list>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include <DroppedPacket.h>

class MacPkt;

/**
 * @brief Implementation of B-MAC (called also Berkeley MAC, Low Power
 * Listening or LPL).
 *
 * The protocol works as follows: each node is allowed to sleep for
 * slotDuration. After waking up, it first checks the channel for ongoing
 * transmissions.
 * If a transmission is catched (a beacon is received), the node stays awake
 * for at most slotDuration and waits for the actual data packet.
 * If a node wants to send a packet, it first sends beacons for at least
 * slotDuration, thus waking up all nodes in its transmission radius and
 * then sends out the data packet. If a mac-level ack is required, then the
 * receiver sends the ack immediately after receiving the packet (no beacons)
 * and the sender waits for some time more before going back to sleep.
 *
 * B-MAC is designed for low traffic, low power communication in WSN and is one
 * of the most widely used protocols (e.g. it is part of TinyOS).
 * The finite state machine of the protocol is given in the below figure:
 *
 * \image html RicerFSM.png "B-MAC Layer - finite state machine"
 *
 * A paper describing this implementation can be found at:
 * http://www.omnet-workshop.org/2011/uploads/slides/OMNeT_WS2011_S5_C1_Foerster.pdf
 *
 * @class Ricer3bLayer
 * @ingroup macLayer
 * @author Anna Foerster
 *
 */
class MIXIM_API Ricer3bLayer : public BaseMacLayer
{
  private:
	/** @brief Copy constructor is not allowed.
	 */
	Ricer3bLayer(const Ricer3bLayer&);
	/** @brief Assignment operator is not allowed.
	 */
	Ricer3bLayer& operator=(const Ricer3bLayer&);

  public:
	Ricer3bLayer()
		: BaseMacLayer()
		, macQueue()
		, nbTxDataPackets(0), nbTxBeacons(0), nbRxDataPackets(0), nbRxBeacons(0)
		, nbMissedAcks(0), nbRecvdAcks(0), nbDroppedDataPackets(0), nbTxAcks(0)
		, macState(INIT)
		, resend_data(NULL), ack_timeout(NULL), start_Ricer(NULL), wakeup(NULL)
		, send_ack(NULL), cca_timeout(NULL), ack_tx_over(NULL), send_beacon(NULL), wait_over(NULL)
		, data_tx_over(NULL), data_timeout(NULL)
		, lastDataPktSrcAddr()
		, lastDataPktDestAddr()
		, txAttempts(0)
		, droppedPacket()
		, nicId(-1)
		, queueLength(0)
		, animation(false)
		, slotDuration(0), bitrate(0), headerLength(0), checkInterval(0), txPower(0)
		, useMacAcks(0)
		, maxTxAttempts(0)
		, stats(false)
	{}
	virtual ~Ricer3bLayer();

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
    typedef std::list<MacPkt*> MacQueue;

    /** @brief A queue to store packets from upper layer in case another
	packet is still waiting for transmission.*/
    MacQueue macQueue;

    /** @name Different tracked statistics.*/
	/*@{*/
    int numNodes;
	long nbTxDataPackets;
	long nbTxBeacons;
	long nbRxDataPackets;
	long nbRxBeacons;
	long nbMissedAcks;
	long nbRecvdAcks;
	long nbDroppedDataPackets;
	long nbTxAcks;
	long nbPacketDrop;
	long nbTxRelayData;
	long nbRxRelayData;
	long nbTxBuzz;
	/*@}*/

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
	*  				and wiats for the actual data packet
	*  SEND_DATA -- node has sent enough beacons and sends the actual data
	*  				packet
	*  WAIT_TX_DATA_OVER -- node waits until the data packet sending is ready
	*  WAIT_ACK -- node has sent the data packet and waits for ack from the
	*  			   receiving node
	*  SEND_ACK -- node send an ACK back to the sender
	*  WAIT_ACK_TX -- node waits until the transmission of the ack packet is
	*  				  over
	*/
	enum States {
		INIT,	//0
		SLEEP,	//1
		CCA,	//2
		SEND_BEACON, 	//3
		WAIT_DATA,		//4
		SEND_DATA,		//5
		WAIT_TX_DATA_OVER,	//6
		WAIT_ACK,		//7
		SEND_ACK,		//8
		WAIT_ACK_TX,		//9
		WAIT_BEACON,  //10
		WAIT_RELAY,   //11
		SEND_BUZZ,    //12
		WAIT_BUZZ,    //13
		SEND_RELAYDATA
	  };
	/** @brief The current state of the protocol */
	States macState;

	/** @brief Types of messages (self messages and packets) the node can
	 * process **/
	enum TYPES {
		// packet types
	    Ricer_DATA = 191,
	    Ricer_BEACON,  //192
		Ricer_BUZZ,     //193
		Ricer_RELAYDATA,  //194
		Ricer_ACK,          //195
		// self message types
		Ricer_RESEND_DATA,   //196
		Ricer_ACK_TIMEOUT,   //197
		Ricer_START_Ricer,    //198
		Ricer_WAKE_UP,        //199
		Ricer_SEND_ACK,       //200
		Ricer_CCA_TIMEOUT,     //201
		Ricer_ACK_TX_OVER,
		Ricer_SEND_BEACON,     //203
		Ricer_WAIT_OVER,
		Ricer_DATA_TX_OVER,     //205
		Ricer_DATA_TIMEOUT,
		Ricer_RELAY_TIMEOUT,
		Ricer_BUZZ_TIMEOUT,     //208
		Ricer_SEND_BUZZ,       //209
		Ricer_SEND_RELAYDATA

	};

	// messages used in the FSM
	cMessage *resend_data;
	cMessage *ack_timeout;
	cMessage *start_Ricer;
	cMessage *wakeup;
	cMessage *send_ack;
	cMessage *cca_timeout;
	cMessage *ack_tx_over;
	cMessage *send_beacon;
	cMessage *wait_over;
	cMessage *data_tx_over;
	cMessage *data_timeout;
	cMessage *relay_timeout;
	cMessage *buzz_timeout;
	cMessage *send_buzz;
	MacPkt   *dupdata;
	/** @name Help variables for the acknowledgment process. */
	/*@{*/
	LAddress::L2Type lastDataPktSrcAddr;
	LAddress::L2Type lastDataPktDestAddr;
	LAddress::L2Type sinkAddr;
	int              txAttempts;
	int flagack;
	int flagsendack;
	int buzztx;
	/*@}*/


	/** @brief Inspect reasons for dropped packets */
	DroppedPacket droppedPacket;

	/** @brief publish dropped packets nic wide */
	int nicId;
	double coeff;
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
	double wkduration;
	double buzzduration;
	double dataduration;
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
	enum Ricer_COLORS {
		GREEN = 1,
		BLUE = 2,
		RED = 3,
		BLACK = 4,
		YELLOW = 5
	};

	/** @brief Internal function to change the color of the node */
	void changeDisplayColor(Ricer_COLORS color);

	/** @brief Internal function to send the first packet in the queue */
	void sendDataPacket();

	/** @brief Internal function to send an ACK */
	void sendMacAck();

	/** @brief Internal function to send one beacon */
	void sendBeacon();
	void sendBuzz();
	void sendRelayData();
	/** @brief Internal function to attach a signal to the packet */
	void attachSignal(MacPkt *macPkt);

	/** @brief Internal function to add a new packet from upper to the queue */
	bool addToQueue(cMessage * msg);
};

#endif /* RicerLAYER_H_ */
