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

package tadmac.modules.mac;

//***************************************************************************
//* file:        FTAMacLayer.ned
//*
//* author:      NGUYEN Van Thiep
//* 
//* copyright:   (C) 22008- 2010, CAIRN, ENSSAT, Lannion, France.
//*
//*              This program is free software; you can redistribute it
//*              and/or modify it under the terms of the GNU General Public
//*              License as published by the Free Software Foundation; either
//*              version 2 of the License, or (at your option) any later
//*              version.
//*              For further information see file COPYING
//*              in the top level directory
//*
//**************************************************************************/

import org.mixim.base.modules.BaseMacLayer;

//
// Implementation of FTA-MAC (called also Fast Traffic Adaptive MAC).
// See C++ documentation for details.
//

//****************
// Version 1.1: support multi senders
//****************

simple FTAMacLayer extends BaseMacLayer
{
    parameters:
        @class(FTAMacLayer);
        //*********
        // Sepcific parameters for FTAMAC
        //*********
        
        // Define the role of node
        // 0 - receiver
        // 1 - sender (default)
        // 2 - transmiter : will be implemented later
        int role = default(1);
        
        // time for CCA
        double waitCCA @unit(s) = default(0.005s); //CCA time 5ms
        // time maximum for CCA
        double maxCCA @unit(s) = default(0.01s); //maxCCA time 10ms
        // time to wait WB - used on sender only because receiver send WB
        double waitWB @unit(s) = default(0.25s);  //max time for wait WB is 250ms
        // time to wait ACK - used on sender
        double waitACK @unit(s) = default(0.01s); //time to wait ACK is 10ms
        // time to wait DATA - used on receiver
        double waitDATA @unit(s) = default(0.02s); //time to wait DATA is 20ms
        // system clock - we calculate by 1ms
        double sysClock @unit(s) = default(0.001s);
        
        // system clock factor
        int sysClockFactor = default(75);
        // weighting factor
        double alpha = default(0.5);
        // wake up interval for each node - used in receiver only because in sender, node wake up when it has data to send
        double WUIInit @unit(s) = default(0.1s);  // value initialization for wakeup interval: 100ms
        
        // used to define number sender in network - for receiver node
        int numberSender = default(1);
        
        // debug switch
        bool debug = default(false);
        //collect statistics?
        bool stats = default(true);
        
        // size of the MAC queue (maximum number of packets in Tx buffer)
        int queueLength = default(8);
        
        // should we animate the nodes with colors depending on their state or 
        // not?
        bool animation = default(true);
        
        // bit rate 250kbps TI CC2420 radio chip
        double bitrate @unit(bps) = default(250000 bps);
        
        //tx power [mW]
        double txPower @unit(mW) = default(50 mW);
          
        //should mac send and expect acknowledgments?
        bool useMACAcks = default(true);
        // maximum number of frame retransmission
        // only used when usage of MAC acks is enabled
        int macMaxFrameRetries = default(3);
}