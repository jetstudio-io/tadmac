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

package ftamac.modules.mac;

//***************************************************************************
//* file:        BMacLayer.ned
//*
//* author:      Anna Foerster
//* 
//* copyright:   (C) 22008- 2010, NetLab, SUPSI, Switzerland.
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
// Implementation of Ricer MAC protocol
// See C++ documentation for details.
//
simple RicerLayer extends BaseMacLayer
{
    parameters:
		@class(RicerLayer);
        // debug switch
        bool debug = default(false);
        //collect statistics?
        bool stats = default(true);

		// Ricer specific parameters
		// Define the role of node
        // 0 - receiver
        // 1 - sender (default)
        // 2 - transmiter : will be implemented later
        int role = default(1);
        
        string relayAddr = default("ff:ff:ff:ff:ff:ff");
        string forwardAddr = default("ff:ff:ff:ff:ff:ff");
        
        // how many slot after WB
        int nbSlot = default(2);
        
		// how long is one slot?
		double slotDuration @unit(s) = default(0.09s);
		
		// how long is the check interval (CCA)?
		double checkInterval @unit(s) = default(0.005s);
		
		// how long is the waiting data?
        double dataduration @unit(s) = default(0.01s);
        
        // how long is the initialization
        double initialization @unit(s) = default(0.001s);
        bool randInit = default (false);
		
		// size of the MAC queue (maximum number of packets in Tx buffer)
        double queueLength = default(20);
        
        // should we animate the nodes with colors depending on their state or 
        // not?
        bool animation = default(true);
        
        // bit rate 
        double bitrate @unit(bps) = default(19200 bps);
        
        //tx power [mW]
        double txPower @unit(mW) = default(1.0 mW);
          
        //should mac send and expect acknowledgments?
        bool useMACAcks = default(false);
        // maximum number of frame retransmission
        // only used when usage of MAC acks is enabled
        double macMaxFrameRetries = default(3);
}

