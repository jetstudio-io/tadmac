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
//**************************************************************************
// * file:        MacPktWB.msg
// *
// * author:      Nguyen Van Thiep
// *
// * copyright:   (C) 2014 CAIRN - ENSSAT - IRISA
// *			  Lannion, 22300, France
// *
// *              This program is free software; you can redistribute it 
// *              and/or modify it under the terms of the GNU General Public 
// *              License as published by the Free Software Foundation; either
// *              version 2 of the License, or (at your option) any later 
// *              version.
// *              For further information see file COPYING 
// *              in the top level directory
// **************************************************************************
// * part of:     Fast Traffic Adaptive (FTA) Mac protocol
// * description: - Definie Mac package for FTA protocol
// **************************************************************************/

cplusplus {{
    #include "SimpleAddress.h"
    #include "MacPkt_m.h"
}}

class LAddress::L2Type extends void;
class MacPkt;

packet MacPktWB extends MacPkt
{
    LAddress::L2Type destAddr; // destination mac address
    LAddress::L2Type srcAddr;  // source mac address
//    long             sequenceId; // Sequence Number to detect duplicate messages
	bool           forwarded;  // This WB packet already forwarded
}