/*******************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2011                            *
 *  Developed at the ATP lab, Networked Systems research theme                 *
 *  Author(s): Athanassios Boulis, Yuriy Tselishchev                           *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *  If you do not find this file, copies can be found by writing to:           *
 *                                                                             *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                *
 *      Attention:  License Inquiry.                                           *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/flooding/flooding.h"

Define_Module(flooding);

void flooding::startup() {
    repeat = par("repeat");
}

void flooding::fromApplicationLayer(cPacket * pkt, const char *destination)
{
	FloodingPacket *netPacket = new FloodingPacket("BypassRouting packet", NETWORK_LAYER_PACKET);
	netPacket->setSource(SELF_NETWORK_ADDRESS);
	netPacket->setDestination(destination);
	encapsulatePacket(netPacket, pkt);
	toMacLayer(netPacket, BROADCAST_MAC_ADDRESS);
    trace()<<"Packet "<<netPacket->getSequenceNumber()<<" sent to "<<destination;
}

/* MAC layer sends a packet together with the source MAC address and other info.
 * With BypassMAC we just filter the packet by the NET address. If this
 * node is the right destination decapsulatePacket will extract the APP packet.
 * If not, there is no need to do anything. The whole net packet (including
 * the encapsulated apppacket will be deleted by the virtual routing code
 */
void flooding::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
	RoutingPacket *netPacket = dynamic_cast <RoutingPacket*>(pkt);
    trace()<<"From: "<<srcMacAddress;
    if(netPacket && isNotDuplicatePacket(pkt)) {
		string destination(netPacket->getDestination());
        trace()<<"Destination: "<<destination;
		if (destination.compare(SELF_NETWORK_ADDRESS) == 0 ||
		    destination.compare(BROADCAST_NETWORK_ADDRESS) == 0) {
			toApplicationLayer(decapsulatePacket(pkt));
            trace()<<"Forwarding to application layer";
        } else {
            trace()<<"Broadcasting packet "<<netPacket->getSequenceNumber()<<" destined to "<<destination<<" Size: "<<netPacket->getByteLength();
            for(int i=0 ; i < repeat ; ++i) {
                toMacLayer(netPacket->dup(), resolveNetworkAddress(BROADCAST_NETWORK_ADDRESS));
            }
        }     
    } else {
        trace()<<"Duplicate packet: "<<netPacket->getSequenceNumber()<<" from: "<<netPacket->getSource();
    }
}

