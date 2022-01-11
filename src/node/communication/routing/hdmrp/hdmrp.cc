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

#include "node/communication/routing/hdmrp/hdmrp.h"

Define_Module(hdmrp);

void hdmrp::startup() {
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");
    if(appModule->hasPar("isSink")) {
        role = appModule->par("isSink") ? hdmrpRoleDef::SINK : hdmrpRoleDef::NON_ROOT;
    } else {
        throw cRuntimeError("\nHDMRP nodes require the parameter isSink");
    }

    t_l=par("t_l");

    // Set round to 0
    round=0;
    state=hdmrpStateDef::WORK;

    if(isSink) {
        sendRREQ();
        //Trigger 1st RREQ, timer should be also here
    }
}

void hdmrp::sendRREQ() {
    if(isSink) {
        hdmrpPacket *RREQPkt=new hdmrpPacket("HDMRP RREQ packet", NETWORK_LAYER_PACKET);
        RREQPkt->setHdmrpPacketKind(hdmrpPacketDef::RREQ_PACKET);
        RREQPkt->setSource(SELF_NETWORK_ADDRESS);
        RREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

        // Per HDMRP, increment round number to trigger new RREQ round
        round+=1;

        RREQPkt->setRound(round);
        toMacLayer(RREQPkt, BROADCAST_MAC_ADDRESS);
    }
}

void hdmrp::storeRREQ(hdmrpPacket *pkt) {
    return;
}


/* Application layer sends a packet together with a dest network layer address.
 * Network layer is responsible to route that packet by selecting an appropriate
 * MAC address. With BypassRouting we do not perform any routing function. We
 * just encapsulate the app packet and translate the net address to a MAC address
 * (e.g. "3" becomes 3, or a BROADCAST_NETWORK_ADDRESS becomes BROADCAST_MAC_ADDRESS)
 * If the destination is a 1-hop neighbor it will receive the packet.
 */
void hdmrp::fromApplicationLayer(cPacket * pkt, const char *destination)
{
    hdmrpPacket *netPacket = new hdmrpPacket("HDMRP packet", NETWORK_LAYER_PACKET);
    netPacket->setSource(SELF_NETWORK_ADDRESS);
    netPacket->setDestination(destination);
    std::cout<<"Destination "<<destination<<std::endl;
    encapsulatePacket(netPacket, pkt);
    toMacLayer(netPacket, resolveNetworkAddress(destination));
}

/* MAC layer sends a packet together with the source MAC address and other info.
 * With BypassMAC we just filter the packet by the NET address. If this
 * node is the right destination decapsulatePacket will extract the APP packet.
 * If not, there is no need to do anything. The whole net packet (including
 * the encapsulated apppacket will be deleted by the virtual routing code
 */
void hdmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi)
{
    trace()<<this->selfAddress<<" "<<this->self<<" Packet received"<<" RSSI: "<<rssi;
    hdmrpPacket *netPacket = dynamic_cast <hdmrpPacket*>(pkt);

    if (!netPacket) {
        trace()<<"dynamic_cast of packet failed";
        return;
    }

    switch(netPacket->getHdmrpPacketKind()) {
        case hdmrpPacketDef::DATA_PACKET: {
            // process or route somewhere
            break;
        }
        case hdmrpPacketDef::RREQ_PACKET: {
            if(role == hdmrpRoleDef::SINK) {
                trace()<<"RREQ discarded by sink";
            }
           // else if(
            else {
                if(hdmrpStateDef::WORK == state) {
                    // New round
                    if(netPacket->getRound() > round) {
                        storeRREQ(netPacket);
                        state=hdmrpStateDef::LEARN;
                        //startTIMER
                    }
                }
            }
            break;
        }

    }

        // route or process
        string destination(netPacket->getDestination());
        if (destination.compare(SELF_NETWORK_ADDRESS) == 0 ||
            destination.compare(BROADCAST_NETWORK_ADDRESS) == 0) {
            toApplicationLayer(decapsulatePacket(pkt));
    }
}

