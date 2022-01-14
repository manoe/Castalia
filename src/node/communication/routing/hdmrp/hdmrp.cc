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
    // Define role
    if(appModule->hasPar("role")) {
        std::string role_str=appModule->par("role");
        if(0==role_str.compare("NON_ROOT")) {
            role=hdmrpRoleDef::NON_ROOT;
        }
        else if(0==role_str.compare("ROOT")) {
            role=hdmrpRoleDef::ROOT;
        }
        else if(0==role_str.compare("SINK")) {
            role=hdmrpRoleDef::SINK;
        }
        else if(0==role_str.compare("SUB_ROOT")) {
            role=hdmrpRoleDef::SUB_ROOT;
        }
        else {
            trace()<<"Invalid role parameter value, fallback to NON_ROOT";
            role=hdmrpRoleDef::NON_ROOT;
        }
    } else {
        role=hdmrpRoleDef::NON_ROOT;
//        throw cRuntimeError("\nHDMRP nodes require the parameter role");
    }

    t_l=par("t_l");

    // Set round to 0
    round=0;
    state=hdmrpStateDef::WORK;

    if(isSink()) {
        setTimer(hdmrpTimerDef::SINK_START,1);
        //Trigger 1st RREQ, timer should be also here
    }
}

void hdmrp::timerFiredCallback(int index) {
    if(hdmrpTimerDef::SINK_START == index) {
        trace()<<"SINK_START timer expired";
        sendRREQ();
    }
    setTimer(hdmrpTimerDef::SINK_START,2);
}

bool hdmrp::isSink() const {
    return hdmrpRoleDef::SINK==role;
}


void hdmrp::sendRREQ() {
    trace()<<"sendRRQ() called";
    if(isSink()) {
        hdmrpPacket *RREQPkt=new hdmrpPacket("HDMRP RREQ packet", NETWORK_LAYER_PACKET);
        RREQPkt->setHdmrpPacketKind(hdmrpPacketDef::SINK_RREQ_PACKET);
        RREQPkt->setSource(SELF_NETWORK_ADDRESS);
        RREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

        // Per HDMRP, increment round number to trigger new RREQ round
        round+=1;

        RREQPkt->setRound(round);
        RREQPkt->setPath_id(0);
        RREQPkt->setNmas(0);
        RREQPkt->setLen(0);

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
            else if(role == hdmrpRoleDef::ROOT) {
                trace()<<"Root node received RREQ";
                if(netPacket->getRound() > round) {
                    trace()<<"Root node received RREQ with higher round number.";
                    if(0 == netPacket->getPath_id()) {
                    }
                }
            }
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
//        string destination(netPacket->getDestination());
//        if (destination.compare(SELF_NETWORK_ADDRESS) == 0 ||
//            destination.compare(BROADCAST_NETWORK_ADDRESS) == 0) {
//            toApplicationLayer(decapsulatePacket(pkt));
//    }
}

