/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/hdmrp/hdmrp.h"

Define_Module(hdmrp);

void hdmrp::startup() {
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");

    // Define role
    if(appModule->hasPar("isSink")) {
        appModule->par("isSink")?setRole(hdmrpRoleDef::SINK):setRole(hdmrpRoleDef::NON_ROOT);
    } else {
        role=hdmrpRoleDef::NON_ROOT;
        throw cRuntimeError("\nHDMRP nodes require the parameter role");
    }

    if(appModule->hasPar("isMaster")) {
       setMaster(appModule->par("isMaster"));
    } else {
        setMaster(false);
    }

    // T_l timer
    t_l=par("t_l");

    // Set round to 0
    initRound();
    setState(hdmrpStateDef::WORK);

    if(isSink()) {
        setTimer(hdmrpTimerDef::SINK_START,1);
        //Trigger 1st RREQ, timer should be also here
    }
}

bool hdmrp::isSink() const {
    return hdmrpRoleDef::SINK==role;
}

bool hdmrp::isRoot() const {
    return hdmrpRoleDef::ROOT==role;
}

bool hdmrp::isSubRoot() const {
    return hdmrpRoleDef::SUB_ROOT==role;
}

bool hdmrp::isNonRoot() const {
    return hdmrpRoleDef::NON_ROOT==role;
}

void hdmrp::setRole(hdmrpRoleDef new_role) {
    role=new_role;
}

bool hdmrp::isMaster() const {
    return master; 
}

void hdmrp::setMaster(bool master) {
    this->master=master;
}

bool hdmrp::isWorkingState() const {
    return hdmrpStateDef::WORK==state;
}

bool hdmrp::isLearningState() const {
    return hdmrpStateDef::LEARN==state;
}

void hdmrp::setState(hdmrpStateDef new_state) {
    state=new_state;
}

void hdmrp::initRound() {
    round = 0;
}

void hdmrp::newRound() {
    ++round;
}

bool hdmrp::isNewRound(hdmrpPacket* pkt) const {
    return pkt->getRound()>round;
}

int hdmrp::getRound() const {
    return round;
}

void hdmrp::setRound(int new_round) {
    round=new_round;
}

void hdmrp::sendRREQ() {
    sendRREQ(getRound(),0,0,0);
}

void hdmrp::sendRREQ(int round, hdmrp_path path) {
    sendRREQ(round, path.path_id, path.nmas+isMaster()?1:0, path.len+1);
}

void hdmrp::sendRREQ(int round, int path_id) {
    sendRREQ(round,path_id,0,0);
}

void hdmrp::sendRREQ(int round, int path_id, int nmas, int len) {
    trace()<<"sendRREQ() called";
    hdmrpPacket *RREQPkt=new hdmrpPacket("HDMRP RREQ packet", NETWORK_LAYER_PACKET);
    RREQPkt->setHdmrpPacketKind(hdmrpPacketDef::RREQ_PACKET);
    RREQPkt->setSource(SELF_NETWORK_ADDRESS);
    RREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    RREQPkt->setRound(round);
    RREQPkt->setPath_id(path_id);
    RREQPkt->setNmas(nmas);
    RREQPkt->setLen(len);

    toMacLayer(RREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();

}

void hdmrp::sendSRREQ() {
    trace()<<"sendSRRQ() called";
    hdmrpPacket *SRREQPkt=new hdmrpPacket("HDMRP Sink RREQ packet", NETWORK_LAYER_PACKET);
    SRREQPkt->setHdmrpPacketKind(hdmrpPacketDef::SINK_RREQ_PACKET);
    SRREQPkt->setSource(SELF_NETWORK_ADDRESS);
    SRREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    // Per HDMRP, increment round number to trigger new RREQ round
    newRound();

    SRREQPkt->setRound(getRound());
    SRREQPkt->setPath_id(0);
    SRREQPkt->setNmas(0);
    SRREQPkt->setLen(0);

    toMacLayer(SRREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();
}

void hdmrp::storeRREQ(hdmrpPacket *pkt) {
    trace()<<"New RREQ stored";
    hdmrp_path candidate_path;
    candidate_path.path_id=pkt->getPath_id();
    candidate_path.len=pkt->getLen();
    candidate_path.nmas=pkt->getNmas();
    candidate_path.next_hop=string(pkt->getSource());

    if(rreq_table.find(candidate_path.path_id) == rreq_table.end()) {
        rreq_table[candidate_path.path_id]=candidate_path;
    } else {
        if(calculateCost(candidate_path)<calculateCost(rreq_table[candidate_path.path_id])) {
                rreq_table[candidate_path.path_id]=candidate_path;
        }
    }
    return;
}

hdmrp_path hdmrp::selectRREQ() {
    trace()<<"WTF?!?!?";
    std::random_device rd;
    uniform_int_distribution<int> dist(0, rreq_table.size()-1);
    auto it=rreq_table.begin();

    for(int i=dist(rd); i > 0 ; --i, ++it);
    trace()<<"RREQ table size: "<<rreq_table.size();
    return it->second;
}

void hdmrp::removeRREQ(hdmrp_path path) {
    rreq_table.erase(path.path_id);
}

void hdmrp::clearRREQ() {
    rreq_table.clear();
}

bool hdmrp::isRREQempty() const {
    return rreq_table.empty();
}

float hdmrp::calculateCost(hdmrp_path path) const {
    return (float)path.len/(float)path.nmas;
}

void hdmrp::addRoute(hdmrp_path path) {
    routing_table[path.path_id]=path;
}

bool hdmrp::RouteExists() const {
    return routing_table.size()>0;
}

void hdmrp::clearRoutes() {
    routing_table.clear();
}

hdmrp_path hdmrp::getRoute(const int path_id) const {
   return rreq_table.find(path_id)->second; // pretty unsafe
}

hdmrp_path hdmrp::getRoute() const {
    std::random_device rd;
    uniform_int_distribution<int> dist(0, routing_table.size()-1);
    auto it=routing_table.begin();

    for(int i=dist(rd); i > 0 ; --i, ++it);

    return it->second;
}

// Timer handling
void hdmrp::timerFiredCallback(int index) {
    switch (index) {
        case hdmrpTimerDef::SINK_START: {
            trace()<<"SINK_START timer expired";
            sendSRREQ();
            setTimer(hdmrpTimerDef::SINK_START,2);
            break;
        }
        case hdmrpTimerDef::T_L: {
            trace()<<"T_L expired";
            // select path
            auto path=selectRREQ();
            sendRREQ(getRound(),path);
            addRoute(path);
            if(isMaster()) {
                removeRREQ(path);
                if(isRREQempty()) {
                    setState(hdmrpStateDef::WORK);
                } else {
                    setState(hdmrpStateDef::RELAY);
                    setTimer(hdmrpTimerDef::T_RELAY,1);
                }
            } else {
                clearRREQ();
                setState(hdmrpStateDef::WORK);
            }
            break;
        }
        case hdmrpTimerDef::T_RELAY: {
            trace()<<"T_RELAY expired";
            auto path=selectRREQ();
            sendRREQ(getRound(),path);
            addRoute(path);
            removeRREQ(path);
            if(isRREQempty()) {
                setState(hdmrpStateDef::WORK);
            } else {
                setState(hdmrpStateDef::RELAY);
                setTimer(hdmrpTimerDef::T_RELAY,1);
            }
            break;
        }
    }
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
    auto path=getRoute();
    netPacket->setDestination(path.next_hop.c_str());
    trace()<<"Destination "<<path.next_hop;

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
            if(netPacket->getSource()==SELF_NETWORK_ADDRESS) {
                trace()<<"What?";
            }
            else if(netPacket->getSource()==BROADCAST_NETWORK_ADDRESS) {
                trace()<<"Broadcast data packet, wtf?";
            }
            else if(netPacket->getDestination()==SELF_NETWORK_ADDRESS) {
                hdmrp_path path;
                if(isSink()) {
                    trace()<<"Packet arrived";
                }
                else if(isRoot()) {
                    path=getRoute(0);
                }
                else if(isSubRoot() || isNonRoot()) {
                    path=getRoute(netPacket->getPath_id());
                }
                // Something missing
                netPacket->setSource(SELF_NETWORK_ADDRESS);
                netPacket->setDestination(path.next_hop.c_str());
            }
            break;
        }
        case hdmrpPacketDef::RREQ_PACKET: {
            if(isSink()) {
                trace()<<"RREQ discarded by sink";
            }
            else if(isRoot()) {
                trace()<<"RREQ discarded by root";
            }
            else if(isSubRoot()) {
                trace()<<"RREQ received by sub-root";
                if(0 == netPacket->getPath_id()) {
                    trace()<<"RREQ with 0 Path_id";
                    // start guard timer, send message with RREQ, learn next hop?
                    if(isNewRound(netPacket)) {
                       trace()<<"New Round";
                       setRound(netPacket->getRound());
                       sendRREQ(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                       addRoute(hdmrp_path{resolveNetworkAddress(SELF_NETWORK_ADDRESS), string(netPacket->getSource()), 0, 0}  );
                    } else {
                        trace()<<"Old round";
                    }
                } else {
                    trace()<<"RREQ with valid Path_id discarded by sub-root";
                }
            }
            else if(isNonRoot()) {
                trace()<<"RREQ received by non-root";
                if(isWorkingState()) {
                    if(0 != netPacket->getPath_id() ) {
                        if(isNewRound(netPacket)) {
                            setState(hdmrpStateDef::LEARN);
                            setRound(netPacket->getRound());
                            setTimer(hdmrpTimerDef::T_L,t_l);
                            clearRREQ();
                            storeRREQ(netPacket);
                        } else {
                            trace()<<"Old round";
                        }
                    } else {
                        // once switched to sub-root, some guard timer is needed??
                        trace()<<"RREQ with 0 Path_id received";
                        if(netPacket->getRound() > getRound()) {
                            trace()<<"New Round";
                            trace()<<"State transition to sub-root";
                            setRole(hdmrpRoleDef::SUB_ROOT);
                            setRound(netPacket->getRound());
                            sendRREQ(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                            clearRoutes();
                            clearRREQ();
                            addRoute( hdmrp_path{resolveNetworkAddress(SELF_NETWORK_ADDRESS),
                                                 string(netPacket->getSource()),
                                                 0,
                                                 0} );
                        } else {
                            trace()<<"Old round";
                        }
                    }
                }
                else if(isLearningState()) { // What about new rounds?
                    if(0 != netPacket->getPath_id()) {
                        if(netPacket->getRound() == getRound()) {
                            storeRREQ(netPacket);
                        } else {
                            trace()<<"RREQ received with invalid round during learning state";
                        }
                    } else {
                        trace()<<"RREQ with 0 Path_id discarded during learning state";
                    }
                }
            }
            break;
        }
        case hdmrpPacketDef::SINK_RREQ_PACKET: {
            if(isSink()) {
                trace()<<"SRREQ discarded by sink";
            }
            else if(isNonRoot() || isRoot()) {
                trace()<<"SRREQ received by non-root or root";
                if(isNonRoot()) {
                    if(getTimer(hdmrpTimerDef::T_L) != -1) {
                        cancelTimer(hdmrpTimerDef::T_L);
                    }
                    setRole(hdmrpRoleDef::ROOT);
                }

                if(netPacket->getRound() > getRound()) {
                    trace()<<"Old round: "<<getRound()<<" new round: "<<netPacket->getRound();
                    trace()<<"Source: "<<netPacket->getSource();
                    setRound(netPacket->getRound());
                    sendRREQ();
                    clearRREQ();
                    clearRoutes();
                    addRoute(hdmrp_path{0, string(netPacket->getSource()),0,0});
                    
                } else {
                    trace()<<"Outdated SRREQ";
                }
            }
            else if(isSubRoot()) {
                trace()<<"SRREQ received by sub-root";
            }
            break;
        }
    }
}

