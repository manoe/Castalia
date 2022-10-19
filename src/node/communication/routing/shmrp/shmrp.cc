/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/shmrp/shmrp.h"

Define_Module(shmrp);

void shmrp::startup() {
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");
    if(appModule->hasPar("isSink")) {
        g_is_sink=appModule->par("isSink");
    } else {
        g_is_sink=false;
    }

    if(hasPar("sink_address")) {
        g_sink_addr.assign(par("sink_address").stringValue());
    } else {
        trace()<<"[error] No sink address provisioned";
    }

    if(isSink()) {
        setHop(0);
        setRound(1);
        setTimer(shmrpTimerDef::SINK_START,par("t_start"));
        setState(shmrpStateDef::WORK);
    } else {
        setHop(std::numeric_limits<int>::max());
        setRound(0);
        setState(shmrpStateDef::INIT);
    }

    g_t_l=par("t_l");
    g_ring_radius=par("ring_radius");
    g_t_est=par("t_est");

}

bool shmrp::isSink() const {
    return g_is_sink;
}

void shmrp::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string shmrp::getSinkAddress() const {
    return g_sink_addr;
}

void shmrp::setHop(int hop) {
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int shmrp::getHop() const {
    return g_hop;
}

void shmrp::calculateHop() {
    trace()<<"[info] Entering shmrp::calculateHop()";
    if(rinv_table.empty()) {
        throw std::length_error("[error] RINV table empty");
    }
    int hop=std::numeric_limits<int>::max();
    for(auto ne: rinv_table) {
        if(hop > ne.second.hop) {
            hop=ne.second.hop;
        }
    }
    setHop(hop+1);
}

void shmrp::setRound(int round) {
    trace()<<"[info] Changing round "<<g_round<<" to "<<round;
    g_round=round;
}

int shmrp::getRound() const {
    return g_round;
}

void shmrp::setState(shmrpStateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string shmrp::stateToStr(shmrpStateDef state) const {
    switch (state) {
        case shmrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case shmrpStateDef::WORK: {
            return "WORK";
        }
        case shmrpStateDef::INIT: {
            return "INIT";
        }
        case shmrpStateDef::LEARN: {
            return "LEARN";
        }
        case shmrpStateDef::ESTABLISH: {
            return "ESTABLISH";
        }
    }
    return "UNKNOWN";
       
}

shmrpStateDef shmrp::getState() const {
    return g_state;
}


void shmrp::sendRinv(int round, int pathid) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<", pathid = "<<pathid<<")";
    shmrpRinvPacket *rinv_pkt=new shmrpRinvPacket("SHMRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setShmrpPacketKind(shmrpPacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setPathid(0);
    rinv_pkt->setHop(getHop());
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp::sendRinv(int round) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<")";
    sendRinv(round,0);
}

void shmrp::clearRinvTable() {
    trace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void shmrp::addToRinvTable(shmrpRinvPacket *rinv_pkt) {
    trace()<<"[info] Add entry to RINV table - source: "<<rinv_pkt->getSource()<<" pathid: "<<rinv_pkt->getPathid()<<" hop: "<<rinv_pkt->getHop();
    node_entry ne;
    ne.nw_address.assign(rinv_pkt->getSource());
    ne.pathid = rinv_pkt->getPathid();
    ne.hop = rinv_pkt->getHop();
    if(rinv_table.find(ne.nw_address) != rinv_table.end()) {
        trace()<<"[info] Entry already exists, overriding";
    }
    rinv_table.insert({ne.nw_address, ne});
}

void shmrp::clearRreqTable() {
    trace()<<"[info] RREQ table erased";
    rreq_table.clear();
}

bool shmrp::isRreqTableEmpty() const {
    return rreq_table.empty();
}

void shmrp::constructRreqTable(shmrpRingDef ring) {
    trace()<<"[info] Entering shmrp::constructRreqTable(ring = "<<ring<<" )";
    if(rinv_table.empty()) {
        throw std::length_error("[error] RINV table empty");
    }
    if(!rreq_table.empty()) {
        throw std::length_error("[error] RREQ table not empty");
    }

    switch (ring) {
        case shmrpRingDef::INTERNAL:
        case shmrpRingDef::BORDER: {
            for(auto ne: rinv_table) {
                if(ne.second.hop < getHop()) {
                    trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<ne.second.pathid;
                    rreq_table.insert(ne);
                }
            }
            break;
        }
        case shmrpRingDef::EXTERNAL: {
            break;
        }
        default: {
            trace()<<"[error] Unknown ring level";
            break;
        }
    }
}

bool shmrp::rreqEntryExists(const char *addr, int pathid) {
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        return true;
    }
    return false;
}

void shmrp::updateRreqTableWithRresp(const char *addr, int pathid) {
    trace()<<"[info] Entering shmrp::UpdateRreqTableWithRresp(addr="<<addr<<", pathid="<<pathid;
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        rreq_table[string(addr)].rresp=true;
    } else {
        throw std::length_error("[error] Entry not found");
    }
}


void shmrp::sendRreqs() {
    trace()<<"[info] Entering shmrp::sendRreqs()";
    if(rreq_table.empty()) {
       throw std::length_error("[error] RREQ table empty");
    }
    for(auto ne: rreq_table) {
        shmrpRreqPacket* rreq_pkt=new shmrpRreqPacket("SHMRP RREQ packet",NETWORK_LAYER_PACKET);
        rreq_pkt->setByteLength(netDataFrameOverhead);
        rreq_pkt->setShmrpPacketKind(shmrpPacketDef::RREQ_PACKET);
        rreq_pkt->setSource(SELF_NETWORK_ADDRESS);
        rreq_pkt->setDestination(ne.second.nw_address.c_str());
        rreq_pkt->setRound(getRound());
        rreq_pkt->setPathid(ne.second.pathid);
        rreq_pkt->setSequenceNumber(currentSequenceNumber++);
        trace()<<"[info] Sending RREQ to "<<ne.second.nw_address<<" with pathid: "<<ne.second.pathid;
        toMacLayer(rreq_pkt, resolveNetworkAddress(ne.second.nw_address.c_str()));
    }
}

void shmrp::sendRresp(const char *dest, int round, int pathid) {
    trace()<<"[info] Sending RRESP to "<<dest<<" with round "<<round<<" and pathid "<<pathid;
    shmrpRrespPacket* rresp_pkt=new shmrpRrespPacket("SHMRP RRESP packet",NETWORK_LAYER_PACKET);
    rresp_pkt->setByteLength(netDataFrameOverhead);
    rresp_pkt->setShmrpPacketKind(shmrpPacketDef::RRESP_PACKET);
    rresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    rresp_pkt->setDestination(dest);
    rresp_pkt->setRound(round);
    rresp_pkt->setPathid(pathid);
    rresp_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rresp_pkt, resolveNetworkAddress(dest));
}


void shmrp::clearRoutingTable() {
    trace()<<"[info] Routing table erased";
    routing_table.clear();
}

void shmrp::constructRoutingTable() {
    trace()<<"[info] Entering shmrp::constructRoutingTable()";

}

void shmrp::timerFiredCallback(int index) {
    switch (index) {
        case shmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendRinv(getRound());
            setRound(1+getRound());
            break;
        }
        case shmrpTimerDef::T_L: {
            trace()<<"[timer] T_L timer expired";
            if(shmrpStateDef::LEARN != getState()) {
                trace()<<"[error] State is not LEARN: "<<stateToStr(getState());
                break;
            }
            setState(shmrpStateDef::ESTABLISH);
            calculateHop();
            clearRreqTable();
            if(getHop() <= g_ring_radius) {
                trace()<<"[info] Node inside mesh ring";
                try {
                    constructRreqTable(shmrpRingDef::INTERNAL);
                } catch (std::exception &e) {
                    trace()<<e.what();
                    break;
                }
                sendRreqs();
                setTimer(shmrpTimerDef::T_ESTABLISH,g_t_est);
//                sendRinv(getRound());
            }
            break;
        }
        case shmrpTimerDef::T_ESTABLISH: {
            trace()<<"[timer] T_ESTABLISH timer expired";
            setState(shmrpStateDef::WORK);
            
            if(isRreqTableEmpty()) {
                trace()<<"[error] RREQ table empty";
                break;
            }

            clearRoutingTable();

            try {
                constructRoutingTable();
            } catch (std::exception &e) {
                trace()<<e.what();
                break;
            }

            if(getHop() < g_ring_radius) {
                trace()<<"[info] Node inside mesh ring";
                sendRinv(getRound());

            } else if(getHop() == g_ring_radius) {
                trace()<<"[info] Node at mesh ring border";
                sendRinv(getRound(), resolveNetworkAddress(SELF_NETWORK_ADDRESS));
            }
            else {
                trace()<<"[info] Node outside mesh ring";
            }

            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void shmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    }
}

void shmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    shmrpPacket *net_pkt=dynamic_cast<shmrpPacket *>(pkt);
    if(!net_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    switch (net_pkt->getShmrpPacketKind()) {
        case shmrpPacketDef::RINV_PACKET: {
            trace()<<"[info] RINV_PACKET received";
            auto rinv_pkt=dynamic_cast<shmrpRinvPacket *>(pkt);

            if(isSink()) {
                trace()<<"[info] RINV_PACKET rejected by Sink";
                break;
            }

            if(rinv_pkt->getRound() > getRound()) {
                setRound(rinv_pkt->getRound());
                clearRinvTable();
                if(shmrpStateDef::LEARN != getState()) {
                    setState(shmrpStateDef::LEARN);
                } else {
                    cancelTimer(shmrpTimerDef::T_L);
                }                    
                setTimer(shmrpTimerDef::T_L,g_t_l);

                addToRinvTable(rinv_pkt);

                /* start learning process */
            }

            break;
        }
        case shmrpPacketDef::RREQ_PACKET: {
            trace()<<"[info] RREQ_PACKET received";
            auto rreq_pkt=dynamic_cast<shmrpRreqPacket *>(pkt);
            if(rreq_pkt->getRound() != getRound()) {
                trace()<<"[error] RREQ_PACKET's round: "<<rreq_pkt->getRound()<<" does not equal local round: "<<getRound();
                break;
            }
            if(getHop()<g_ring_radius) {
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            } else if(getHop()==g_ring_radius) {

            }
            break;
        }
        case shmrpPacketDef::RRESP_PACKET: {
            trace()<<"[info] RRESP_PACKET received";
            auto rresp_pkt=dynamic_cast<shmrpRrespPacket *>(pkt);
            if(rreqEntryExists(rresp_pkt->getSource(),rresp_pkt->getPathid())) {
                updateRreqTableWithRresp(rresp_pkt->getSource(),rresp_pkt->getPathid());
            } else {
                trace()<<"[error] No entry in RREQ table with address "<<rresp_pkt->getSource()<<" and pathid: "<<rresp_pkt->getPathid();
                break;
            }
            break;
        }
        case shmrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            break;
        }
        case shmrpPacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with shmrpPacketKind value: "<<net_pkt->getShmrpPacketKind();
            break;
        }
    }
}

void shmrp::finishSpecific() {
    return;
}
