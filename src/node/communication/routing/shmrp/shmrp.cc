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
//                sendRreqs();
//                sendRinv(getRound());
            }
            break;
        }
        case shmrpTimerDef::T_ESTABLISH: {
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
            break;
        }
        case shmrpPacketDef::RRESP_PACKET: {
            trace()<<"[info] RRESP_PACKET received";
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
