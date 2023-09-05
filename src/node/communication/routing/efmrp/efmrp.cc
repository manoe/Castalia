/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/efmrp/efmrp.h"

Define_Module(efmrp);

void efmrp::startup() {
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
        setTimer(efmrpTimerDef::SINK_START,par("t_start"));
        setState(efmrpStateDef::WORK);
    } else {
        setHop(std::numeric_limits<int>::max());
        setState(efmrpStateDef::INIT);
    }

    fp.ttl              = par("ttl");

}

bool efmrp::isSink() const {
    return g_is_sink;
}

void efmrp::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string efmrp::getSinkAddress() const {
    return g_sink_addr;
}

void efmrp::setHop(int hop) {
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int efmrp::getHop() const {
    return g_hop;
}

void efmrp::setState(efmrpStateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string efmrp::stateToStr(efmrpStateDef state) const {
    switch (state) {
        case efmrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case efmrpStateDef::WORK: {
            return "WORK";
        }
        case efmrpStateDef::INIT: {
            return "INIT";
        }
        case efmrpStateDef::LEARN: {
            return "LEARN";
        }
        case efmrpStateDef::ESTABLISH: {
            return "ESTABLISH";
        }
        case efmrpStateDef::MEASURE: {
            return "MEASURE";
        }
    }
    return "UNKNOWN";
       
}

efmrpStateDef efmrp::getState() const {
    return g_state;
}

void efmrp::sendHello() {
    trace()<<"[info] Entering sendHello()";
    sendHello(0, getClock().dbl());
}

void efmrp::sendHello(int hop, double timestamp) {
    trace()<<"[info] Entering sendHello(hop="<<hop<<", timestamp="<<timestamp<<")";
    auto *hello_pkt=new efmrpHelloPacket("EFMRP HELLO packet", NETWORK_LAYER_PACKET);
    hello_pkt->setByteLength(netDataFrameOverhead);
    hello_pkt->setEfmrpPacketKind(efmrpPacketDef::HELLO_PACKET);
    hello_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    hello_pkt->setSource(SELF_NETWORK_ADDRESS);
    hello_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    hello_pkt->setHop(hop);
    hello_pkt->setTimestamp(timestamp);

    toMacLayer(hello_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::updateHelloTable(efmrpHelloPacket *hello_pkt) {
    trace()<<"[info] Entering updateHelloTable(..)";
    node_entry ne;
    ne.nw_address=hello_pkt->getSource();
    ne.hop=hello_pkt->getHop();
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" hop: "<<ne.hop<<" to hello_table";
    hello_table[hello_pkt->getSource()]=ne;
}


void efmrp::timerFiredCallback(int index) {
    switch (index) {
        case efmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void efmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    }

    switch (getState()) {
        case efmrpStateDef::LEARN: {
            trace()<<"[error] In LEARNING state, can't route packet";
            return;
        }
        case efmrpStateDef::MEASURE: {
            trace()<<"[info] In MEASURE state, routing round-robin";
        }

    }

}

void efmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    efmrpPacket *efmrp_pkt=dynamic_cast<efmrpPacket *>(pkt);
    if(!efmrp_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    trace()<<"[info] EFMRP packet received from MAC: "<<srcMacAddress<<" NW: "<<efmrp_pkt->getSource();

    switch (efmrp_pkt->getEfmrpPacketKind()) {
        case efmrpPacketDef::HELLO_PACKET: {
            trace()<<"[info] HELLO_PACKET received";
            efmrpHelloPacket *hello_pkt=dynamic_cast<efmrpHelloPacket *>(pkt);
            if(getClock().dbl() - hello_pkt->getTimestamp() > fp.ttl) {
                trace()<<"[info] HELLO_PACKET expired, timestamp: "<<hello_pkt->getTimestamp()<<" clock: "<<getClock().dbl();
                break;
            }
            if(getState()==efmrpStateDef::INIT) {
                trace()<<"[info] Node in INIT state, transitioning to LEARN and arming TTL timer";
                setState(efmrpStateDef::LEARN);
                setTimer(efmrpTimerDef::TTL, fp.ttl );
            }

            if(hello_pkt->getHop()<getHop()) {
                setHop(hello_pkt->getHop()+1);
                updateHelloTable(hello_pkt);
                sendHello(getHop(), hello_pkt->getTimestamp());
            }

            break;
        }

        case efmrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            efmrpDataPacket *data_pkt=dynamic_cast<efmrpDataPacket *>(pkt);
            break;
        }
        case efmrpPacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with efmrpPacketKind value: "<<efmrp_pkt->getEfmrpPacketKind();
            break;
        }
    }
}

void efmrp::finishSpecific() {
    return;
}

void efmrp::handleMacControlMessage(cMessage *msg) {
    trace()<<"[info] Entering handleMacControlMessage()";
    TMacControlMessage *mac_msg=check_and_cast<TMacControlMessage *>(msg);
    if(!mac_msg) {
        trace()<<"[error] Not TMacControlMessage";
    }
    trace()<<"[info] Event: "<<mac_msg->getMacControlMessageKind()<<" Node: "<<mac_msg->getDestination()<<" Seqnum: "<<mac_msg->getSeq_num();
    delete msg;
}
