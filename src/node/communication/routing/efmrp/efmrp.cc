/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2023                                        *
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

    fp.ttl   =  par("t_ttl");

    ff_app = dynamic_cast<ForestFire *>(appModule);
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
        case efmrpStateDef::BUILD: {
            return "BUILD";
        }
        case efmrpStateDef::LEARN: {
            return "LEARN";
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

void efmrp::sendField(int hop, double nrg, double env) {
    trace()<<"[info] Entering sendField(hop ="<<hop<<", nrg="<<nrg<<", env="<<env<<")";
    auto *field_pkt=new efmrpFieldPacket("EFMRP FIELD packet",NETWORK_LAYER_PACKET);
    field_pkt->setByteLength(netDataFrameOverhead);
    field_pkt->setEfmrpPacketKind(efmrpPacketDef::FIELD_PACKET);
    field_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    field_pkt->setSource(SELF_NETWORK_ADDRESS);
    field_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    field_pkt->setHop(hop);
    field_pkt->setNrg(nrg);
    field_pkt->setEnv(env);

    toMacLayer(field_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::updateHelloTable(efmrpHelloPacket *hello_pkt) {
    trace()<<"[info] Entering updateHelloTable(..)";
    node_entry ne;
    ne.nw_address=hello_pkt->getSource();
    ne.hop=hello_pkt->getHop();
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" hop: "<<ne.hop<<" to hello_table";
    hello_table.insert({hello_pkt->getSource(),ne});
}

void efmrp::updateFieldTable(efmrpFieldPacket *field_pkt) {
    trace()<<"[info] Entering updateFieldTable(..)";
    node_entry ne;
    ne.nw_address = field_pkt->getSource();
    ne.hop        = field_pkt->getHop();
    ne.nrg        = field_pkt->getNrg();
    ne.env        = field_pkt->getEnv();

    if(field_table.find(ne.nw_address) != field_table.end()) {
        trace()<<"[info] Overriding existing record of node "<<ne.nw_address;
        field_table[ne.nw_address] = ne;
    } else {
        trace()<<"[info] New record of node "<<ne.nw_address;
        field_table.insert({ne.nw_address,ne});
    }
    trace()<<"[info] hop: "<<ne.hop<<" nrg: "<<ne.nrg<<"env: "<<ne.env;
}

void efmrp::timerFiredCallback(int index) {
    switch (index) {
        case efmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            break;
        }
        case efmrpTimerDef::TTL: {
            trace()<<"[timer] TTL timer expired";
            setState(efmrpStateDef::BUILD);
            sendField(getHop(), ff_app->getEnergyValue(), ff_app->getEmergencyValue());
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
                // Add some random to TTL to compensate propagation delay
                setTimer(efmrpTimerDef::TTL, fp.ttl + getRNG(0)->doubleRand());
            }

            if(hello_pkt->getHop()<getHop()) {
                trace()<<"[info] Updating hop status";
                setHop(hello_pkt->getHop()+1);
                updateHelloTable(hello_pkt);
                sendHello(getHop(), hello_pkt->getTimestamp());
            }

            break;
        }

        case efmrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
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

