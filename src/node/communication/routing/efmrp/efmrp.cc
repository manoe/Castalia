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
        setState(efmrpStateDef::INIT);
    } else {
        setHop(std::numeric_limits<int>::max());
        setState(efmrpStateDef::INIT);
    }

    fp.ttl   =  par("t_ttl");
    fp.field =  par("t_field");
    fp.query =  par("t_query");

    fp.alpha =  par("p_alpha");
    fp.beta  =  par("p_beta");
    fp.pnum  =  par("p_pnum");

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
    if(hello_table.find(ne.nw_address) != hello_table.end()) {
        trace()<<"[warn] Overriding record's hop: "<<hello_table[ne.nw_address].hop;
    }
    hello_table.insert({hello_pkt->getSource(),ne});
}

bool efmrp::checkHelloTable(std::string nw_address) {
    trace()<<"[info] Entering checkHelloTable(nw_address="<<nw_address<<")";
    if(hello_table.find(nw_address) != hello_table.end()) {
        return true;
    }
    return false;
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

void efmrp::addRoutingEntry(std::string nw_address, node_entry ne, int prio) {
    addRoutingEntry(nw_address, ne, prio, efmrpPathStatus::AVAILABLE, 0.0);
}

void efmrp::addRoutingEntry(std::string nw_address, node_entry ne, int prio, efmrpPathStatus status, double timestamp) {
    trace()<<"[info] Entering addRoutingEntry(nw_address="<<nw_address<<", node_entry.nw_address="<<ne.nw_address<<", prio="<<prio<<", status="<<status<<", timestamp="<<timestamp;
    for(auto it=routing_table.begin() ; it != routing_table.end() ; ++it) {
        if(it->nw_address == nw_address && it->prio == prio) {
            std::string("[error] record with prio already exists");
        }
    }
    routing_entry re;
    re.nw_address=nw_address;
    re.next_hop=ne.nw_address;
    re.target_value=targetFunction(ne);
    re.status=status;
    re.prio=prio;
    re.query_timestamp=timestamp;
    routing_table.push_back(re);
}

node_entry efmrp::getNthTargetValueEntry(int order) {
    trace()<<"[info] Entering getNthTargetValueEntry(order="<<order<<")";
    if(field_table.size() < order) {
        throw std::string("[error] Less record than order");
    }

    std::vector<node_entry> fv;
    for(auto it = field_table.begin() ; it != field_table.end() ; ++it) {
        fv.push_back(it->second);
    }

    std::sort(fv.begin(), fv.end(), [this](node_entry a, node_entry b) { return targetFunction(a) < targetFunction(b);  });
    return fv[order-1];
}

double efmrp::targetFunction(node_entry a) {
    trace()<<"[info] Entering targetFunction(a)";
    double ret_val = (1.0 - fp.alpha - fp.beta) * 1.0/(a.hop + 1) +
                     fp.alpha * a.env + fp.beta * a.nrg;
    trace()<<"[info] targetFunction value: "<<ret_val;
    return ret_val;
}


int efmrp::numOfAvailPaths(std::string ne) {
    trace()<<"[info] Entering numOfAvailPaths(ne="<<ne<<")";
    int ret_val=0;
    for(auto re: routing_table) {
        if(ne == re.nw_address && re.status==efmrpPathStatus::AVAILABLE) {
            ++ret_val;
        }
    }
    return ret_val;
}

void efmrp::sendQuery(std::string ne) {
    trace()<<"[info] Entering sendQuery(ne="<<ne<<")";
    efmrpQueryPacket *query_pkt=new efmrpQueryPacket("EFMRP QUERY packet", NETWORK_LAYER_PACKET);
    query_pkt->setByteLength(netDataFrameOverhead);
    query_pkt->setEfmrpPacketKind(efmrpPacketDef::QUERY_PACKET);
    query_pkt->setOrigin(ne.c_str());
    query_pkt->setSource(SELF_NETWORK_ADDRESS);
    query_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    toMacLayer(query_pkt, BROADCAST_MAC_ADDRESS);

}

void efmrp::sendQueryAck(std::string origin, std::string dst, bool used) {
    trace()<<"[info] Entering sendQuery(origin="<<origin<<", dst="<<dst<<", used="<<used<<")";
    efmrpQueryAckPacket *query_ack_pkt=new efmrpQueryAckPacket("EFMRP QUERY_ACL packet", NETWORK_LAYER_PACKET);
    query_ack_pkt->setByteLength(netDataFrameOverhead);
    query_ack_pkt->setEfmrpPacketKind(efmrpPacketDef::QUERY_ACK_PACKET);
    query_ack_pkt->setOrigin(origin.c_str());
    query_ack_pkt->setSource(SELF_NETWORK_ADDRESS);
    query_ack_pkt->setDestination(dst.c_str());

    query_ack_pkt->setUsed(used);

    toMacLayer(query_ack_pkt, resolveNetworkAddress(dst.c_str()));

}

bool efmrp::queryStarted(std::string ne) {
    trace()<<"[info] Entering queryStarted(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==efmrpPathStatus::UNDER_QUERY && re.prio>1 && re.query_timestamp+fp.query > getClock().dbl()) {
            trace()<<"[info] Query ongoing";
            return true;
        }
    }
    return false;
}

bool efmrp::queryCompleted(std::string ne) {
    trace()<<"[info] Entering queryStarted(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==efmrpPathStatus::UNDER_QUERY && re.prio>1 && re.query_timestamp+fp.query < getClock().dbl()) {
            trace()<<"[info] Query completed";
            return true;
        }
    }
    return false;
}


void efmrp::sendData(routing_entry re, cPacket *pkt) {
    trace()<<"[info] Entering sendData(re.next_hop="<<re.next_hop;
    efmrpDataPacket *data_pkt=new efmrpDataPacket("EFMRP DATA packet", NETWORK_LAYER_PACKET);

    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setEfmrpPacketKind(efmrpPacketDef::DATA_PACKET);
    data_pkt->setOrigin(re.nw_address.c_str());
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(re.next_hop.c_str());

    data_pkt->setPri(re.prio);

    data_pkt->encapsulate(pkt);

    toMacLayer(data_pkt, resolveNetworkAddress(re.next_hop.c_str()));

}



bool efmrp::checkPath(std::string ne) {
    trace()<<"[info] Entering checkPath(ne="<<ne<<")";
    for(auto entry: routing_table) {
        if(entry.nw_address == ne) {
            trace()<<"[info] Entry found";
            return true;
        }
    }
    trace()<<"[info] Entry not found.";
    return false;
}

routing_entry efmrp::getPath(std::string ne) {
    trace()<<"[info] Entering getPath(ne="<<ne<<")";
    std::vector<routing_entry> rv;
    double tv_sum=0;
    double tv=0;
    double rnd=getRNG(0)->doubleRand();

    for(auto re: routing_table) {
        if(re.nw_address==ne) {
            rv.push_back(re);
            tv_sum+=re.target_value;
            trace()<<"[info] Adding entry ne: "<<re.nw_address<<" next hop: "<<re.next_hop<<" tv: "<<re.target_value;
        }
    }
    if(rv.size()==0) {
        throw std::string("[error] No routing entry");
    }

    trace()<<"[info] rand: "<<rnd<<" tv_sum: "<<tv_sum;

    for(auto re: rv) {
        tv+=re.target_value;
        if(rnd<tv) {
            trace()<<"[info] Selected entry: "<<re.next_hop;
            return re;
        }
    }

    trace()<<"[error] No route selected based on probability. Selecting first one next_hop: "<<rv[0].next_hop;
    return rv[0];
}

void efmrp::updateFieldTableWithQA(efmrpQueryAckPacket *query_ack_pkt) {
    trace()<<"[info] Entering updateFieldTableWithQA()";
}

void efmrp::timerFiredCallback(int index) {
    switch (index) {
        case efmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            setTimer(efmrpTimerDef::TTL, fp.ttl + getRNG(0)->doubleRand());
            setState(efmrpStateDef::LEARN);
            break;
        }
        case efmrpTimerDef::TTL: {
            trace()<<"[timer] TTL timer expired";
            setState(efmrpStateDef::BUILD);
            sendField(getHop(), ff_app->getEnergyValue(), ff_app->getEmergencyValue());
            setTimer(efmrpTimerDef::FIELD, fp.field + getRNG(0)->doubleRand());
        }
        case efmrpTimerDef::FIELD: {
            trace()<<"[timer] FIELD timer expired";
            setState(efmrpStateDef::WORK);
            trace()<<"[info] Construct primary path";
            routing_table.clear();
            addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(1),1);
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
            trace()<<"[error] In LEARN state, can't route packet";
            break;;
        }
        case efmrpStateDef::BUILD: {
            trace()<<"[info] In BUILD state, best effort routing";
            break;
        }
        case efmrpStateDef::WORK: {
            trace()<<"[info] In WORK state, routing";
            if(numOfAvailPaths(SELF_NETWORK_ADDRESS)==0) {
                trace()<<"[error] No route available";
                break;
            }
            if(numOfAvailPaths(SELF_NETWORK_ADDRESS)<fp.pnum) {
                trace()<<"[info] Not all paths are available";
                if(!queryStarted(SELF_NETWORK_ADDRESS)) {
                    node_entry ne;
                    addRoutingEntry(SELF_NETWORK_ADDRESS, ne, numOfAvailPaths(SELF_NETWORK_ADDRESS)+1, efmrpPathStatus::UNDER_QUERY,getClock().dbl());
                    sendQuery(SELF_NETWORK_ADDRESS);
                } else {
                    if(queryCompleted(SELF_NETWORK_ADDRESS)) {

                    }
                }
                sendData(getPath(SELF_NETWORK_ADDRESS),pkt);
                break;
            }
            if(numOfAvailPaths(SELF_NETWORK_ADDRESS)==fp.pnum) {
                trace()<<"[info] All paths are available, performing traffic allocation";
                sendData(getPath(SELF_NETWORK_ADDRESS),pkt);
                break;
            }
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

            if(isSink()) {
                trace()<<"[info] Node is sink, HELLO_PACKET discarded";
                break;
            }

            if(getState()==efmrpStateDef::BUILD) {
                trace()<<"[info] Node in BUILD state, discarding HELLO_PACKET";
                break;
            }

            if(getState()==efmrpStateDef::WORK) {
                trace()<<"[info] Node in WORK state, discarding HELLO_PACKET";
                // should be re-learn?
                break;
            }

            if(getState()==efmrpStateDef::INIT) {
                trace()<<"[info] Node in INIT state, transitioning to LEARN and arming TTL timer";
                setState(efmrpStateDef::LEARN);
                // Add some random to TTL to compensate propagation delay
                setTimer(efmrpTimerDef::TTL, fp.ttl + getRNG(0)->doubleRand());
            }

            if(hello_pkt->getHop()+1<getHop()) {
                trace()<<"[info] Updating hop status";
                setHop(hello_pkt->getHop()+1);
                sendHello(getHop(), hello_pkt->getTimestamp());
            }
            updateHelloTable(hello_pkt);

            break;
        }
        case efmrpPacketDef::FIELD_PACKET: {
            trace()<<"[info] FIELD_PACKET received";
            efmrpFieldPacket *field_pkt=dynamic_cast<efmrpFieldPacket *>(efmrp_pkt);
            if(isSink()) {
                trace()<<"[info] Node is sink, FIELD_PACKET discarded";
                break;
            }

            if(getState()==efmrpStateDef::LEARN) {
                trace()<<"[info] Node in LEARN state";
                updateFieldTable(field_pkt);
            }
            if(getState()==efmrpStateDef::BUILD) {
                trace()<<"[info] Node in BUILD state";
                if(checkHelloTable(std::string(field_pkt->getSource()))) {
                    updateFieldTable(field_pkt);
                } else {
                    trace()<<"[info] Entry not found in hello table, discarding";
                }            
            }
            if(getState()==efmrpStateDef::WORK) {
                trace()<<"[info] Node in WORK state, discarding FIELD_PACKET";
            }

            break;
        }
        case efmrpPacketDef::QUERY_PACKET: {
            trace()<<"[info] QUERY_PACKET received";
            efmrpQueryPacket *query_pkt=dynamic_cast<efmrpQueryPacket *>(efmrp_pkt);
            sendQueryAck(query_pkt->getOrigin(),query_pkt->getSource(), checkPath(query_pkt->getOrigin()));
            break;
        }
        case efmrpPacketDef::QUERY_ACK_PACKET: {
            trace()<<"[info] QUERY_ACK_PACKET received";
            efmrpQueryAckPacket *query_ack_pkt=dynamic_cast<efmrpQueryAckPacket *>(efmrp_pkt);
            updateFieldTableWithQA(query_ack_pkt);
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

