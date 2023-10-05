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
    trace()<<"[info] Startup initialized.";
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
    fp.env_c =  par("t_env_c");

    fp.alpha =  par("p_alpha");
    fp.beta  =  par("p_beta");
    fp.pnum  =  par("p_pnum");
    fp.gamma =  par("p_gamma");
    fp.n_lim =  par("p_n_lim");

    ff_app = dynamic_cast<ForestFire *>(appModule);

    env_val=1.0;
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
        case efmrpStateDef::INIT: {
            return "LEARN";
        }
    }
    return "UNKNOWN";
}

string efmrp::pathStatusToStr(efmrpPathStatus pstatus) const {
    switch (pstatus) {
       case efmrpPathStatus::UNKNOWN: {
           return "UNKNOWN";
       }
       case efmrpPathStatus::AVAILABLE: {
           return "AVAILABLE";
       }
       case efmrpPathStatus::USED: {
           return "USED";
       }
       case efmrpPathStatus::DEAD: {
           return "DEAD";
       }
       case efmrpPathStatus::UNDER_QUERY: {
           return "UNDER_QUERY";
       }
    }
    return "NONDEF";
} 

efmrpStateDef efmrp::getState() const {
    return g_state;
}

void efmrp::sendHello() {
    trace()<<"[info] Entering sendHello()";
    sendHello(0,1,1,getClock().dbl());
}

void efmrp::sendHello(int hop, double env, double nrg, double timestamp) {
    trace()<<"[info] Entering sendHello(hop="<<hop<<", env="<<env<<"timestamp="<<timestamp<<")";
    auto *hello_pkt=new efmrpHelloPacket("EFMRP HELLO packet", NETWORK_LAYER_PACKET);
    hello_pkt->setByteLength(netDataFrameOverhead);
    hello_pkt->setEfmrpPacketKind(efmrpPacketDef::HELLO_PACKET);
    hello_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    hello_pkt->setSource(SELF_NETWORK_ADDRESS);
    hello_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    hello_pkt->setHop(hop);
    hello_pkt->setEnv(env);
    hello_pkt->setNrg(nrg);
    hello_pkt->setTimestamp(timestamp);

    toMacLayer(hello_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::sendField(int hop, double nrg, double env, double trg) {
    trace()<<"[info] Entering sendField(hop ="<<hop<<", nrg="<<nrg<<", env="<<env<<", trg="<<trg<<")";
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
    ne.env=hello_pkt->getEnv();
    ne.nrg=hello_pkt->getNrg();
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" hop: "<<ne.hop<<" env: "<<ne.env<<" nrg: "<<ne.nrg<<" to hello_table";
    if(hello_table.find(ne.nw_address) != hello_table.end()) {
        trace()<<"[warn] Overriding record's hop: "<<hello_table[ne.nw_address].hop<<" to hop "<<ne.hop;
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
    ne.trg        = field_pkt->getTrg();

    if(field_table.find(ne.nw_address) != field_table.end()) {
        trace()<<"[info] Overriding existing record of node "<<ne.nw_address;
        field_table[ne.nw_address] = ne;
    } else {
        trace()<<"[info] New record of node "<<ne.nw_address;
        field_table.insert({ne.nw_address,ne});
    }
    trace()<<"[info] hop: "<<ne.hop<<" nrg: "<<ne.nrg<<"env: "<<ne.env;
}

void efmrp::updateFieldTableEntry(std::string ne, double env, double nrg, double trg) {
    trace()<<"[info] Entering updateFieldTableEntry(ne="<<ne<<", env="<<env<<", nrg="<<nrg<<", trg="<<trg<<")";
    if(field_table.find(ne) == field_table.end()) {
        throw std::string("[error] Entry not present");
    }
    field_table[ne].env=env;
    field_table[ne].nrg=nrg;
    field_table[ne].trg=trg;
}

bool efmrp::checkFieldEntry(std::string ne) {
    trace()<<"[info] checkFieldEntry(ne="<<ne<<")";
    return field_table.find(ne) == field_table.end() ? false : true;
}


void efmrp::initRouting() {
    trace()<<"[info] Entering initRouting()";
    trace()<<"[info] Clearing routing table, construct primary path";
    routing_table.clear();
    addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(1, {}),1);
    setTimer(efmrpTimerDef::ENV_CHK, fp.env_c+getRNG(0)->doubleRand());
    if(isSinkNextHop()) {
        trace()<<"[info] No secondary path needed, as sink is the neighbor";
        return;
    }
    try {
        addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(2, {}),2);
    } catch (std::string &s) {
        trace()<<"[error] "<<s;
    }
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

void efmrp::updateRoutingEntry(std::string nw_address, node_entry ne, int prio, efmrpPathStatus status) {
    trace()<<"[info] Entering updateRoutingEntry(nw_address="<<nw_address<<", node_entry.nw_address="<<ne.nw_address<<", prio="<<prio<<", status="<<status;
    for(auto &&re: routing_table) {
        if(re.nw_address == nw_address && re.prio==prio) {
            trace()<<"[info] record found";
            re.next_hop=ne.nw_address;
            re.status=status;
            re.target_value=targetFunction(ne);
            re.prio=prio;
            return;
        }
    }
    throw std::string("[error] No record found.");
}



bool efmrp::checkRoutingEntry(std::string ne, int prio) {
    trace()<<"[info] Entering checkRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.nw_address == ne && re.prio==prio && re.status==efmrpPathStatus::AVAILABLE) {
            trace()<<"[info] Entry exists";
            return true;
        }
    }
    return false;
}

routing_entry efmrp::getRoutingEntry(std::string ne, int prio) {
    trace()<<"[info] Entering getRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.nw_address == ne && re.prio==prio && re.status==efmrpPathStatus::AVAILABLE) {
            trace()<<"[info] Entry exists";
            return re;
        }
    }
    throw std::string("[error] Entry not found.");
}

void efmrp::removeRoutingEntry(std::string ne, int prio) {
    trace()<<"[info] Entering removeRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto it=routing_table.begin() ; it != routing_table.end();) {
        if(it->nw_address==ne && it->prio==prio) {
            trace()<<"[info] Record present in routing table, erasing.";
            it=routing_table.erase(it);
        } else {
            ++it;
        }
    }

}


double efmrp::calculateTargetValue() {
    trace()<<"[info] Entering calculateTargetValue()";
    node_entry min_ne;
    min_ne=hello_table.begin()->second;
    for(auto ne: hello_table) {
        if(ne.second.env < min_ne.env) {
            min_ne=ne.second;
        }
    }
    return (min_ne.env+ff_app->getEmergencyValue())/2;
}

node_entry efmrp::getNthTargetValueEntry(int order, std::vector<std::string> ne_lst) {
    trace()<<"[info] Entering getNthTargetValueEntry(order="<<order<<")";
    return getNthTargetValueEntry(order,ne_lst,false);
}

node_entry efmrp::getNthTargetValueEntry(int order, std::vector<std::string> ne_lst, bool use_pe) {
    trace()<<"[info] Entering getNthTargetValueEntry(order="<<order<<", use_pe="<<use_pe<<")";
    if(field_table.size() < order) {
        throw std::string("[error] Less record than order");
    }
    trace()<<"[info] Field table size: "<<field_table.size();

    std::vector<node_entry> fv;
    for(auto it = field_table.begin() ; it != field_table.end() ; ++it) {
        bool found=false;
        for(auto ne: ne_lst) {
            if(ne==it->second.nw_address) {
                trace()<<"[info] Record "<<it->second.nw_address<<" filtered out due to ne_lst.";
                found=true;
            }
        }
        if(use_pe) {
            for(auto pe: it->second.pe) {
                if(pe.status==efmrpPathStatus::DEAD && pe.origin==SELF_NETWORK_ADDRESS) {
                    trace()<<"[info] Record "<<it->second.nw_address<<" filtered out due to DEAD.";
                    found=true;
                }
            }
        }
        if(!found) {
            trace()<<"[info] Adding record "<<it->second.nw_address;
            fv.push_back(it->second);
        }
    }
    if(fv.size()<order) {
        throw std::string("[error] No record left");
    }

    std::sort(fv.begin(), fv.end(), [this](node_entry a, node_entry b) { return targetFunction(a) > targetFunction(b);  });
    
    return fv[order-1];
}

double efmrp::targetFunction(node_entry a) {
    trace()<<"[info] Entering targetFunction(a)";
    double ret_val = (1.0 - fp.alpha - fp.beta) * 1.0/(a.hop + 1) +
                     fp.alpha * a.trg + fp.beta * a.nrg;
    trace()<<"[info] targetFunction value for node "<<a.nw_address <<": "<<ret_val;
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
    trace()<<"[info] Number of available paths: "<<ret_val;
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
    trace()<<"[info] Entering sendQueryAck(origin="<<origin<<", dst="<<dst<<", used="<<used<<")";
    efmrpQueryAckPacket *query_ack_pkt=new efmrpQueryAckPacket("EFMRP QUERY_ACK packet", NETWORK_LAYER_PACKET);
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
        if(re.nw_address==ne && re.status==efmrpPathStatus::UNDER_QUERY && re.prio>1) {
            trace()<<"[info] Query started";
            return true;
        }
    }
    trace()<<"[info] Query not started";
    return false;
}

bool efmrp::queryCompleted(std::string ne) {
    trace()<<"[info] Entering queryCompleted(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==efmrpPathStatus::UNDER_QUERY && re.prio>1 && re.query_timestamp+fp.query < getClock().dbl()) {
            trace()<<"[info] Query completed";
            return true;
        }
    }
    trace()<<"[info] Query not completed";
    return false;
}


void efmrp::sendData(routing_entry re, cPacket *pkt) {
    trace()<<"[info] Entering sendData(re.next_hop="<<re.next_hop<<")";
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

void efmrp::forwardData(efmrpDataPacket *data_pkt) {
    trace()<<"[info] Entering forwardData()";
    routing_entry re;
    try {
        re=getPath(data_pkt->getOrigin(),data_pkt->getPri());
    } catch(std::string &e) {
        trace()<<"[error] "<<e;
        return;
    }
    trace()<<"[info] Next hop: "<<re.next_hop;
    data_pkt->setDestination(re.next_hop.c_str());
    data_pkt->setSource(SELF_NETWORK_ADDRESS);

    toMacLayer(data_pkt, resolveNetworkAddress(re.next_hop.c_str()));

}

void efmrp::sendRetreat(efmrpDataPacket *data_pkt) {
    trace()<<"[info] Entering sendRetreat()";
    efmrpRetreatPacket *retreat_pkt=new efmrpRetreatPacket("EFMRP RETREAT packet",NETWORK_LAYER_PACKET);

    retreat_pkt->setByteLength(netDataFrameOverhead);
    retreat_pkt->setEfmrpPacketKind(efmrpPacketDef::RETREAT_PACKET);
    retreat_pkt->setOrigin(data_pkt->getOrigin());
    retreat_pkt->setSource(SELF_NETWORK_ADDRESS);
    retreat_pkt->setDestination(data_pkt->getSource());

    retreat_pkt->setPri(data_pkt->getPri());

    toMacLayer(retreat_pkt, resolveNetworkAddress(data_pkt->getSource()));

}

void efmrp::sendAlarm(efmrpAlarmDef alarm_kind, double env_val, double nrg_val, double trg_val) {
    trace()<<"[info] Entering sendAlarm(alarm_kind="<<alarm_kind<<", env_val="<<env_val<<")";
    efmrpAlarmPacket *alarm_pkt=new efmrpAlarmPacket("EFMRP ALARM packet",NETWORK_LAYER_PACKET);

    alarm_pkt->setByteLength(netDataFrameOverhead);
    alarm_pkt->setEfmrpPacketKind(efmrpPacketDef::ALARM_PACKET);
    alarm_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    alarm_pkt->setSource(SELF_NETWORK_ADDRESS);
    alarm_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    alarm_pkt->setEfmrpAlarmKind(alarm_kind);

    if(alarm_kind==efmrpAlarmDef::ENVIRONMENT_ALARM) {
        alarm_pkt->setEnv(env_val);
        alarm_pkt->setNrg(nrg_val);
        alarm_pkt->setTrg(trg_val);
    } 

    toMacLayer(alarm_pkt, BROADCAST_MAC_ADDRESS);
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

bool efmrp::isSinkNextHop() {
    trace()<<"[info] Entering isSinkNextHop()";
    for(auto ne: routing_table) {
        if(ne.next_hop==getSinkAddress()) {
            trace()<<"[info] Sink is next hop.";
            return true;
        }
    }
    trace()<<"[info] Sink in routing table not present.";
    return false;
}

bool efmrp::checkNextHop(std::string ne, int prio) {
    trace()<<"[info] checkNextHop(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.next_hop==ne && re.nw_address==SELF_NETWORK_ADDRESS && re.prio==prio) {
            return true;
        }
    }
    return false;
}


routing_entry efmrp::getPath(std::string ne, int prio) {
    trace()<<"[info] Entering getPath(ne="<<ne<<", prio="<<prio<<")";
    for(auto entry: routing_table) {
        if(entry.nw_address==ne && entry.prio==prio && entry.status==efmrpPathStatus::AVAILABLE) {
            return entry;
        }
    }
    throw std::string("[error] No path found");
}

void efmrp::updateEntries(std::string ne, double env, double nrg, double trg) {
    trace()<<"[info] Entering updateEntries(ne="<<ne<<")";
    auto it=field_table.find(ne);
    if(it!=field_table.end()) {
        trace()<<"[info] Record present in field table, updating.";
        it->second.env=env;
        it->second.trg=trg;
    }
    for(auto &&e: routing_table) {
        if(e.next_hop==ne) {
            trace()<<"[info] Record present in routing table, updating.";
            e.target_value=targetFunction(it->second);
        }
    }
}

void efmrp::removeEntries(std::string ne) {
    trace()<<"[info] Entering removeEntries(ne="<<ne<<")";
    if(field_table.find(ne) != field_table.end()) {
        trace()<<"[info] Record present in field table, erasing.";
        field_table.erase(ne);
    }
    for(auto it=routing_table.begin() ; it != routing_table.end();) {
        if(it->next_hop==ne) {
            trace()<<"[info] Record present in routing table, erasing.";
            it=routing_table.erase(it);
        } else {
            ++it;
        }
    }
}

routing_entry efmrp::getPath(std::string ne) {
    trace()<<"[info] Entering getPath(ne="<<ne<<")";
    std::vector<routing_entry> rv;
    double tv_sum=0;
    double tv=0;
    double rnd=getRNG(0)->doubleRand();

    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==efmrpPathStatus::AVAILABLE) {
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
        tv+=re.target_value/tv_sum;
        if(rnd<tv) {
            trace()<<"[info] Selected entry: "<<re.next_hop;
            return re;
        }
    }

    trace()<<"[error] No route selected based on probability. Selecting first one next_hop: "<<rv[0].next_hop;
    return rv[0];
}

node_entry efmrp::findSecondaryPath(std::string ne, std::vector<std::string> ne_lst) {
    trace()<<"[info] Entering findSecondaryPath(ne="<<ne<<")";
    std::vector<node_entry> nv;
    for(auto entry: field_table) {
        for(auto pe: entry.second.pe) {
            if(pe.origin == ne && pe.status == efmrpPathStatus::AVAILABLE) {
                bool found=false;
                for(auto xe: ne_lst) {
                    if(xe==entry.second.nw_address) {
                        found=true;
                        trace()<<"[info] Exluded entry found: "<<xe;
                    }
                }
                if(!found) {
                    trace()<<"[info] Entry matches: "<<entry.second.nw_address;
                    nv.push_back(entry.second);
                }
            }
        }
    }
    if(0==nv.size()) {
        trace()<<"[error] No path candidate, giving up.";
        throw std::string("[error] No path candidate");
    }

    std::sort(nv.begin(), nv.end(), [this](node_entry a, node_entry b) { return targetFunction(a) > targetFunction(b);  });
    
    return nv[0];
}

void efmrp::updateFieldTableWithQA(efmrpQueryAckPacket *query_ack_pkt) {
    trace()<<"[info] Entering updateFieldTableWithQA(source="<<query_ack_pkt->getSource()<<", origin: "<<query_ack_pkt->getOrigin() <<", used: "<<query_ack_pkt->getUsed()<<")";
    efmrpPathStatus status = query_ack_pkt->getUsed() ? efmrpPathStatus::USED : efmrpPathStatus::AVAILABLE;
    if(field_table.find(query_ack_pkt->getSource()) != field_table.end()) {
        trace()<<"[info] Record exists";
        for(auto &&pe: field_table[query_ack_pkt->getSource()].pe) {
            if(pe.origin==std::string(query_ack_pkt->getOrigin())) {
                trace()<<"[info] QUERY_ACK record exists: "<<pe.origin<<" status: "<<pe.status;
                pe.status=status;
                return;
            }
        }
        field_table[query_ack_pkt->getSource()].pe.push_back({query_ack_pkt->getOrigin(),status });
    } else {
        trace()<<"[warn] Query responder does not exists in field_table";
    }
}

void efmrp::updateFieldTableWithPE(std::string ne, std::string pe, efmrpPathStatus status) {
    trace()<<"[info] Entering updateFieldTableWithPE(ne="<<ne<<", pe="<<pe<<", status="<<status<<")";
    if(field_table.find(ne) != field_table.end()) {
        trace()<<"[info] Record found";
        for(auto &&e: field_table[ne].pe) {
            if(e.origin==pe) {
                trace()<<"[info] Path entry found";
                e.status=status;
            }
        }
    } else {
        trace()<<"[error] Record not found, where is it gone?";
    }
}


void efmrp::timerFiredCallback(int index) {
    trace()<<"[info] Entering timerFiredCallback(index="<<index<<")";
    switch (index) {
        case efmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            setTimer(efmrpTimerDef::BUILD_START, fp.ttl + getRNG(0)->doubleRand());
            setState(efmrpStateDef::LEARN);
            break;
        }
        case efmrpTimerDef::BUILD_START: {
            trace()<<"[timer] BUILD_START expired"; 
            sendField(getHop(), 1.0, 1.0, 1.0);
            setState(efmrpStateDef::WORK);
            break;
        }
        case efmrpTimerDef::TTL: {
            trace()<<"[timer] TTL timer expired";
            setState(efmrpStateDef::BUILD);
            sendField(getHop(), ff_app->getEnergyValue(), ff_app->getEmergencyValue(), calculateTargetValue());
            setTimer(efmrpTimerDef::FIELD, fp.field + getRNG(0)->doubleRand());
            break;
        }
        case efmrpTimerDef::FIELD: {
            trace()<<"[timer] FIELD timer expired";
            setState(efmrpStateDef::WORK);
            initRouting();
            break;
        }
        case efmrpTimerDef::ENV_CHK: {
            trace()<<"[timer] ENV_CHK timer expired";
            double nrg_val=ff_app->getEnergyValue();
            double new_env_val=ff_app->getEmergencyValue();
            if(nrg_val<fp.n_lim) {
                trace()<<"[info] Energy below limit.";
                sendAlarm(efmrpAlarmDef::ENERGY_ALARM,0.0,0.0,0.0);
                break;
            }
            if(abs(new_env_val-env_val)>fp.gamma) {
                trace()<<"[info] Sensor reading difference exceeds gamma - new_env_val: "<<new_env_val<<" env_val: "<<env_val;
                sendAlarm(efmrpAlarmDef::ENVIRONMENT_ALARM,new_env_val,nrg_val, calculateTargetValue());
                env_val=new_env_val;
            }
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void efmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    trace()<<"[info] Entering fromApplicationLayer(..)";
    if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    }

    switch (getState()) {
        case efmrpStateDef::LEARN: {
            trace()<<"[error] In LEARN state, can't route packet";
            break;
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
            sendData(getPath(SELF_NETWORK_ADDRESS),pkt);
            break;
        }
    }
}


void efmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    trace()<<"[info] Entering fromMacLayer(..srcMacAddress="<<srcMacAddress<<")";
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
                sendHello(getHop(),ff_app->getEmergencyValue(), ff_app->getEnergyValue(), hello_pkt->getTimestamp());
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
            if(getState()==efmrpStateDef::INIT) {
                trace()<<"[info] Node in INIT state, doing nothing.";
            }

            if(getState()==efmrpStateDef::LEARN) {
                trace()<<"[info] Node in LEARN state, still updating Field table";
                updateFieldTable(field_pkt);
            }
            if(getState()==efmrpStateDef::BUILD) {
                trace()<<"[info] Node in BUILD state, updating Field table, if possible";
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
            efmrpDataPacket *data_pkt=dynamic_cast<efmrpDataPacket *>(efmrp_pkt);
            trace()<<"[info] origin: "<<data_pkt->getOrigin()<<" pri: "<<data_pkt->getPri();
            if(std::string(data_pkt->getOrigin()) == SELF_NETWORK_ADDRESS) {
                trace()<<"[error] Loop detected, origin equals destination";
                break;
            }

            if(isSink() && SELF_NETWORK_ADDRESS == std::string(data_pkt->getDestination())) {
                trace()<<"[info] Data packet arrived at sink";
                data_pkt->setSource(data_pkt->getOrigin());
                toApplicationLayer(decapsulatePacket(data_pkt));
                break;
            }

            auto pri=data_pkt->getPri();

            if(pri==1) {
                if(checkRoutingEntry(data_pkt->getOrigin(), pri)) {
                    forwardData(data_pkt->dup());
                } else {
                    addRoutingEntry(std::string(data_pkt->getOrigin()),getNthTargetValueEntry(1, {data_pkt->getSource()}),1);
                    forwardData(data_pkt->dup());
                }
                break;
            }
            
            if(pri==2) {
                trace()<<"[info] Secondary path";
                if(checkRoutingEntry(data_pkt->getOrigin(), pri)) {
                    trace()<<"[info] Secondary path exists";
                    forwardData(data_pkt->dup());
                } else {
                    trace()<<"[info] Secondary path not available, check status";
                    if(queryCompleted(data_pkt->getOrigin())) {
                        trace()<<"[info] Query completed";
                        node_entry ne;
                        try {
                            ne=findSecondaryPath(data_pkt->getOrigin(),{data_pkt->getSource() } );
                        } catch (std::string &e) {
                            trace()<<"[error] "<<e;
                            removeRoutingEntry(data_pkt->getOrigin(), data_pkt->getPri());
                            sendRetreat(data_pkt);
                            break;
                        }
                        updateRoutingEntry(data_pkt->getOrigin(),ne,pri,efmrpPathStatus::AVAILABLE);
                        forwardData(data_pkt->dup());
                    } else if(queryStarted(data_pkt->getOrigin())) {
                        trace()<<"[info] Query ongoing, dropping packet.";
                        break;
                    } else {
                        trace()<<"[info] No secondary path available.";
                        node_entry ne;
                        addRoutingEntry(data_pkt->getOrigin(), ne, pri, efmrpPathStatus::UNDER_QUERY,getClock().dbl());
                        sendQuery(data_pkt->getOrigin());
                        break;
                    }
                }
            }
            if(pri>2) {
                trace()<<"[info] Not implemented";
                break;
            }
            break;
        }
        case efmrpPacketDef::RETREAT_PACKET: {
            trace()<<"[info] RETREAT_PACKET received";
            efmrpRetreatPacket *retreat_pkt=dynamic_cast<efmrpRetreatPacket*>(efmrp_pkt);
            routing_entry re;
            try {
                re=getRoutingEntry(retreat_pkt->getOrigin(),retreat_pkt->getPri());
            } catch (std::string &e) {
                trace()<<"[error] "<<e;
                break;
            }
            if(re.next_hop == retreat_pkt->getSource()) {
                if(retreat_pkt->getOrigin()==SELF_NETWORK_ADDRESS) {
                    trace()<<"[info] Node is the origin";
                    node_entry ne;
                    updateFieldTableWithPE(retreat_pkt->getSource(), retreat_pkt->getOrigin(), efmrpPathStatus::DEAD);
                    removeRoutingEntry(SELF_NETWORK_ADDRESS, retreat_pkt->getPri());
                    try {
                        addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(2, {}, true),2);
                    } catch (std::string &e) {
                        trace()<<"[error] Giving up secondary path: "<<e;
                    }
                } else {
                    trace()<<"[info] Node is interim";
                    node_entry ne;
                    updateRoutingEntry(retreat_pkt->getOrigin(), ne, retreat_pkt->getPri(), efmrpPathStatus::UNDER_QUERY);
                    updateFieldTableWithPE(retreat_pkt->getSource(), retreat_pkt->getOrigin(), efmrpPathStatus::DEAD);
                }

            } else {
                trace()<<"[error] Retreat sender not next hop";
            }
            break;
        }
        case efmrpPacketDef::ALARM_PACKET: {
            trace()<<"[info] ALARM_PACKET received";
            efmrpAlarmPacket *alarm_pkt=dynamic_cast<efmrpAlarmPacket*>(efmrp_pkt);
            switch (alarm_pkt->getEfmrpAlarmKind()) {
                case efmrpAlarmDef::ENERGY_ALARM: {
                    trace()<<"[info] ENERGY_ALARM received, removing node";
                    if(checkNextHop(efmrp_pkt->getSource(),1)) {
                        trace()<<"[info] Node is primary path for this node";
                        removeEntries(alarm_pkt->getSource());
                        addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(1, {}),1);
                    } else if(checkNextHop(efmrp_pkt->getSource(),2)) {
                        trace()<<"[info] Node is secondary path for this node";
                        removeEntries(alarm_pkt->getSource());
                        try {
                            addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),getNthTargetValueEntry(2, {}),2);
                        } catch (std::string &s) {
                            trace()<<"[error] "<<s;
                        }
                    } else {
                        trace()<<"[info] Node is not primary or secondary at least for this node";
                        removeEntries(alarm_pkt->getSource());
                    }
                    break;
                }
                case efmrpAlarmDef::ENVIRONMENT_ALARM: {
                    trace()<<"[info] ENVIRONMENT_ALARM received, updating tables";
                    if(checkFieldEntry(alarm_pkt->getSource())) {
                        trace()<<"[info] Sender present in field table";
                        updateFieldTableEntry(alarm_pkt->getSource(),alarm_pkt->getEnv(), alarm_pkt->getNrg(), alarm_pkt->getTrg());
                        initRouting();
                    }
                    break;
                }
            }
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
    trace()<<"[info] Entering finishSpecific()";
    trace()<<"[info] Routing table";
    for(auto re: routing_table) {
        trace()<<"[info] Origin="<<re.nw_address<<", next hop="<<re.next_hop<<", status="<<pathStatusToStr(re.status)<<", prio="<<re.prio;

    }
    trace()<<"[info] Field table";
    for(auto ne: field_table) {
        trace()<<"[info] Addr="<<ne.second.nw_address;
       for(auto pe: ne.second.pe) {
           trace()<<"[info] Path entry - origin: "<<pe.origin<<" status: "<<pathStatusToStr(pe.status);
       }
    }
    if(isSink()) {
        generateYaml();
    }
}

void efmrp::generateYaml() {
    cTopology *topo;        // temp variable to access energy spent by other nodes
    topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    y_out<<YAML::BeginSeq;
    for(int i=0 ; i < topo->getNumNodes() ; ++i) {
        y_out<<YAML::BeginMap;
        auto pos=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"))->getLocation();
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i;
        y_out<<YAML::Key<<"x";
        y_out<<YAML::Value<<pos.x;
        y_out<<YAML::Key<<"y";
        y_out<<YAML::Value<<pos.y;
        auto *efmrp_instance = dynamic_cast<efmrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));

        serializeRoutingTable(efmrp_instance->getRoutingTable());
        y_out<<YAML::EndMap;

    }
    y_out<<YAML::EndSeq;
    
    ofstream loc_pdr_file("loc_pdr.yaml");
    loc_pdr_file<<y_out.c_str();
    loc_pdr_file.close();

    delete(topo);
}

void efmrp::serializeRoutingTable(std::vector<routing_entry> rt) {
    y_out<<YAML::Key<<"routing_table";
    y_out<<YAML::Value;
    y_out<<YAML::BeginSeq;
    for(auto re: rt) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"origin";
        y_out<<YAML::Value<<re.nw_address;
        y_out<<YAML::Key<<"next_hop";
        y_out<<YAML::Value<<re.next_hop;
        y_out<<YAML::Key<<"target_value";
        y_out<<YAML::Value<<re.target_value;
        y_out<<YAML::Key<<"status";
        y_out<<YAML::Value<<pathStatusToStr(re.status);
        y_out<<YAML::Key<<"prio";
        y_out<<YAML::Value<<re.prio;
        y_out<<YAML::EndMap;
    }

    y_out<<YAML::EndSeq;
}
