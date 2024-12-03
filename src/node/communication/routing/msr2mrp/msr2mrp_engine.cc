/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2024                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/msr2mrp/msr2mrp_engine.h"

msr2mrp_engine::msr2mrp_engine(msr2mrp *nw_layer, bool is_sink, bool is_master, std::string sink_addr, std::string self_addr, msr2mrp_feat_par fp, int netDataFrameOverhead, msr2mrpStateDef state) :
            nw_layer(nw_layer),
            g_sink_addr(sink_addr),
            selfAddress(self_addr),
            g_hop(std::numeric_limits<int>::max()),
            g_round(0),
            g_state(msr2mrpStateDef::UNDEF),
            g_is_sink(is_sink),
            g_is_master(is_master),
            netDataFrameOverhead(netDataFrameOverhead),
            currentSequenceNumber(0),
            fp(fp) {
                extTrace()<<"[info] MSR2MRP Engine starting with sink "<<getSinkAddress();
            if(isSink()) {
                extTrace()<<"[info] Node is a sink.";
                setHop(0);
                initPongTableSize();
                SetTimer(msr2mrpTimerDef::T_SINK_START,fp.t_start);
                setState(msr2mrpStateDef::WORK);
                if(fp.second_learn != msr2mrpSecLParDef::OFF) {
                    SetTimer(msr2mrpTimerDef::T_SEC_L_START,fp.t_sec_l_start);
                }
            } else {
                if(msr2mrpStateDef::LIMIT == state) {
                    setLimitedState(true);
                }
                setHop(std::numeric_limits<int>::max());
                setState(msr2mrpStateDef::INIT);
            }
            setRound(0);

            // this should be node level?
            forw_pkt_count=0;
    // move out
    //    updateTimer();
                  
        };

std::ostream & msr2mrp_engine::extTrace() { return nw_layer->extTrace(); };

void msr2mrp_engine::SetTimer(int index, simtime_t time) {
    extTrace()<<"[timer] SetTimer(index="<<index<<",time="<<time<<")";
    nw_layer->extSetTimer(stoi(getSinkAddress()),index,time);
}

simtime_t msr2mrp_engine::GetTimer(int index) {
    extTrace()<<"[timer] GetTimer(index="<<index<<")";
    return nw_layer->extGetTimer(stoi(getSinkAddress()),index);
}

void msr2mrp_engine::CancelTimer(int index) {
    extTrace()<<"[timer] CancelTimer(index="<<index<<")";
    nw_layer->extCancelTimer(stoi(getSinkAddress()),index);
}

cRNG* msr2mrp_engine::getRNG(int index) {
    extTrace()<<"[info] getRNG(index="<<index<<")";
    return nw_layer->extGetRNG(index);
}


int msr2mrp_engine::resolveNetworkAddress(const char *netAddr)
{
        if (!netAddr[0] || netAddr[0] < '0' || netAddr[0] > '9')
                return BROADCAST_MAC_ADDRESS;
        return atoi(netAddr);
}


cPacket* msr2mrp_engine::decapsulatePacket(cPacket * pkt)
{
        RoutingPacket *netPkt = check_and_cast <RoutingPacket*>(pkt);
        ApplicationPacket *appPkt = check_and_cast <ApplicationPacket*>(netPkt->decapsulate());

        appPkt->getAppNetInfoExchange().RSSI = netPkt->getNetMacInfoExchange().RSSI;
        appPkt->getAppNetInfoExchange().LQI = netPkt->getNetMacInfoExchange().LQI;
        appPkt->getAppNetInfoExchange().source = netPkt->getSource();
        return appPkt;
}


bool msr2mrp_engine::isSink() const {
    return g_is_sink;
}

bool msr2mrp_engine::isMaster() const {
    return g_is_master;
}

bool msr2mrp_engine::getSecL() {
    extTrace()<<"[info] Entering getSecL()";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(true==p.secl_performed) {
                return true;
            }
        }
    }
    return false;
}

bool msr2mrp_engine::getSecL(int pathid) {
    extTrace()<<"[info] Entering getSecL(pathid="<<pathid<<")";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid == pathid && p.secl_performed == true) {
                return true;
            }
        }
    }
    return false;
}


void msr2mrp_engine::setSecL(bool flag) {
    extTrace()<<"[info] Entering setSecL(flag="<<flag<<")";
    if(false==flag) {
        for(auto &&i: routing_table) {
            for(auto &&p: i.second.pathid) {
                p.secl_performed=false;
            }
        }
    } else {
        extTrace()<<"[info] set to true not implemented";
    }
}

void msr2mrp_engine::setSecL(int pathid, bool flag) {
    extTrace()<<"[info] Entering setSecL(pathid="<<pathid<<", flag="<<flag<<")";
    for(auto &&ne: routing_table) {
        for(auto &&p: ne.second.pathid) {
            if(p.pathid == pathid) {
                p.secl_performed = true;
            }
        }
    }
}

void msr2mrp_engine::pushSecLPathid(int pathid) {
    extTrace()<<"[info] Entering pushSecLPathid(pathid="<<pathid<<")";
    g_sec_l_pathid.push(pathid);
}

int  msr2mrp_engine::popSecLPathid() {
   extTrace()<<"[info] Entering popSecLPathid()";
   auto ret_val=g_sec_l_pathid.front();
   g_sec_l_pathid.pop();
   return ret_val;
}

int  msr2mrp_engine::getSecLPathid() {
    return g_sec_l_pathid.front();

}
bool msr2mrp_engine::isSecLPathidEmpty() {
    return g_sec_l_pathid.empty();
}


void msr2mrp_engine::setSinkAddress(std::string p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string msr2mrp_engine::getSinkAddress() const {
    return g_sink_addr;
}

std::string msr2mrp_engine::getSinkAddressFromRtTbl() {
    extTrace()<<"[info] Entering getSinkAddressFromRtTable()";
    if(isRoutingTableEmpty()) {
        throw routing_table_empty("[error] Routing table empty, cannot identify sink");
    }
    vector<int> origins;
    for(auto re: routing_table) {
        for(auto pe: re.second.pathid) {
            origins.push_back(pe.origin);
        }
    }
    extTrace()<<"[info] Sink addresses: "<<pathidToStr(origins);
    if(!std::all_of(origins.begin(), origins.end(), [&origins](int i) { return i == origins.front(); })) {
        throw state_not_permitted("[error] Sink entries ambigous");
    }
    return std::to_string(origins.front());
}

double msr2mrp_engine::getTl() {
    double t_l=fp.t_l;
    if(fp.random_t_l) {
        t_l=omnetpp::normal(getRNG(0),fp.t_l,fp.random_t_l_sigma);
    }
    extTrace()<<"[info] T_l timer's value: "<<t_l;
    return t_l;
}


double msr2mrp_engine::getTmeas() const {
    return fp.t_meas;
}

double msr2mrp_engine::getTest() const {
    return fp.t_est;
}

void msr2mrp_engine::handleTSecLTimer() {
    extTrace()<<"[info] Entering handleTSecLTimer()";
    if(fp.second_learn != msr2mrpSecLParDef::OFF) {
        extTrace()<<"[info] Second learn feature active";
        if(GetTimer(msr2mrpTimerDef::T_SEC_L) != -1) {
            extTrace()<<"[info] T_SEC_L timer active, restarting";
            CancelTimer(msr2mrpTimerDef::T_SEC_L);
            SetTimer(msr2mrpTimerDef::T_SEC_L,fp.t_sec_l);
        }
    }
}


msr2mrpCostFuncDef msr2mrp_engine::strToCostFunc(string str) const {
    if("hop" == str) {
        return msr2mrpCostFuncDef::HOP;
    } else if("hop_and_interf" == str) {
        return msr2mrpCostFuncDef::HOP_AND_INTERF;
    } else if("hop_emerg_and_interf" == str) {
        return msr2mrpCostFuncDef::HOP_EMERG_AND_INTERF;
    } else if("hop_and_pdr" == str) {
        return msr2mrpCostFuncDef::HOP_AND_PDR;
    } else if("hop_pdr_and_interf" == str) {
        return msr2mrpCostFuncDef::HOP_PDR_AND_INTERF;
    } else if("hop_emerg_pdr_and_interf" == str) {
        return msr2mrpCostFuncDef::HOP_EMERG_PDR_AND_INTERF;
    } else if("hop_enrgy_emerg_pdr_and_interf" == str) {
        return msr2mrpCostFuncDef::HOP_ENRGY_EMERG_PDR_AND_INTERF;
    } else if("hop_enrgy_emerg_and_pdr" == str) {
        return msr2mrpCostFuncDef::HOP_ENRGY_EMERG_AND_PDR;
    } else if("hop_enrgy_and_pdr" == str) {
        return msr2mrpCostFuncDef::HOP_ENRGY_AND_PDR;
    } else if("xpr_interf" == str) {
        return msr2mrpCostFuncDef::XPR_INTERF;
    } else if("xpr_hop_and_pdr" == str) {
        return msr2mrpCostFuncDef::XPR_HOP_AND_PDR;
    } else if("xpr_hop_pdr_and_interf" == str) {
        return msr2mrpCostFuncDef::XPR_HOP_PDR_AND_INTERF;
    } else if("sum_hop_enrgy_emerg_pdr_and_interf" == str) {
        return msr2mrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF;
    }
    throw std::invalid_argument("[error] Unkown cost function");
    return msr2mrpCostFuncDef::NOT_DEFINED; 
}

msr2mrpRinvTblAdminDef msr2mrp_engine::strToRinvTblAdmin(string str) const {
    if("erase_on_learn" == str) {
        return msr2mrpRinvTblAdminDef::ERASE_ON_LEARN;
    } else if("erase_on_round" == str) {
        return msr2mrpRinvTblAdminDef::ERASE_ON_ROUND;
    } else if(" never_erase" == str) {
        return msr2mrpRinvTblAdminDef::NEVER_ERASE;
    }
    throw std::invalid_argument("[error] Unkown RINV table admin");
    return msr2mrpRinvTblAdminDef::UNDEF_ADMIN;
}

msr2mrpSecLParDef msr2mrp_engine::strToSecLPar(std::string str) const {
    if("off" == str) {
        return msr2mrpSecLParDef::OFF;
    } else if("broadcast" == str) {
        return msr2mrpSecLParDef::BROADCAST;
    } else if("unicast" == str) {
        return msr2mrpSecLParDef::UNICAST;
    }
    throw std::invalid_argument("[error] Unknown second_learn parameter");
    return msr2mrpSecLParDef::OFF;
}


void msr2mrp_engine::setHop(int hop) {
    extTrace()<<"[info] Entering msr2mrp_engine::setHop(hop="<<hop<<")";  
    extTrace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int msr2mrp_engine::getHop() const {
    return g_hop;
}

int msr2mrp_engine::getHop(int pathid) {
    extTrace()<<"[info] Entering msr2mrp_engine::getHop(pathid="<<pathid<<")";
    int hop=getHop();
    bool fail=true;
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                hop = ne.second.hop;
                fail=false;
            }
        }
    }
    if(fail) {
        throw no_available_entry("No route available for the path"); 
    }
    extTrace()<<"[info] Hop count for pathid "<<pathid<<" is "<<hop;
    return hop;
}

int msr2mrp_engine::calculateHop(bool max_hop=false) {
    extTrace()<<"[info] Entering msr2mrp_engine::calculateHop(max_hop="<<max_hop<<")";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }

    if(std::all_of(rinv_table.begin(), rinv_table.end(),[](std::pair<std::string,msr2mrp_node_entry> ne){return ne.second.used; } )) {
        throw no_available_entry("[error] All RINV table entry used");
    }

    int hop=std::numeric_limits<int>::max();
    bool (*gt_lt)(int,int)=[](int lh, int rh) { return lh > rh; };

    if(max_hop) {
        hop=std::numeric_limits<int>::min();
        gt_lt=[](int lh, int rh) { return lh < rh; };
    }

    for(auto ne: rinv_table) {
        extTrace()<<"[info] Hop level for node "<<ne.second.nw_address<<" is "<<ne.second.hop;
        if(!ne.second.used && gt_lt(hop,ne.second.hop)) {
            hop=ne.second.hop;
        }
    }
    return hop+1;
}

int msr2mrp_engine::calculateHopFromRoutingTable() {
    extTrace()<<"[info] Entering msr2mrp_engine::calculateHopFromRoutingTable()";
    int hop=std::numeric_limits<int>::max();
    for(auto i: routing_table) {
        if(i.second.hop < hop) {
            hop=i.second.hop;
        }
    }
    return hop+1;
}

void msr2mrp_engine::setRound(int round) {
    extTrace()<<"[info] Changing round "<<g_round<<" to "<<round;
    g_round=round;
}

int msr2mrp_engine::getRound() const {
    return g_round;
}

void msr2mrp_engine::setState(msr2mrpStateDef state) {
    extTrace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string msr2mrp_engine::stateToStr(msr2mrpStateDef state) const {
    switch (state) {
        case msr2mrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case msr2mrpStateDef::WORK: {
            return "WORK";
        }
        case msr2mrpStateDef::INIT: {
            return "INIT";
        }
        case msr2mrpStateDef::LEARN: {
            return "LEARN";
        }
        case msr2mrpStateDef::ESTABLISH: {
            return "ESTABLISH";
        }
        case msr2mrpStateDef::MEASURE: {
            return "MEASURE";
        }
        case msr2mrpStateDef::LOCAL_LEARN: {
            return "LOCAL_LEARN";
        }
        case msr2mrpStateDef::DEAD: {
            return "DEAD";
        }
        case msr2mrpStateDef::S_ESTABLISH: {
            return "S_ESTABLISH";
        }
    }
    return "UNKNOWN";

}

msr2mrpStateDef msr2mrp_engine::getState() const {
    if(nw_layer->getState() == msr2mrpStateDef::DEAD) {
        return msr2mrpStateDef::DEAD;
    }
    return g_state;
}

void msr2mrp_engine::sendPing(int round) {
    extTrace()<<"[info] Entering msr2mrp_engine::sendPing(round = "<<round<<")";
    msr2mrpPingPacket *ping_pkt=new msr2mrpPingPacket("MSR2MRP PING packet", NETWORK_LAYER_PACKET);
    ping_pkt->setByteLength(netDataFrameOverhead);
    ping_pkt->setSink(getSinkAddress().c_str());
    ping_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::PING_PACKET);
    ping_pkt->setSource(SELF_NETWORK_ADDRESS);
    ping_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    ping_pkt->setRound(round);
    ping_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(ping_pkt, BROADCAST_MAC_ADDRESS);
}

void msr2mrp_engine::sendPong(int round) {
    extTrace()<<"[info] Entering msr2mrp_engine::sendPong(round = "<<round<<")";
    msr2mrpPongPacket *pong_pkt=new msr2mrpPongPacket("MSR2MRP PONG packet", NETWORK_LAYER_PACKET);
    pong_pkt->setByteLength(netDataFrameOverhead);
    pong_pkt->setSink(getSinkAddress().c_str());
    pong_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::PONG_PACKET);
    pong_pkt->setSource(SELF_NETWORK_ADDRESS);
    pong_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    pong_pkt->setRound(round);
    pong_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(pong_pkt, BROADCAST_MAC_ADDRESS);

}

void msr2mrp_engine::storePong(msr2mrpPongPacket *pong_pkt) {
    extTrace()<<"[info] Entering msr2mrp_engine::storePong(pong_pkt.source="<<pong_pkt->getSource()<<")";
    pong_table.insert({pong_pkt->getSource(),{pong_pkt->getSource(),std::vector<msr2mrp_pathid_entry>(1,{0,0}),0,false,0,0,false,pong_pkt->getRound()}});
}

void msr2mrp_engine::clearPongTable() {
    pong_table.clear();
}

void msr2mrp_engine::clearPongTable(int round) {
    extTrace()<<"[info] Entering msr2mrp_engine::clearPongTable(round="<<round<<")";
    if(pong_table.empty()) {
        return;
    }

    for(auto it = pong_table.begin() ; it != pong_table.end();) {
        if(it->second.round < round ) {
            it = pong_table.erase(it);
        } else {
            ++it;
        }
    }
}

int msr2mrp_engine::getPongTableSize() const {
    return pong_table.size();
}

void msr2mrp_engine::initPongTableSize() {
    extTrace()<<"Entering setPongTableSize()";
    msr2mrp_node_entry ne;
    ne.nw_address=SELF_NETWORK_ADDRESS;
//    pong_table.insert({std::string(SELF_NETWORK_ADDRESS),ne});
}


void msr2mrp_engine::sendRinv(int round, std::vector<msr2mrp_pathid_entry> pathid) {
    extTrace()<<"[info] Entering msr2mrp_engine::sendRinv(round = "<<round<<", pathid = "<<pathidToStr(pathid)<<")";
    msr2mrpRinvPacket *rinv_pkt=new msr2mrpRinvPacket("MSR2MRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setSink(getSinkAddress().c_str());
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setPathidArraySize(pathid.size());
    for(int i=0 ; i < pathid.size() ; ++i) {
        msr2mrpPathDef p_id;
        p_id.pathid = pathid[i].pathid;
        p_id.nmas   = pathid[i].nmas;
        p_id.enrgy  = pathid[i].enrgy;
        p_id.emerg  = pathid[i].emerg;
        p_id.pdr    = pathid[i].pdr;
        p_id.b_enrgy= pathid[i].b_enrgy;
        rinv_pkt->setPathid(i, p_id);
    }
    try {
        // This is wrong, hop should be moved to pathid entry, but whatever...
        rinv_pkt->setHop(getHop(pathid[0].pathid)+1);
    } catch (exception &e) {
        extTrace()<<"[info] "<<e.what()<<" fall back to normal hop";
        rinv_pkt->setHop(getHop());
    }
    rinv_pkt->setInterf(getPongTableSize());
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void msr2mrp_engine::sendRinv(int round, std::string origin) {
    extTrace()<<"[info] Entering msr2mrp_engine::sendRinv(round = "<<round<<")";
    std::vector<msr2mrp_pathid_entry> pathid;
    msr2mrp_pathid_entry pe;
    pe.pathid         = 0;
    pe.nmas           = 0;
    pe.secl           = 0;
    pe.secl_performed = 0;
    pe.used           = 0;
    pe.enrgy          = nw_layer->getEnergyValue(); 
    pe.emerg          = nw_layer->getEmergencyValue();
    pe.pdr            = 1.0;
    pe.origin         = atoi(origin.c_str()); 
    pathid.push_back(pe);
    sendRinv(round,pathid);
}

void msr2mrp_engine::sendRinvBasedOnHop() {
    extTrace()<<"[info] Entering sendRinvBasedOnHop()";
    if(getHop() < fp.ring_radius) {
        extTrace()<<"[info] Node inside mesh ring";
        sendRinv(getRound(),getSinkAddress());
    } else if(getHop() == fp.ring_radius) {
        extTrace()<<"[info] Node at mesh ring border";
        // With this the master/sensor node capabilities inside the ring won't matter
        sendRinv(getRound(), std::vector<msr2mrp_pathid_entry>(1,{resolveNetworkAddress(SELF_NETWORK_ADDRESS),static_cast<int>(isMaster()),false,false,false,  nw_layer->getEnergyValue(), nw_layer->getEmergencyValue(),1.0,atoi(getSinkAddress().c_str()), nw_layer->getEnergyValue()}) );
    } else {
        extTrace()<<"[info] Node outside mesh ring";
        std::vector<msr2mrp_pathid_entry> pathid;
        try {
            if(isMaster()) {
                // maybe it is implemented?
                //throw state_not_permitted("Master functionality not implemented");
                extTrace()<<"[info] Node is master node, selecting all pathids";
                extTrace()<<"[info] Master hack, let's consume 1 random number, to keep simulation in-sync: "<<getRNG(0)->intRand(pathid.size());
                for(auto ne: routing_table) {
                    for(auto p: ne.second.pathid) {
                        bool found=false;
                        for(auto pp: pathid) {
                            if(p.pathid==pp.pathid) {
                                found=true;
                            }
                        }
                        if(!found) {
                            msr2mrp_pathid_entry pe;
                            pe.pathid = p.pathid;
                            pe.nmas   = p.nmas+1;
                            pe.enrgy  = nw_layer->getEnergyValue(); // select min
                            pe.emerg  = nw_layer->getEmergencyValue(); // select min
                            pe.pdr    = static_cast<double>(rreq_table[ne.first].ack_count)/static_cast<double>(rreq_table[ne.first].pkt_count);
                            pe.origin = p.origin;
                            pe.b_enrgy = p.b_enrgy;
                            pathid.push_back(pe);
                        }
                    }
                }
            } else {
                extTrace()<<"[info] Node is sensor node, selecting random pathid";
                auto pe = selectPathid(false, true);
                pe.enrgy = nw_layer->getEnergyValue();
                pe.emerg = nw_layer->getEmergencyValue();

                pe.pdr    = 0;


                pathid.push_back(pe);
            }
        } catch (std::exception &e) {
            extTrace()<<e.what();
            throw e;
        }
        sendRinv(getRound(), pathid);
    }
}


void msr2mrp_engine::clearRinvTable() {
    extTrace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void msr2mrp_engine::addToRinvTable(msr2mrpRinvPacket *rinv_pkt) {
    extTrace()<<"[info] Add entry to RINV table - source: "<<rinv_pkt->getSource()<<" pathid size: "<<rinv_pkt->getPathidArraySize()<<" hop: "<<rinv_pkt->getHop()<<" interf: "<<rinv_pkt->getInterf();
    msr2mrp_node_entry ne;
    ne.nw_address = std::string(rinv_pkt->getSource());
    ne.pathid.clear(); 
    for(int i=0 ; i < rinv_pkt->getPathidArraySize() ; ++i) {
        msr2mrp_pathid_entry pe;
        pe.pathid         = rinv_pkt->getPathid(i).pathid;
        pe.nmas           = rinv_pkt->getPathid(i).nmas;
        pe.secl           = false;
        pe.secl_performed = false;
        pe.used           = false;
        pe.enrgy          = rinv_pkt->getPathid(i).enrgy;
        pe.emerg          = rinv_pkt->getPathid(i).emerg;
        pe.pdr            = rinv_pkt->getPathid(i).pdr;
        pe.b_enrgy        = rinv_pkt->getPathid(i).b_enrgy;
        extTrace()<<"[info] b_enrgy:"<<pe.b_enrgy;
        ne.pathid.push_back(pe);
    }
    extTrace()<<"[info] Pathid added: "<<pathidToStr(ne.pathid);
    ne.hop = rinv_pkt->getHop();
    ne.interf = rinv_pkt->getInterf();
    ne.nmas = rinv_pkt->getNmas();
    ne.used = false;
    if(rinv_table.find(ne.nw_address) != rinv_table.end()) {
        extTrace()<<"[info] Entry already exists, overriding";

        bool used = rinv_table.find(ne.nw_address)->second.used;
        ne.used=used;
        extTrace()<<"[info] Do not loose pathid used flags";
        for(auto &&new_pe: ne.pathid) {
            for(auto old_pe: rinv_table[ne.nw_address].pathid) {
                if(old_pe.pathid == new_pe.pathid) {
                    extTrace()<<"[info] pathid found: "<<old_pe.pathid<<" with flag: "<<old_pe.used;
                    new_pe.used=old_pe.used;
                }
            }
        }
        rinv_table[ne.nw_address]=ne;
    } else {
        extTrace()<<"[info] Adding new entry";
        rinv_table[ne.nw_address]=ne;
    }
}

int msr2mrp_engine::getRinvTableSize() const {
    return rinv_table.size();
}

void msr2mrp_engine::clearRinvTableLocalFlags() {
    extTrace()<<"[info] entering clearRinvTableLocalFlags()";
    for(auto i=rinv_table.begin() ; i != rinv_table.end() ; ++i) {
        i->second.local=false;
    }
}

void msr2mrp_engine::removeRinvEntry(std::string ne) {
    extTrace()<<"[info] Entering removeRinvEntry(ne="<<ne<<")";
    rinv_table.erase(ne);
}


void msr2mrp_engine::markRinvEntryLocal(std::string id) {
    extTrace()<<"[info] Entering markRinvEntryLocal("<<id<<")";
    if(rinv_table.find(id) == rinv_table.end()) {
        throw no_available_entry("[error] No entry available in RINV table");
    }
    rinv_table[id].local = true;
}

void msr2mrp_engine::markRinvEntryFail(std::string id) {
    extTrace()<<"[info] Entering markRinvEntryLocal(id="<<id<<")";
    if(!checkRinvEntry(id)) {
        throw no_available_entry("[error] No entry available in RINV table");
    }
    rinv_table[id].fail=true; 
}

void msr2mrp_engine::clearRreqTable() {
    extTrace()<<"[info] RREQ table erased";
    rreq_table.clear();
}

void msr2mrp_engine::saveRreqTable() {
    extTrace()<<"[info] Entering saveRreqTable()";
    backup_rreq_table=rreq_table;
    for(auto ne: rreq_table) {
        extTrace()<<"[info] ne: "<<ne.first<<" pathid: "<<pathidToStr(ne.second.pathid);
    }
}

void msr2mrp_engine::retrieveRreqTable() {
    extTrace()<<"[info] Entering retrieveRreqTable()";
    rreq_table=backup_rreq_table;
}

// Merge pathid array...
void msr2mrp_engine::retrieveAndMergeRreqTable() {
    extTrace()<<"[info] Entering retrieveAndMergeRreqTable()";
    for(auto ne: backup_rreq_table) {
        if(rreq_table.find(ne.first) != rreq_table.end()) {
            extTrace()<<"[warn] Duplicate record:"<<ne.first<<" merging pathid "<<pathidToStr(ne.second.pathid)<<" with "<<pathidToStr(rreq_table[ne.first].pathid);
            mergePathids(rreq_table[ne.first].pathid,ne.second.pathid);
        } else {
            extTrace()<<"[info] Merging element: "<<ne.second.nw_address<<" with pathid: "<<pathidToStr(ne.second.pathid)<<" to RREQ table";
            rreq_table[ne.first]=ne.second;
        }
    }
}

void msr2mrp_engine::mergePathids(std::vector<msr2mrp_pathid_entry> &p1, std::vector<msr2mrp_pathid_entry> &p2) {
    extTrace()<<"[info] Entering mergePathids(p1="<<pathidToStr(p1)<<", p2="<<pathidToStr(p2)<<")";
    for(auto pe: p1) {
        for(auto pe2: p2) {
            if(pe.pathid == pe2.pathid) {
                extTrace()<<"[error] Matching pathid entries: "<<pe.pathid;
                throw state_not_permitted("[error] Matching pathid entries");
            } 
        }
    }
    p1.insert( p1.end(), p2.begin(), p2.end() );
}

bool msr2mrp_engine::isRreqTableEmpty() const {
    return rreq_table.empty();
}

double msr2mrp_engine::calculateCostFunction(msr2mrp_node_entry ne) {
    double ret_val;

    // FIXME
    if(fp.e2e_cost) {
        ne.pathid[0].pdr;
    }

    switch (fp.cost_func) {
        case msr2mrpCostFuncDef::HOP: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) ;
            break;
        }
        case msr2mrpCostFuncDef::HOP_AND_PDR: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case msr2mrpCostFuncDef::HOP_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case msr2mrpCostFuncDef::HOP_PDR_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * log10(pow(ne.interf,fp.cost_func_iota)) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case msr2mrpCostFuncDef::HOP_EMERG_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(ne.emerg+1,fp.cost_func_epsilon) * log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case msr2mrpCostFuncDef::HOP_EMERG_PDR_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(ne.emerg+1,fp.cost_func_epsilon) * log10(pow(ne.interf,fp.cost_func_iota)) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case msr2mrpCostFuncDef::XPR_INTERF: {
            ret_val=log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case msr2mrpCostFuncDef::XPR_HOP_AND_PDR: {
            ret_val=fp.cost_func_phi * log10(static_cast<double>(ne.hop)) + fp.cost_func_pi * log10(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count));
            break;
        }
        case msr2mrpCostFuncDef::XPR_HOP_PDR_AND_INTERF: {
            ret_val=fp.cost_func_phi * log10(static_cast<double>(ne.hop)) + fp.cost_func_pi * log10(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count)) + fp.cost_func_iota * log10(ne.interf);
            break;
        }
        case msr2mrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF: {
            // CostFunction = 1 - ObjectiveFunction
            // CostFunction = 1 - [ (1-Pi-Epsilon-Iota-Eta-Mu)*1/(1+hop) + Pi*pdr + Epsilon*emerg + Iota*interf + Eta*enrgy + Mu*nmas/hop  ]
            extTrace()<<"[info] CF terms: hop="<<ne.hop<<", pdr="<<static_cast<double>(ne.ack_count)/static_cast<double>(ne.pkt_count)<<", emerg="<<ne.pathid[0].emerg<<", interf="<<ne.interf<<", enrgy="<<ne.pathid[0].enrgy<<", nmas="<<ne.pathid[0].nmas;
            extTrace()<<"[info] (1-Pi-Epsilon-Iota-Eta-Mu)="<<(1-fp.cost_func_pi-fp.cost_func_epsilon-fp.cost_func_iota-fp.cost_func_eta-fp.cost_func_mu)<<" Pi="<<fp.cost_func_pi<<", Epsilon="<<fp.cost_func_epsilon<<", Iota="<<fp.cost_func_iota<<", Eta="<<fp.cost_func_eta<<", Mu="<<fp.cost_func_mu;
            ret_val = 1 - ( 
                    (1-fp.cost_func_pi-fp.cost_func_epsilon-fp.cost_func_iota-fp.cost_func_eta-fp.cost_func_mu)*(1/(1+ne.hop))
                    + fp.cost_func_pi*static_cast<double>(ne.ack_count)/static_cast<double>(ne.pkt_count)
                    + fp.cost_func_epsilon*ne.pathid[0].emerg
                    + fp.cost_func_iota*ne.interf
                    + fp.cost_func_eta*ne.pathid[0].enrgy
                    + fp.cost_func_mu*static_cast<double>(ne.pathid[0].nmas)/static_cast<double>(ne.hop)
                    );

            break;
        }
        default: {
            extTrace()<<"[error] Unknown cost function: "<<fp.cost_func;
            throw unknown_cost_function("[error] Unknown cost function");
        }
    }
    if(ne.pathid.size() != 1) {
        extTrace()<<"[error] Ambigous pathid array";
        throw state_not_permitted("[error] pathid array size not 1");
    }
    if(msr2mrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF != fp.cost_func) {
        ret_val=ret_val/pow(static_cast<double>(ne.pathid[0].nmas+1), fp.cost_func_mu);
    }
    extTrace()<<"[info] Cost function for node entry "<<ne.nw_address<<": "<<ret_val;
    return ret_val;
}


void msr2mrp_engine::constructRreqTable() {
    extTrace()<<"[info] Entering msr2mrp_engine::constructRreqTable()";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }
    if(!rreq_table.empty()) {
        throw rreq_table_non_empty("[error] RREQ table not empty");
    }

    setHop(calculateHop(fp.calc_max_hop));

    if(getHop() <= fp.ring_radius) {
        extTrace()<<"[info] Node inside mesh ring";
        for(auto ne: rinv_table) {
            if(ne.second.hop < getHop() && !ne.second.used) {
                extTrace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                rreq_table.insert(ne);
                rinv_table[ne.second.nw_address].used=true;
            }
        }
    } else {
        extTrace()<<"[info] Node outside mesh ring";
        std::map<int, std::vector<msr2mrp_node_entry>> cl;
        std::for_each(rinv_table.begin(),rinv_table.end(),[&](std::pair<std::string,msr2mrp_node_entry> ne){
                if(ne.second.hop < getHop() && !ne.second.used) {
                    for(auto p: ne.second.pathid) {
                        if(cl.find(p.pathid) == cl.end()) {
                            extTrace()<<"[info] Creating entry for pathid: "<<p.pathid<<" and adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                            cl.insert({p.pathid,std::vector<msr2mrp_node_entry>{ne.second}});
                        } else {
                            extTrace()<<"[info] Adding entry for pathid: "<<p.pathid<<" with address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                            cl[p.pathid].push_back(ne.second);
                        }
                    }
                }
            });
        if(fp.cf_after_rresp) {
            extTrace()<<"[info] Selecting all RINV nodes to RREQ";
            for(auto l: cl) {
                for(auto n: l.second) {
                    if(n.hop < getHop()) {
                        rreq_table.insert({n.nw_address,n});
                        rinv_table[n.nw_address].used = true;
                        extTrace()<<"[info] Updating pathid entries";
                        for(auto it = rinv_table[n.nw_address].pathid.begin() ; it != rinv_table[n.nw_address].pathid.end() ; ++it) {
                            extTrace()<<"[info] "<<it->pathid;
                            it->used = true;
                        } 
                    }
                }
            }
        } else {
            for(auto l: cl) {
                extTrace()<<"[info] Selecting nodes per pathid to RREQ";
                msr2mrp_node_entry c_ne=l.second[0];
                extTrace()<<"[info] Candidate list entries for pathid "<<l.first<<" is " <<l.second.size();
                for(auto ne: l.second) {
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        extTrace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
                extTrace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<pathidToStr(c_ne.pathid);
                rreq_table.insert({c_ne.nw_address,c_ne});

                rinv_table[c_ne.nw_address].used=true;
            }
        }
    }
    if(rreq_table.empty()) {
        throw rreq_table_empty("[error] RREQ table empty after constructRreqTable()");
    }


    //            for(auto cl: candidate_list) {
    //                auto i=getRNG(0)->intRand(cl.second.size());
    //                extTrace()<<"[info] Selecting node "<<cl.second[i].nw_address <<" with pathid "<<cl.second[i].pathid<<" from "<<cl.second.size()<<" nodes";
    //                rreq_table.insert({cl.second[i].nw_address,cl.second[i]});
    //            }
}

void msr2mrp_engine::constructRreqTable(std::vector<int> path_filter) {
    extTrace()<<"[info] Entering constructRreqTable(path_filter="<<pathidToStr(path_filter)<<")";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }
    if(!rreq_table.empty()) {
        throw rreq_table_non_empty("[error] RREQ table not empty");
    }

    rreq_table=rinv_table;

    for(auto it=rreq_table.begin() ; it != rreq_table.end(); ) {
        bool erase=false;
        for(auto i: path_filter) {
            for(auto p_it=it->second.pathid.begin() ; p_it != it->second.pathid.end() ;) {
                if(i==p_it->pathid || p_it->used) {
                    // torol+leptet
                    extTrace()<<"[info] Erasing path: "<<p_it->pathid;
                    extTrace()<<"ne: "<<it->second.nw_address;
                    p_it=it->second.pathid.erase(p_it);
                } else {
                    p_it->secl=true;
                    ++p_it;
                }
            }
        }

        auto pathidUnavailable = [&](std::vector<msr2mrp_pathid_entry> pathid){ for(auto pe: pathid) { if(pe.used == false) { return false;  } } return true; };

        if(0==it->second.pathid.size() || ( it->second.used && pathidUnavailable(it->second.pathid))) {
            extTrace()<<"[info] Erasing node: "<<it->second.nw_address<<" pathid: "<<pathidToStr(it->second.pathid);
            rreq_table.erase(it++);
        } else {
            rinv_table[it->first].used=true;
            for(auto &&pe : rinv_table[it->first].pathid) {
                pe.used=true;
            }
            ++it;
        }
    }

    if(rreq_table.empty()) {
        throw rreq_table_empty("[error] RREQ table empty after construction");
    }
    for(auto ne: rreq_table) {
        extTrace()<<"[info] ne: "<<ne.first<<" pathid: "<<pathidToStr(ne.second.pathid);
    }

}


bool msr2mrp_engine::rreqEntryExists(const char *addr, int pathid) {
    if(rreq_table.find(string(addr)) != rreq_table.end()) {
        for(auto p: rreq_table[string(addr)].pathid) {
            if(p.pathid==pathid) {
                return true;
            }
        }
    }
    return false;
}

bool msr2mrp_engine::rreqEntryExists(std::string ne) {
    if(rreq_table.find(ne) != rreq_table.end()) {
        return true;
    }
    return false;
}

void msr2mrp_engine::updateRreqTableWithRresp(const char *addr, int pathid) {
    extTrace()<<"[info] Entering msr2mrp_engine::updateRreqTableWithRresp(addr="<<addr<<", pathid="<<pathid;
    if(rreqEntryExists(addr,pathid)) {
        extTrace()<<"[info] RRESP received flag set to true";
        rreq_table[string(addr)].rresp=true;
        rreq_table[string(addr)].ack_count++;
    } else {
        throw std::length_error("[error] Entry not found");
    }
}


int  msr2mrp_engine::getRreqPktCount() {
    if(rreq_table.empty()) {
        throw std::length_error("[error] RREQ table empty");
    }
    extTrace()<<"[info] Rreq Pkt count: "<<rreq_table.begin()->second.pkt_count;
    return rreq_table.begin()->second.pkt_count;
}


bool msr2mrp_engine::rrespReceived() const {
    if(std::any_of(rreq_table.begin(), rreq_table.end(),[](std::pair<std::string,msr2mrp_node_entry> ne){return ne.second.rresp; } )) {
        return true;
    }
    return false;
}


void msr2mrp_engine::updateRreqEntryWithEmergency(const char *addr, double emerg, double enrgy) {
    extTrace()<<"Entering msr2mrp_engine::updateRreqEntryWithEmergency(addr="<<addr<<", emerg="<<emerg<<", enrgy="<<enrgy<<")";
    if(rreq_table.find(string(addr)) == rreq_table.end()) {
        throw no_available_entry("[error] Entry not present in routing table"); 
    }
    // Most probably, this is unused
    rreq_table[string(addr)].emerg = emerg;
    extTrace()<<"[info] Pathid size: "<<rreq_table[string(addr)].pathid.size();
    for(auto &&pe: rreq_table[string(addr)].pathid) {
        extTrace()<<"[info] Updating path "<<pe.pathid;
        pe.emerg = emerg;
        pe.enrgy = enrgy;
    }
}


void msr2mrp_engine::removeRreqEntry(std::string ne, bool try_backup=false) {
    extTrace()<<"[info] Entering removeRreqEntry(ne="<<ne<<")";
    if(rreq_table.find(ne) != rreq_table.end()) {
        rreq_table.erase(ne);
        return;
    } else {
        if(try_backup) {
            if(backup_rreq_table.find(ne) != backup_rreq_table.end()) {
                backup_rreq_table.erase(ne);
                return;
            }
        }
    }
    throw no_available_entry("[error] Entry not available in RREQ table"); 
}

void msr2mrp_engine::sendRreqs(int count) {
    extTrace()<<"[info] Entering msr2mrp_engine::sendRreqs(count="<<count<<")";

    if(rreq_table.empty()) {
        throw std::length_error("[error] RREQ table empty");
    }
    for(auto &&ne: rreq_table) {
        for(int i = 0 ; i < count ; ++i) {
            msr2mrpRreqPacket* rreq_pkt=new msr2mrpRreqPacket("MSR2MRP RREQ packet",NETWORK_LAYER_PACKET);
            rreq_pkt->setSink(getSinkAddress().c_str());
            rreq_pkt->setByteLength(netDataFrameOverhead);
            rreq_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::RREQ_PACKET);
            rreq_pkt->setSource(SELF_NETWORK_ADDRESS);
            rreq_pkt->setDestination(ne.second.nw_address.c_str());
            rreq_pkt->setRound(getRound());
            rreq_pkt->setPathid(ne.second.pathid[0].pathid);
            rreq_pkt->setSequenceNumber(currentSequenceNumber++);
            extTrace()<<"[info] Sending RREQ to "<<ne.second.nw_address<<" with pathid: "<<ne.second.pathid[0].pathid;
            ne.second.pkt_count++;
            nw_layer->extToMacLayer(rreq_pkt, resolveNetworkAddress(ne.second.nw_address.c_str()));
        }
    }
}

void msr2mrp_engine::sendRreqs() {
    extTrace()<<"[info] Entering msr2mrp_engine::sendRreqs()";
    sendRreqs(1);
}

void msr2mrp_engine::sendRresp(const char *dest, int round, int pathid) {
    extTrace()<<"[info] Sending RRESP to "<<dest<<" with round "<<round<<" and pathid "<<pathid;
    msr2mrpRrespPacket* rresp_pkt=new msr2mrpRrespPacket("MSR2MRP RRESP packet",NETWORK_LAYER_PACKET);
    rresp_pkt->setByteLength(netDataFrameOverhead);
    rresp_pkt->setSink(getSinkAddress().c_str());
    rresp_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::RRESP_PACKET);
    rresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    rresp_pkt->setDestination(dest);
    rresp_pkt->setRound(round);
    rresp_pkt->setPathid(pathid);
    rresp_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(rresp_pkt, resolveNetworkAddress(dest));
}


void msr2mrp_engine::sendLreqBroadcast(int round, int pathid) {
    extTrace()<<"[info] Entering sendLReqBroadcast(round="<<round<<", pathid="<<pathid<<")";
    sendLreq(BROADCAST_NETWORK_ADDRESS,round,pathid);
}

void msr2mrp_engine::sendLreq(const char *nw_address, int round, int pathid) {
    extTrace()<<"[info] Entering sendLReq(nw_address="<<nw_address<<", round="<<round<<", pathid="<<pathid<<")";
    msr2mrpLreqPacket *lreq_pkt=new msr2mrpLreqPacket("MSR2MRP LREQ packet",NETWORK_LAYER_PACKET);
    lreq_pkt->setByteLength(netDataFrameOverhead);
    lreq_pkt->setSink(getSinkAddress().c_str());
    lreq_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::LREQ_PACKET);
    lreq_pkt->setSource(SELF_NETWORK_ADDRESS);
    lreq_pkt->setDestination(nw_address);
    lreq_pkt->setRound(round);
    lreq_pkt->setPathid(pathid);
    lreq_pkt->setHop(getHop());
    lreq_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(lreq_pkt, resolveNetworkAddress(nw_address));
}

void msr2mrp_engine::sendLreqUnicast(int round, int pathid) {
    extTrace()<<"[info] Entering sendLReqUnicast(round="<<round<<", pathid="<<pathid<<")";
    for(auto ne: recv_table) {
        // pathid 0 means we are inside the ring
        bool send=false;
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                send=true;
            }
        }
        if((send || pathid == 0) && !ne.second.secl) {
            sendLreq(ne.second.nw_address.c_str(), round, pathid);
        }
    }
}

void msr2mrp_engine::sendLresp(const char *nw_address, int round, int pathid) {
    extTrace()<<"[info] Entering sendLResp(nw_address="<<nw_address<<", round="<<round<<", pathid="<<pathid<<")";
    msr2mrpLrespPacket *lresp_pkt=new msr2mrpLrespPacket("MSR2MRP LRESP packet",NETWORK_LAYER_PACKET);
    lresp_pkt->setByteLength(netDataFrameOverhead);
    lresp_pkt->setSink(getSinkAddress().c_str());
    lresp_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::LRESP_PACKET);
    lresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    lresp_pkt->setDestination(nw_address);
    lresp_pkt->setRound(round);
    lresp_pkt->setPathid(pathid);
    lresp_pkt->setHop(getHop());
    lresp_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(lresp_pkt, resolveNetworkAddress(nw_address));
}

void msr2mrp_engine::sendData(cPacket *pkt, std::string dest, int pathid) {
    extTrace()<<"[info] Sending DATA to "<<dest<<" via pathid "<<pathid;
    msr2mrpDataPacket *data_pkt=new msr2mrpDataPacket("MSR2MRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setSink(getSinkAddress().c_str());
    data_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setHop(getHop(pathid));
    data_pkt->setReroute(0);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->encapsulate(pkt);
    nw_layer->extToMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void msr2mrp_engine::schedulePkt(cPacket *pkt, std::string dest, int pathid) {
    extTrace()<<"[info] Entering schedulePkt(pkt, dest="<<dest<<", pathid="<<pathid<<")";
    msr2mrpDataPacket *data_pkt=new msr2mrpDataPacket("MSR2MRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setSink(getSinkAddress().c_str());
    data_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->setRepeat(0);
    data_pkt->setReroute(0);
    data_pkt->encapsulate(pkt);
    pkt_list.push_back(data_pkt);
}

void msr2mrp_engine::forwardData(msr2mrpDataPacket *data_pkt, std::string dest, bool reroute=false) {
    extTrace()<<"[info] Entering forwardData(dest="<<dest<<")";
    forwardData(data_pkt, dest, data_pkt->getPathid(),reroute);
}

void msr2mrp_engine::forwardData(msr2mrpDataPacket *data_pkt, std::string dest, int pathid, bool reroute) {
    extTrace()<<"[info] Entering forwardData(dest="<<dest<<", pathid="<<pathid<<", reroute="<<reroute<<")";
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    if(reroute) {
        data_pkt->setReroute(data_pkt->getReroute()+1);
    }
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
    ++forw_pkt_count;
}

void msr2mrp_engine::clearRoutingTable() {
    extTrace()<<"[info] Routing table erased";
    routing_table.clear();
}

void msr2mrp_engine::addRoute(std::string next_hop, int pathid) {
    extTrace()<<"[info] Entering msr2mrp_engine::addRoute(next_hop="<<next_hop<<", pathid="<<pathid<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        routing_table[next_hop]={next_hop,std::vector<msr2mrp_pathid_entry>(1,{pathid,0}) };
    } else {
        extTrace()<<"[error] Route already exists";
    }
}

bool msr2mrp_engine::isRoutingTableEmpty() const {
    return routing_table.empty();
}

void msr2mrp_engine::constructRoutingTableFromRinvTable() {
    extTrace()<<"[info] Entering constructRoutingTableFromRinvTable()";
    std::map<std::string,msr2mrp_node_entry> filt_table;
    for(auto ne: rinv_table) {
        if(ne.second.local) {
            filt_table[ne.second.nw_address]=ne.second;
        }
    }

    //    for(auto 
}

void msr2mrp_engine::removeRoute(std::string ne) {
    extTrace()<<"[info] Entering removeRoute(ne="<<ne<<")";
    routing_table.erase(ne);
}


void msr2mrp_engine::constructRoutingTable(bool rresp_req) {
    extTrace()<<"[info] Entering msr2mrp_engine::constructRoutingTable(rresp_req="<<rresp_req<<")";
    for(auto ne: rreq_table) {
        if(ne.second.rresp || !rresp_req) {
            extTrace()<<"[info] Adding node "<<ne.second.nw_address<<" with pathid "<<pathidToStr(ne.second.pathid);
            routing_table.insert(ne);
        }
    }
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
}

void msr2mrp_engine::constructRoutingTable(bool rresp_req, bool app_cf, double pdr=0.0, bool update=false) {
    extTrace()<<"[info] Entering msr2mrp_engine::constructRoutingTable(rresp_req="<<rresp_req<<", app_cf="<<app_cf<<", pdr="<<pdr<<")";
    if(!app_cf) {
        constructRoutingTable(rresp_req);
        return;
    }

    auto calc_pdr=[&](msr2mrp_node_entry n){ extTrace()<<"[info] PDR: "<<static_cast<double>(n.ack_count)/static_cast<double>(n.pkt_count);return static_cast<double>(n.ack_count)/static_cast<double>(n.pkt_count);};
    auto clean_pathid=[](msr2mrp_node_entry n, int p){ for(int i=0 ; i < n.pathid.size() ; ++i) { if(n.pathid[i].pathid == p) { auto ret=n; ret.pathid=std::vector<msr2mrp_pathid_entry>(1,n.pathid[i]); return ret; } } };

    if(getHop() <= fp.ring_radius) {
        if(update) {
            extTrace()<<"[error] No routing table update inside the ring";
            throw state_not_permitted("[error] No routing table update inside the ring");
            /* but why? */ 
        }
        extTrace()<<"[info] Node inside mesh ring";
        for(auto ne: rreq_table) {
            if(ne.second.hop < getHop() && (ne.second.rresp || !fp.rresp_req ) && calc_pdr(ne.second) >= pdr ) {
                extTrace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                routing_table.insert(ne);
                //                rinv_table[ne.second.nw_address].used=true;
            }
        }
        if(isRoutingTableEmpty()) {
            throw routing_table_empty("[error] routing table empty after constructRoutingTable()");
        }
        return;
    }

    std::map<int, std::vector<msr2mrp_node_entry>> cl;
    std::for_each(rreq_table.begin(),rreq_table.end(),[&](std::pair<std::string,msr2mrp_node_entry> ne){
            // Either select only hop-based, if rresp is not required or based on rresp received
            // During update do not consider hop
            if((update || ne.second.hop < getHop()) && (ne.second.rresp || !fp.rresp_req) && calc_pdr(ne.second) >= pdr ) {
                for(auto p: ne.second.pathid) {
                    if(cl.find(p.pathid) == cl.end()) {
                        extTrace()<<"[info] Creating pathid entry "<<p.pathid<<" and adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                        cl.insert({p.pathid,std::vector<msr2mrp_node_entry>{clean_pathid(ne.second,p.pathid)}});
                    } else {
                        extTrace()<<"[info] Adding to pathid entry "<<p.pathid<<" entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                        cl[p.pathid].push_back(clean_pathid(ne.second,p.pathid));
                    }
                }
            }
       });



    for(auto l: cl) {
        extTrace()<<"[info] Selecting nodes per pathid to RREQ";
        msr2mrp_node_entry c_ne=l.second[0];

        extTrace()<<"[info] Candidate list entries for pathid "<<l.first<<" is " <<l.second.size();
        if(fp.drop_1st_rt_c && getRNG(0)->doubleRand() > 1.0-fp.drop_prob) {
            extTrace()<<"[info] Drop 1st routing table candidate active";
            if(l.second.size()>1) {
                for(auto ne: l.second) {
                    extTrace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        extTrace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
                auto it=l.second.begin();
                while(it->nw_address != c_ne.nw_address) {
                    ++it;
                }
                l.second.erase(it);
                c_ne=l.second[0];
                for(auto ne: l.second) {
                    extTrace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        extTrace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
            }
        } else {
            for(auto ne: l.second) {
                extTrace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                    extTrace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                    c_ne=ne;
                }
            }
        }
        extTrace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<pathidToStr(c_ne.pathid);
        c_ne.pkt_count = 0;
        c_ne.ack_count = 0;
       
        // quite dangerous 
        auto p=c_ne.pathid[0];
        p.secl=update;
        c_ne.pathid.clear();
        for(auto &&ne: routing_table) {
            for(auto it=ne.second.pathid.begin() ; it !=ne.second.pathid.end();) {
                if(it->pathid==p.pathid) {
                    extTrace()<<"Duplicate pathid found, erasing";
                    it=ne.second.pathid.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if(routing_table.find(c_ne.nw_address) == routing_table.end()) {
            routing_table.insert({c_ne.nw_address,c_ne});
            routing_table[c_ne.nw_address].secl=update; // Set secl parameter to true, if second learn
            rinv_table[c_ne.nw_address].used=true;
            routing_table[c_ne.nw_address].pathid.push_back(p);
        } else {
            routing_table[c_ne.nw_address].pathid.push_back(p);
        }
    }
    if(routing_table.empty()) {
        throw routing_table_empty("[error] routing table empty after constructRoutingTable()");
    }
}

msr2mrp_pathid_entry msr2mrp_engine::selectPathid(bool replay, bool rinv_gen) {
    extTrace()<<"[info] Entering msr2mrp_engine::selectPathid(replay="<<replay<<")";
    if(!replay) {
        g_pathid=selectPathid(rinv_gen);
    }
    return g_pathid;
}


std::vector<int> msr2mrp_engine::selectAllPathid() {
    return selectAllPathid(fp.path_sel);
}

std::vector<int> msr2mrp_engine::selectAllPathid(int path_sel) {
    extTrace()<<"[info] Entering selectAllPathid()";
     if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }

    auto tmp_table = routing_table;

    if(0 != path_sel) {
        for(auto ne: routing_table) {
            // Prefer primary
            if(1 == path_sel && ne.second.secl) {
                tmp_table.erase(ne.first);
            }
            // Prefer secl
            if(2 == path_sel && !ne.second.secl) {
                tmp_table.erase(ne.first);
            }
        }
    }

    if(tmp_table.empty()) {
        extTrace()<<"[info] Cannot apply path selection policy"; 
        tmp_table = routing_table;
    }

    std::vector<int> pathid;
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            pathid.push_back(p.pathid);
        }
    }
    extTrace()<<"[info] Selected pathids: "<<pathidToStr(pathid);
    return pathid;
}

msr2mrp_pathid_entry msr2mrp_engine::selectPathid(bool rinv_gen) {
    extTrace()<<"[info] Entering msr2mrp_engine::selectPathid()";
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }

    auto tmp_table = routing_table;

    if(0 != fp.path_sel) {
        for(auto ne: routing_table) {
            // Prefer primary
            if(1 == fp.path_sel && ne.second.secl) {
                tmp_table.erase(ne.first);
            }
            // Prefer secl
            if(2 == fp.path_sel && !ne.second.secl) {
                tmp_table.erase(ne.first);
            }
        }
    }

    if(tmp_table.empty()) {
        extTrace()<<"[info] Cannot apply path selection policy"; 
        tmp_table = routing_table;
    }

    std::vector<msr2mrp_pathid_entry> pathid;
    for(auto ne: tmp_table) {
        for(auto p: ne.second.pathid) {
            extTrace()<<"[info] Pathid collected: "<<p.pathid;
            pathid.push_back(p);
        }
    }

    if(!rinv_gen) {
        extTrace()<<"[info] Performing random even pathid selection, no RINV generation";
        auto i=getRNG(0)->intRand(pathid.size());
        extTrace()<<"[info] Random number: "<<i;
        auto it=pathid.begin();
        std::advance(it,i);
        extTrace()<<"[info] Selected pathid: "<<it->pathid;
        return *it;
    }

    msr2mrp_pathid_entry ret_val;
    extTrace()<<"[info] Getting pathid for RINV";
    switch (fp.rinv_pathid) {
        case msr2mrpRinvPathidDef::EVEN: {
            extTrace()<<"[info] Performing random even pathid selection";
            auto i=getRNG(0)->intRand(pathid.size());
            extTrace()<<"[info] Random number: "<<i;
            auto it=pathid.begin();
            std::advance(it,i);
            extTrace()<<"[info] Selected pathid: "<<it->pathid;
            ret_val = *it;
            break;
        }
        case msr2mrpRinvPathidDef::INV_PROB: {
            extTrace()<<"[info] Performing inverse probability pathid selection";
            auto p_w_num = getPathIdWithNum();
            std::map<int,double> prob_map;
            double sum_prob = 0.0;
            double agg_prob = 0.0;
            for(auto i: p_w_num) {
                sum_prob += 1.0/static_cast<double>(i.second);
                extTrace()<<"Sum probability: "<<sum_prob;
            }

            for(auto p: p_w_num) {
                double prob = 1.0/static_cast<double>(p.second);
                agg_prob += prob;
                extTrace()<<"Probability for "<<p.first<<" pathid is: "<<prob<<" aggregated probability is: "<<agg_prob;
                prob_map[p.first] = agg_prob/sum_prob;
            }
            auto rnd_num=getRNG(0)->doubleRand();
            extTrace()<<"Random: "<<rnd_num;
            int pathid = 0;
            for(auto p: prob_map) {
                if(p.second > rnd_num) {
                    extTrace()<<"Pathid selected: "<<p.first<<" with agg prob: "<<p.second;
                    pathid = p.first;
                    break;
                }
            }
            for(auto r: routing_table) {
                for(auto p: r.second.pathid) {
                    if(p.pathid == pathid) {
                        ret_val = p;
                        break;
                    }
                }
            }
        }
        case msr2mrpRinvPathidDef::MIN_COUNT: {
            extTrace()<<"[info] Performing minimum count pathid selection";
            auto p_w_num = getPathIdWithNum();
            std::pair<int,int> min=*(p_w_num.begin());
            for(auto i: p_w_num) {
                if(i.second < min.second) {
                    min = i;
                }
            }
            extTrace()<<"[info] Selected pathid: "<<min.first<<" with count: "<<min.second;
            auto pathid = min.first;
            for(auto r: routing_table) {
                for(auto p: r.second.pathid) {
                    if(p.pathid == pathid) {
                        ret_val = p;
                        break;
                    }
                }
            }
        }

    }
    return ret_val;
}

std::map<int,int> msr2mrp_engine::getPathIdWithNum() {
    extTrace()<<"Entering getPathIdWithNum()";
    std::map<int,int> pathid_num;
    for(auto i: rinv_table) {
        for(auto j: routing_table) {
            for(auto k: j.second.pathid) {
                for(auto l: i.second.pathid) {
                    if(k.pathid==l.pathid) {
                        extTrace()<<"Pathid "<<k.pathid<<" found.";
                        if(pathid_num.find(k.pathid) == pathid_num.end()) {
                            pathid_num[k.pathid]=1;
                        } else {
                            pathid_num[k.pathid]++;
                        }
                    }
                }
            }
        }
    }
    return pathid_num;
} 

std::string msr2mrp_engine::pathidToStr(vector<msr2mrp_pathid_entry> pathid) {
    std::string str;
    for(auto i: pathid) {
        str.append(std::to_string(i.pathid));
        str.append(" ");
    }
    return str;
}

std::string msr2mrp_engine::pathidToStr(vector<int> pathid) {
    std::string str;
    for(auto i: pathid) {
        str.append(std::to_string(i));
        str.append(" ");
    }
    return str;
}

std::string msr2mrp_engine::getNextHop(int pathid) {
    extTrace()<<"[info] Entering getNextHop(pathid="<<pathid<<")";
    msr2mrp_node_entry next_hop;
    bool found=false;
    std::for_each(routing_table.begin(),routing_table.end(),
            [&](std::pair<std::string,msr2mrp_node_entry> ne){
            for(auto p: ne.second.pathid) {
            if(p.pathid== pathid) {
            found=true;
            next_hop=ne.second;
            }
            }
            });
    if(!found) {
        throw no_available_entry("Next hop not available");
    }
    extTrace()<<"[info] Next hop: "<<next_hop.nw_address;
    return next_hop.nw_address;
}

std::string msr2mrp_engine::getNextHop(int pathid, bool random_node) {
    extTrace()<<"[info] Entering getNextHop(pathid="<<pathid<<", random_node="<<random_node<<")";
    std::string next_hop;
    if(random_node) {
        std::vector<msr2mrp_node_entry> nodes;
        for(auto a : routing_table) {
            for(auto p: a.second.pathid) {
                if(p.pathid==pathid) {
                    nodes.push_back(a.second);
                }
            }
        }
        if(nodes.empty()) {
            throw no_available_entry("[error] Next hop not available");
        }
        auto i=getRNG(0)->intRand(nodes.size());
        next_hop=nodes[i].nw_address;
        extTrace()<<"[info] Randomly selected node: "<<next_hop;    
    } else {
        next_hop=getNextHop(pathid);
    }
    return next_hop;
}

void msr2mrp_engine::incPktCountInRoutingTable(std::string next_hop) {
    extTrace()<<"[info] Entering incPktCountInRoutingTable(next_hop="<<next_hop<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        throw no_available_entry("[error] No available entry to update Pkt count"); 
    }

    routing_table[next_hop].pkt_count++;
}

void msr2mrp_engine::incOrigPktCountInRoutingTable(std::string next_hop) {
    extTrace()<<"[info] Entering incOrigPktCountInRoutingTable(next_hop="<<next_hop<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        throw no_available_entry("[error] No available entry to update Pkt count"); 
    }

    routing_table[next_hop].orig_pkt_count++;
}


bool msr2mrp_engine::checkPathid(int pathid) {
    extTrace()<<"[info] Entering checkPathid(pathid="<<pathid<<")";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                extTrace()<<"[info] Pathid found";
                return true;
            }
        }
    }
    extTrace()<<"[info] Pathid not found";
    return false;
}

bool msr2mrp_engine::checkRoute(std::string ne) {
    extTrace()<<"Entering checkRoute(ne="<<ne<<")";
    if(routing_table.find(ne)!=routing_table.end()) {
        return true;
    }
    return false;
}


int msr2mrp_engine::calculateRepeat(const char *dest) {
    extTrace()<<"[info] Entering msr2mrp_engine::calculateRepeat(dest="<<dest<<")";

    auto entry_it = routing_table.find(std::string(dest));

    if(entry_it == routing_table.end()) {
        throw no_available_entry("[error] Entry not available in routing table");
    } 

    double hop = static_cast<double>(entry_it->second.hop) + 1;
    double rep = 0;
    if(fp.rep_m_pdr) {
        double m_pdr=static_cast<double>(entry_it->second.ack_count)/static_cast<double>(entry_it->second.pkt_count);
        rep = ceil( log(1.0 - fp.e2e_qos_pdr) / log(1.0 - pow(m_pdr,hop) ));
    } else {
        rep = ceil( log(1.0 - fp.e2e_qos_pdr) / log(1.0 - pow(fp.qos_pdr,hop) ));
    }

    extTrace()<<"[info] Repeat for destination "<<dest<<" with hop "<<hop<<" is "<<rep;

    return static_cast<int>(rep);

    //    return rep;
}


void msr2mrp_engine::incPktCountInRecvTable(std::string entry, int pathid, int round) {
    extTrace()<<"[info] Entering incPktCountInRecvTable("<<entry<<")";
    if(recv_table.find(entry) == recv_table.end()) {
        recv_table[entry] = {entry,std::vector<msr2mrp_pathid_entry>(1,{pathid,0}),0,false,0,0,false,round,1};
    } else {
        bool create=true;
        for(auto p: recv_table[entry].pathid) {
            if(p.pathid==pathid) {
                create=false;
            }
        }
        if(create) {
            recv_table[entry].pathid.push_back({pathid,0});
        }
        recv_table[entry].pkt_count++;
    }
}

void msr2mrp_engine::updateRinvTableFromRreqTable() {
    extTrace()<<"[info] Entering updateRinvTableFromRreqTable()";
    for(auto ne: rreq_table) {
        if(rinv_table.find(ne.first) != rinv_table.end()) {
            extTrace()<<"[info] Updating node "<<ne.first<<" as used in RINV table";
            rinv_table[ne.first].used = true;
        }
    }
}

void msr2mrp_engine::timerFiredCallback(int index) {
    switch (index) {
        case msr2mrpTimerDef::T_SINK_START: {
            extTrace()<<"[timer] T_SINK_START timer expired";
            setRound(1+getRound());
            sendRinv(getRound(),SELF_NETWORK_ADDRESS);
            SetTimer(msr2mrpTimerDef::T_REPEAT,fp.t_start*10.0);
            if(fp.periodic_restart) {
                SetTimer(msr2mrpTimerDef::T_RESTART,fp.t_restart);
            }
            break;
        }
        case msr2mrpTimerDef::T_RESTART: {
            extTrace()<<"[timer] T_RESTART timer expired";
            setRound(1+getRound());
            sendRinv(getRound(),SELF_NETWORK_ADDRESS);
            if(GetTimer(msr2mrpTimerDef::T_REPEAT)!=-1) {
                extTrace()<<"[info] cancelling T_REPEAT timer";
                CancelTimer(msr2mrpTimerDef::T_REPEAT);
            }
            SetTimer(msr2mrpTimerDef::T_REPEAT,fp.t_start*10.0);
            SetTimer(msr2mrpTimerDef::T_RESTART,fp.t_restart);
            break;
        }
        case msr2mrpTimerDef::T_SEC_L_START: {
            extTrace()<<"[timer] T_SEC_L_START timer expired, starting T_SEC_L timer";
            SetTimer(msr2mrpTimerDef::T_SEC_L,fp.t_sec_l);
            break;
        }
        case msr2mrpTimerDef::T_SEC_L: {
            extTrace()<<"[timer] T_SEC_L timer expired";
            if(isSink()) {
                extTrace()<<"[info] Sink starting second learn";
                switch(fp.second_learn) {
                    case msr2mrpSecLParDef::OFF: {
                        extTrace()<<"[error] second learn off, while T_SEC_L timer expired";
                        throw std::runtime_error("[error] Invalid state");
                    }
                    case msr2mrpSecLParDef::BROADCAST: {
                        sendLreqBroadcast(getRound(),0);
                        break;
                    }
                    case msr2mrpSecLParDef::UNICAST: {
                        if(0 == recv_table.size()) {
                            extTrace()<<"[info] Recv table size condition not met (empty)";
                            SetTimer(msr2mrpTimerDef::T_SEC_L,fp.t_sec_l);
                            break;
                        }
                        // most probably true...
                        if(!secLPerformed(getRound(), 0)) {
                            sendLreqUnicast(getRound(),0);
                            SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                        }
                        break;
                    }
                    default: {
                        extTrace()<<"[error] How do we get here?";
                    }
                }
                break;
            }
            if(msr2mrpStateDef::WORK != getState()) {
                extTrace()<<"[info] Node still not in WORK state, starting T_SEC_L timer.";
                SetTimer(msr2mrpTimerDef::T_SEC_L, fp.t_sec_l);
                break;
            }

            switch (getRingStatus()) {
                case msr2mrpRingDef::CENTRAL: {
                    extTrace()<<"[info] Node is sink, impossibru";
                    break;
                }
                case msr2mrpRingDef::INTERNAL: {
                    extTrace()<<"[info] Node is internal";
                    switch(fp.second_learn) {
                        case msr2mrpSecLParDef::OFF: {
                            extTrace()<<"[error] second learn off, while T_SEC_L timer expired";
                            throw std::runtime_error("[error] Invalid state");
                        }
                        case msr2mrpSecLParDef::BROADCAST: {
                            sendLreqBroadcast(getRound(),0);
                            break;
                        }
                        case msr2mrpSecLParDef::UNICAST: {
                            // most probably true...
                            if(!secLPerformed(getRound(), 0)) {
                                sendLreqUnicast(getRound(),0);
                                SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                            }
                            break;
                        }
                        default: {
                            extTrace()<<"[error] How do we get here?";
                        }
                    }
                    break;
                }
                case msr2mrpRingDef::BORDER: {
                    extTrace()<<"[info] Node is at border";
                    switch(fp.second_learn) {
                        case msr2mrpSecLParDef::OFF: {
                            extTrace()<<"[error] second learn off, while T_SEC_L timer expired";
                            throw std::runtime_error("[error] Invalid state");
                        }
                        case msr2mrpSecLParDef::BROADCAST: {
                            sendLreqBroadcast(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                            break;
                        }
                        case msr2mrpSecLParDef::UNICAST: {
                            // most probably true...
                            if(!secLPerformed(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS) )) {
                                sendLreqUnicast(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                                SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                            }
                            break;
                        }
                        default: {
                            extTrace()<<"[error] How do we get here?";
                        }
                    }

                    break;
                }
                case msr2mrpRingDef::EXTERNAL: {
                    extTrace()<<"[info] Node is external.";
                    bool send_sec_l=false;
                    if(1 < getRoutingTableSize()) {
                        extTrace()<<"[info] Enough routing entries";
                    } 
                    saveRreqTable();
                    clearRreqTable();
                    try {
                        constructRreqTable(selectAllPathid(0));
                    } catch (rreq_table_empty &e) {
                        extTrace()<<e.what();
                        extTrace()<<"[info] No alternate path possible to learn, propagate second learn.";
                        send_sec_l=true;
                        retrieveRreqTable();
                    }
                    catch (exception &e) {
                        extTrace()<<e.what();
                        break;
                    
                    }
                    if(send_sec_l) {
                        switch(fp.second_learn) {
                            case msr2mrpSecLParDef::OFF: {
                                extTrace()<<"[error] second learn off, while T_SEC_L timer expired";
                                throw std::runtime_error("[error] Invalid state");
                            }
                            case msr2mrpSecLParDef::BROADCAST: {
                            while(!isSecLPathidEmpty()) {
                                sendLreqBroadcast(getRound(),popSecLPathid());
                            }
                            }
                            case msr2mrpSecLParDef::UNICAST: {
                                // most probably true...
                                if(!secLPerformed(getRound(),getSecLPathid() )) {
                                    sendLreqUnicast(getRound(),getSecLPathid());
                                    SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                                }
                                break;
                            }
                            default: {
                                extTrace()<<"[error] How do we get here?";
                            }
                        }

                        break;
                    }

                    setState(msr2mrpStateDef::S_ESTABLISH);
                    sendRreqs(); 
                    SetTimer(msr2mrpTimerDef::T_ESTABLISH,getTest());
                    break;
                }
            } 
            break;
        }
        case msr2mrpTimerDef::T_SEC_L_REPEAT: {
            extTrace()<<"[timer] T_SEC_L_REPEAT timer expired";
            if(!secLPerformed(getRound(),getSecLPathid()) && g_sec_l_timeout < fp.t_sec_l_timeout-1 ) {
                sendLreqUnicast(getRound(),getSecLPathid());
                ++g_sec_l_timeout;
                SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
            }
            break;
        }

        case msr2mrpTimerDef::T_REPEAT: {
            extTrace()<<"[timer] T_REPEAT timer expired";
            sendRinv(getRound(),SELF_NETWORK_ADDRESS);
            SetTimer(msr2mrpTimerDef::T_REPEAT,fp.t_start*10.0);
            break;
        }
        case msr2mrpTimerDef::T_L: {
            extTrace()<<"[timer] T_L timer expired";

            switch (getState()) {
                case msr2mrpStateDef::LOCAL_LEARN: {
                    extTrace()<<"[info] LOCAL_LEARN finished";
                    constructRoutingTableFromRinvTable();
                    setState(msr2mrpStateDef::WORK);
                    break;
                }
                case msr2mrpStateDef::LEARN: {
                    extTrace()<<"[info] LEARN finished";
                    setState(msr2mrpStateDef::ESTABLISH);
                    clearRreqTable();
                    try {
                        constructRreqTable();
                    } catch (rinv_table_empty &e) {
                        extTrace()<<e.what();
                        extTrace()<<"[info] Empty RINV table after LEARNING state - most probably due to re-learn";
                        setRound(getRound()-1);
                        setState(msr2mrpStateDef::INIT); // could be also work, if routing table is not empty
                        break;
                    } catch (rreq_table_empty &e) {
                        extTrace()<<e.what();
                        extTrace()<<"[info] Empty RREQ table after LEARNING state - returning to INIT";
                        setRound(getRound()-1);
                        setState(msr2mrpStateDef::INIT);
                        break;
                    } catch (no_available_entry &e) {
                        extTrace()<<e.what();
                        extTrace()<<"[info] No node to connect to - returning to INIT";
                        setRound(getRound()-1);
                        setState(msr2mrpStateDef::INIT);
                        break;
                    } catch (std::exception &e) {
                        extTrace()<<e.what();
                        break;
                    }
                    sendRreqs(); //maybe we could go directly to measure or work in case of hdmrp
                    SetTimer(msr2mrpTimerDef::T_ESTABLISH,getTest());
                    break;
                }
                default: {
                    extTrace()<<"[error] State is not LEARN or LOCAL_LEARN: "<<stateToStr(getState());
                }
            }
            break;
        }
        case msr2mrpTimerDef::T_ESTABLISH: {
            extTrace()<<"[timer] T_ESTABLISH timer expired";

            if(isRreqTableEmpty()) {
                extTrace()<<"[error] RREQ table empty, impossibru";
                throw rreq_table_empty("[error] RREQ table empty");
            }

            if(fp.measure_w_rreq && (fp.meas_rreq_count > getRreqPktCount()) ) {
                sendRreqs();
                extTrace()<<"[info] measure_w_rreq active, restarting T_ESTABLISH timer";
                SetTimer(msr2mrpTimerDef::T_ESTABLISH,getTest());
                break;
            }
            if(msr2mrpStateDef::S_ESTABLISH == getState()) {
                extTrace()<<"[info] Second learn finished";
                constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /*update */ );
                retrieveAndMergeRreqTable();

                if(msr2mrpSecLParDef::OFF != fp.second_learn && isMaster()) {
                    sendRinvBasedOnHop();
                }

                switch(fp.second_learn) {
                    case msr2mrpSecLParDef::OFF: {
                        extTrace()<<"[error] second learn off, while T_SEC_L timer expired";
                        throw std::runtime_error("[error] Invalid state");
                    }
                    case msr2mrpSecLParDef::BROADCAST: {
                        while(!isSecLPathidEmpty()) {
                            sendLreqBroadcast(getRound(),popSecLPathid());
                        }
                    }
                    case msr2mrpSecLParDef::UNICAST: {
                        // most probably true...
                        if(!secLPerformed(getRound(),getSecLPathid() )) {
                            sendLreqUnicast(getRound(),getSecLPathid());
                            SetTimer(msr2mrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                        }
                        break;
                    }
                    default: {
                        extTrace()<<"[error] How do we get here?";
                    }
                }

                setState(msr2mrpStateDef::WORK);
                break;
            }

            if(!rrespReceived() && fp.rresp_req ) {
                extTrace()<<"[error] No RRESP packet received";
                if(fp.rst_learn) {
                    extTrace()<<"[info] Returning to learning state, staying in round";
                    setState(msr2mrpStateDef::LEARN);
                    //setRound(getRound()-1);
                    SetTimer(msr2mrpTimerDef::T_L,getTl());
                    if(msr2mrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                        extTrace()<<"[info] Clearing RINV table";
                        clearRinvTable();
                    }
                    if(fp.cf_after_rresp) {
                        extTrace()<<"[info] Setting used flag in all RINV entries that are present in RREQ";
                        updateRinvTableFromRreqTable();
                    }
                    break;
                }
            }

            clearRoutingTable();

            try {
                if(fp.cf_after_rresp) {
                    constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr);
                } else {
                    constructRoutingTable(fp.rresp_req);
                }
                setHop(calculateHopFromRoutingTable());
            } catch (routing_table_empty &e) {
                extTrace()<<e.what();
                extTrace()<<"[info] Returning to LEARN state";
                setState(msr2mrpStateDef::LEARN);
                SetTimer(msr2mrpTimerDef::T_L,getTl());
                break;
                // return
            }

            if(fp.interf_ping) {
                extTrace()<<"[info] Performing PING based interference measurement";
                setState(msr2mrpStateDef::MEASURE);
                SetTimer(msr2mrpTimerDef::T_MEASURE,getTmeas());
                if(fp.round_keep_pong) {
                    clearPongTable(getRound());
                } else {
                    clearPongTable();
                }
                sendPing(getRound());

            } else {
                extTrace()<<"[info] Establishment done, transitioning to WORK state";
                if(fp.e2e_qos_pdr > 0) {
                    extTrace()<<"[info] Starting T_SEND_PKT to schedule pkt sending";
                    SetTimer(msr2mrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
                }
                setState(msr2mrpStateDef::WORK);
                if(isLimitedState()) {
                    extTrace()<<"[info] Node is in LIMIT state, not sending RINV message.";
                } else {
                    sendRinvBasedOnHop();
                }
            }
            break;
        }
        case msr2mrpTimerDef::T_MEASURE: {
            extTrace()<<"[timer] T_MEASURE timer expired, PONG table size: "<<getPongTableSize();
            // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
                extTrace()<<"[info] Measurement done, transitioning to WORK state";
                if(fp.e2e_qos_pdr > 0) {
                    extTrace()<<"[info] Starting T_SEND_PKT to schedule pkt sending";
                    SetTimer(msr2mrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
                }

            setState(msr2mrpStateDef::WORK);
            if(isLimitedState()) {
                extTrace()<<"[info] Node is in LIMIT state, not sending RINV message.";
            } else {
                sendRinvBasedOnHop();
            }

            break;
        }
        case msr2mrpTimerDef::T_SEND_PKT: {
            extTrace()<<"[timer] T_SEND_PKT timer expired, pkt_list size: "<<pkt_list.size();
            if(!pkt_list.empty()) {
                for(auto it = pkt_list.begin() ; it != pkt_list.end(); ) {
                    bool erased = false;
                    extTrace()<<"[info] Entry's destination: "<<(*it)->getDestination();
                    int rep;
                    try {
                        rep = calculateRepeat((*it)->getDestination());
                    } catch (exception &e) {
                        extTrace()<<e.what();
                        extTrace()<<"[info] Remove entry";
                        msr2mrpDataPacket *pkt_ptr = *it;
                        pkt_list.erase(it++);
                        erased=true;
                        delete pkt_ptr;
                        ++it;
                        continue;
                    }
                    nw_layer->extToMacLayer((*it)->dup(), resolveNetworkAddress((*it)->getDestination()));
                    incPktCountInRoutingTable(std::string((*it)->getDestination()));

                    (*it)->setRepeat((*it)->getRepeat()+1);
                    if((*it)->getRepeat() >= rep) {
                        extTrace()<<"[info] Repeat count for pkt reached";
                        msr2mrpDataPacket *pkt_ptr = *it;
                        pkt_list.erase(it++);
                        erased=true;
                        delete pkt_ptr;
                    }
                    if(!erased) {
                        ++it;
                    }
                }
            }
            extTrace()<<"[timer] Re-scheduling T_SEND_PKT";
            SetTimer(msr2mrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
            break;
        }

        default: {
            extTrace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}


void msr2mrp_engine::sendViaPathid(cPacket *pkt, int pathid) {
    extTrace()<<"[info] sendViaPathid(pkt, pathid="<<pathid<<")";
    std::string next_hop;

    if(msr2mrpRingDef::EXTERNAL==getRingStatus()) {
        next_hop=getNextHop(pathid);
    } else {
        next_hop=getNextHop(pathid,fp.rand_ring_hop);
    }

    incPktCountInRoutingTable(next_hop);
    incOrigPktCountInRoutingTable(next_hop);

    if(fp.e2e_qos_pdr > 0.0) {
        schedulePkt(pkt, next_hop, pathid);
    }
    else {
        sendData(pkt,next_hop,pathid);
    }
   
}


void msr2mrp_engine::fromApplicationLayer(cPacket * pkt, const char *destination) {
        // Shouldn't we buffer?
        extTrace()<<"[info] fromApplicationLayer()";
        extTrace()<<"[info] Sink address: "<<getSinkAddress();
        if(isRoutingTableEmpty()) {
            extTrace()<<"[error] Routing table empty, can't route packet";
            return;
        }

        auto pathid=selectPathid(false,false);
        sendViaPathid(pkt,pathid.pathid);
}

void msr2mrp_engine::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    msr2mrpPacket *net_pkt=dynamic_cast<msr2mrpPacket *>(pkt);
    if(!net_pkt) {
        extTrace()<<"[error] Dynamic cast of packet failed";
    }

    extTrace()<<"[info] Packet received from: NW "<<net_pkt->getSource()<<" MAC: "<<srcMacAddress;

    switch (net_pkt->getMsr2mrpPacketKind()) {
        case msr2mrpPacketDef::RINV_PACKET: {
            extTrace()<<"[info] RINV_PACKET received";
            auto rinv_pkt=dynamic_cast<msr2mrpRinvPacket *>(pkt);

            handleTSecLTimer();

            if(isSink()) {
                extTrace()<<"[info] RINV_PACKET discarded by Sink";
                break;
            }

            if(rinv_pkt->getRound() > getRound()) {
                setRound(rinv_pkt->getRound());
                setHop(std::numeric_limits<int>::max());
                switch (getState()) {
                    case msr2mrpStateDef::LEARN: {
                        CancelTimer(msr2mrpTimerDef::T_L);
                        break;
                    }
                    case msr2mrpStateDef::ESTABLISH: {
                        CancelTimer(msr2mrpTimerDef::T_ESTABLISH);
                        break;
                    }
                    case msr2mrpStateDef::MEASURE: {
                        CancelTimer(msr2mrpTimerDef::T_MEASURE);
                        break;
                    }
                }
                if(msr2mrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin || msr2mrpRinvTblAdminDef::ERASE_ON_ROUND==fp.rinv_tbl_admin) {
                    clearRinvTable();
                }

                addToRinvTable(rinv_pkt);

                extTrace()<<"[info] Start LEARNING process"; 
                // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
                setSecL(false);
                setState(msr2mrpStateDef::LEARN);
                SetTimer(msr2mrpTimerDef::T_L,getTl());
                extTrace()<<"[info] Setting limited state to false, since this is a new round.";
                setLimitedState(false);


            } else if(rinv_pkt->getRound() == getRound()) {
                if(true == rinv_pkt->getLocal()) {
                    // HOP!!!!!
                    if(msr2mrpStateDef::WORK == getState()) {
                        if(checkLocalid(rinv_pkt->getLocalid())) {
                            extTrace()<<"[info] Local learn initiated by "<<rinv_pkt->getLocalid()<<" already executed, exiting";
                        } else {
                            extTrace()<<"[info] New local learn, initiated by "<<rinv_pkt->getLocalid();
                            storeLocalid(rinv_pkt->getLocalid());
                            setState(msr2mrpStateDef::LOCAL_LEARN);
                            SetTimer(msr2mrpTimerDef::T_L, getTl());
                            clearRinvTableLocalFlags();
                            try {
                                markRinvEntryLocal(std::string(rinv_pkt->getSource()));
                            } catch (no_available_entry &e) {
                                extTrace()<<e.what();
                                // maybe create?
                                break;
                            }
                        }
                        break;
                    } else if(msr2mrpStateDef::LOCAL_LEARN == getState()) {
                        if(checkLocalid(rinv_pkt->getLocalid())) {
                            extTrace()<<"[info] Local learn initiated by "<<rinv_pkt->getLocalid()<<" already executed, LOCAL_LEARN not restarting, exiting";
                            break; // this should be moved out
                        } else {
                            extTrace()<<"[info] Local learn restart due to "<<rinv_pkt->getLocalid();
                            storeLocalid(rinv_pkt->getLocalid());
                            CancelTimer(msr2mrpTimerDef::T_L);
                            SetTimer(msr2mrpTimerDef::T_L, getTl());
                            try {
                                markRinvEntryLocal(std::string(rinv_pkt->getSource()));
                            } catch (no_available_entry &e) {
                                extTrace()<<e.what();
                                // maybe create?
                                break;
                            }
                            break;
                        }
                    }
                } else if(msr2mrpStateDef::LEARN != getState() && msr2mrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                        extTrace()<<"[info] RINV packet discarded by node, not in learning state anymore";
                        break;
                    }
                addToRinvTable(rinv_pkt);
            } else {
                extTrace()<<"[info] RINV_PACKET with round "<<rinv_pkt->getRound()<<" discarded by node with round "<<getRound();
            }

            break;
        }
        case msr2mrpPacketDef::RREQ_PACKET: {
            extTrace()<<"[info] RREQ_PACKET received";
            auto rreq_pkt=dynamic_cast<msr2mrpRreqPacket *>(pkt);
            if(rreq_pkt->getRound() != getRound()) {
                extTrace()<<"[error] RREQ_PACKET's round: "<<rreq_pkt->getRound()<<" does not equal local round: "<<getRound();
                break;
            }

            handleTSecLTimer();

            if(getHop()<fp.ring_radius) {
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            } else if(getHop()==fp.ring_radius) {
                if(resolveNetworkAddress(SELF_NETWORK_ADDRESS)!=rreq_pkt->getPathid()) {
                    extTrace()<<"[error] RREQ packet's pathid "<<rreq_pkt->getPathid()<<" does not match node's network address: "<<SELF_NETWORK_ADDRESS;
                    break;
                }
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            } else {
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            }
            break;
        }
        case msr2mrpPacketDef::RRESP_PACKET: {
            extTrace()<<"[info] RRESP_PACKET received";
            auto rresp_pkt=dynamic_cast<msr2mrpRrespPacket *>(pkt);
            if(rreqEntryExists(rresp_pkt->getSource(),rresp_pkt->getPathid())) {
                updateRreqTableWithRresp(rresp_pkt->getSource(),rresp_pkt->getPathid());
            } else {
                extTrace()<<"[error] No entry in RREQ table with address "<<rresp_pkt->getSource()<<" and pathid: "<<rresp_pkt->getPathid();
                break;
            }
            break;
        }
        case msr2mrpPacketDef::LREQ_PACKET: {
            extTrace()<<"[info] LREQ_PACKET received";
            if(isSink()) {
                extTrace()<<"[info] Node is sink, discarding LREQ_PACKET";
                break;
            }
            auto lreq_packet=dynamic_cast<msr2mrpLreqPacket *>(pkt);

            if(lreq_packet->getRound() != getRound()) {
                extTrace()<<"LREQ_PACKET out of round. Packet round: "<<lreq_packet->getRound()<<" own round: "<<getRound();
                break;
            }

            if(getSecL(lreq_packet->getPathid())) {
                extTrace()<<"[info] Second learn already performed";
                if(0==std::strcmp(lreq_packet->getDestination(),SELF_NETWORK_ADDRESS)) {
                    sendLresp(lreq_packet->getSource(),getRound(),lreq_packet->getPathid());
                }
                break;
            }

            // This is not a real issue, isn't it?
            if(lreq_packet->getHop() >= getHop()) {
                extTrace()<<"[info] LREQ_PACKET hop is higher than local hop. Packet hop: "<<lreq_packet->getRound()<<" own hop: "<<getHop();
                //                break;
            }

//            if(msr2mrpRingDef::EXTERNAL == getRingStatus()) {
                // Most probably always true
                if(!checkPathid(lreq_packet->getPathid())) {
                    extTrace()<<"[info] LREQ_PACKET indicates unknown pathid, discarding.";
                    break;
                }
//            }

            if(0==std::strcmp(lreq_packet->getDestination(),SELF_NETWORK_ADDRESS)) {
                sendLresp(lreq_packet->getSource(),getRound(),lreq_packet->getPathid());
            }

            setSecL(lreq_packet->getPathid(),true);
            if(getRingStatus() == msr2mrpRingDef::BORDER) {
                pushSecLPathid(resolveNetworkAddress(SELF_NETWORK_ADDRESS));
            } else {
                pushSecLPathid(lreq_packet->getPathid());
            }
            if(GetTimer(msr2mrpTimerDef::T_SEC_L)!=-1 || getState()==msr2mrpStateDef::S_ESTABLISH ) {
                extTrace()<<"[info] T_SEC_L timer active or S_ESTABLISH state, no restart";
            } else {
                extTrace()<<"[info] Starting T_SEC_L timer";
                SetTimer(msr2mrpTimerDef::T_SEC_L, fp.t_sec_l);
            }
            break;
        }
        case msr2mrpPacketDef::LRESP_PACKET: {
            extTrace()<<"[info] LRESP_PACKET received";
            auto lresp_packet=dynamic_cast<msr2mrpLrespPacket *>(pkt);
            if(recv_table.find(std::string(lresp_packet->getSource())) != recv_table.end()) {
                extTrace()<<"[info] Entry "<<lresp_packet->getSource()<<" exists";
                if(recv_table[std::string(lresp_packet->getSource())].round == lresp_packet->getRound()) { // && recv_table[std::string(lresp_packet->getSource())].pathid.end() != std::find(recv_table[std::string(lresp_packet->getSource())].pathid.begin(), recv_table[std::string(lresp_packet->getSource())].pathid.end() ,lresp_packet->getPathid())) {
                    extTrace()<<"[info] Entry has valid round";
                    recv_table[std::string(lresp_packet->getSource())].secl=true;
                }
            }
            break;
        }
        case msr2mrpPacketDef::RWARN_PACKET: {
            extTrace()<<"[info] RWARN_PACKET received";
            auto rwarn_pkt=dynamic_cast<msr2mrpRwarnPacket *>(pkt);
            if(rwarn_pkt->getRound() > getRound()) {
                extTrace()<<"[warn] Warning node's round is greater than receiving node's round. Exiting.";
                break;
            }
            switch (rwarn_pkt->getCause()) {
                case msr2mrpWarnDef::EMERGENCY_EVENT: {
                    extTrace()<<"[info] EMERGENCY_EVENT, emergency: "<<rwarn_pkt->getEmerg()<<", energy: "<<rwarn_pkt->getEnrgy();
                    if(fp.rt_recalc_warn) {
                        try {
                            updateRreqEntryWithEmergency(rwarn_pkt->getSource(), rwarn_pkt->getEmerg(), rwarn_pkt->getEnrgy());
                            if(fp.cf_after_rresp) {
                                constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /* update */);
                            } else {
                                constructRoutingTable(fp.rresp_req);
                            }
                        } catch (no_available_entry &e) {
                            extTrace()<<"[info] Entry not available (not a real error): "<<e.what();
                        } catch (routing_table_empty &e) {
                            extTrace()<<"[error] "<<e.what();
                            setState(msr2mrpStateDef::INIT);
                        }
                    }
                    break;
                }
                case msr2mrpWarnDef::PATH_FAILURE_EVENT: {
                    extTrace()<<"[info] PATH_FAILURE_EVENT";
                    if(checkRoute(std::string(rwarn_pkt->getSource()))) {
                        removeRoute(std::string(rwarn_pkt->getSource()));
                        try { 
                            removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        } catch (no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what()<<", trying backup table";
                            try {
                                removeRreqEntry(std::string(rwarn_pkt->getSource()),true);
                            } catch (no_available_entry &e) {
                                extTrace()<<"[error] "<<e.what()<<", backup table failed, giving up";
                                break;
                            }
                        }
                        try {
                            markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                        } catch ( no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what();
                        }

  
                        handleLinkFailure(rwarn_pkt->getPathid());
                    } else if(rreqEntryExists(std::string(rwarn_pkt->getSource()))) {
                        removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    } else if(checkRinvEntry(std::string(rwarn_pkt->getSource()))) {
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    }
                    break;
                }
                case msr2mrpWarnDef::MOBILITY_EVENT: {
                    extTrace()<<"[info] MOBILITY_EVENT";
                    if(rwarn_pkt->getSource()==getSinkAddress()) {
                        extTrace()<<"[info] Sink in mobility";
                    }
                    if(checkRoute(std::string(rwarn_pkt->getSource()))) {
                        auto p=routing_table[rwarn_pkt->getSource()].pathid;
                        removeRoute(std::string(rwarn_pkt->getSource()));
                        try { 
                            removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        } catch (no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what()<<", trying backup table";
                            try {
                                removeRreqEntry(std::string(rwarn_pkt->getSource()),true);
                            } catch (no_available_entry &e) {
                                extTrace()<<"[error] "<<e.what()<<", backup table failed, giving up";
                                break;
                            }
                        }
                        try {
                            markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                        } catch ( no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what();
                        }
                        handleLinkFailure(p[0].pathid);
                    } else if(rreqEntryExists(std::string(rwarn_pkt->getSource()))) {
                        removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    } else if(checkRinvEntry(std::string(rwarn_pkt->getSource()))) {
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    }
                    break;
                }
            }
            break;
        }
        case msr2mrpPacketDef::RIREQ_PACKET: {
            extTrace()<<"[info] RIREQ_PACKET received";
            try {
                sendRinvBasedOnHop();
            } catch (std::exception &e) {
                extTrace()<<"[error] RIREQ_PACKET construction failed";
                extTrace()<<"[error] "<<e.what();
            }
            break;
        }

        case msr2mrpPacketDef::PING_PACKET: {
            extTrace()<<"[info] PING_PACKET received";
            sendPong(dynamic_cast<msr2mrpPingPacket *>(pkt)->getRound());
            handleTSecLTimer();

            break;
        }
        case msr2mrpPacketDef::PONG_PACKET: {
            extTrace()<<"[info] PONG_PACKET received";
            storePong(dynamic_cast<msr2mrpPongPacket *>(pkt));
            break;
        }
        case msr2mrpPacketDef::DATA_PACKET: {
            extTrace()<<"[info] DATA_PACKET received";
            msr2mrpDataPacket *data_pkt=dynamic_cast<msr2mrpDataPacket *>(pkt);
            incPktCountInRecvTable(std::string(data_pkt->getSource()), data_pkt->getPathid(), getRound() );
            if(isSink() && 0==std::strcmp(data_pkt->getDestination(),SELF_NETWORK_ADDRESS)) {
                extTrace()<<"[info] DATA packet arrived, forwarding to Application layer";
                data_pkt->setSource(data_pkt->getOrigin());
                nw_layer->extToApplicationLayer(decapsulatePacket(data_pkt));
                incPktCountInTrafficTable(std::string(data_pkt->getOrigin()), data_pkt->getPathid(), data_pkt->getReroute());
                break;
            } else if(0==std::strcmp(data_pkt->getDestination(), BROADCAST_NETWORK_ADDRESS)) {
                extTrace()<<"[info] Broadcast packet, forwarding to Application layer";
                nw_layer->extToApplicationLayer(decapsulatePacket(data_pkt));
            } else {
                extTrace()<<"[info] DATA packet at interim node, routing forward";
                if(data_pkt->getOrigin() == std::string(SELF_NETWORK_ADDRESS)) {
                    extTrace()<<"[error] Loop detected at path: "<<data_pkt->getPathid();
                    auto entry = getNextHop(data_pkt->getPathid());
                    removeRoute(entry);
                    try { 
                        removeRreqEntry(entry);
                    } catch (no_available_entry &e) {
                        extTrace()<<"[error] "<<e.what()<<", trying backup table";
                        try {
                            removeRreqEntry(entry,true);
                        } catch (no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what()<<", backup table failed, giving up";
                            break;
                        }
                    }
                    try {
                        markRinvEntryFail(entry);
                    } catch ( no_available_entry &e) {
                        extTrace()<<"[error] "<<e.what();
                    }
                    handleLinkFailure(data_pkt->getPathid());
                    break;



                }
                        
                incPktCountInTrafficTable(std::string(data_pkt->getOrigin()), data_pkt->getPathid(), data_pkt->getReroute());

                std::string next_hop;
                bool reroute=false;
                try {
                    if(msr2mrpRingDef::EXTERNAL==getRingStatus()) {
                        next_hop=getNextHop(data_pkt->getPathid());
                    } else {
                        next_hop=getNextHop(selectPathid(false,false).pathid,fp.rand_ring_hop );
                    }
                } catch (no_available_entry &e) {
                    extTrace()<<"[error] Next hop not available for pathid: "<<data_pkt->getPathid();
                    if(fp.reroute_pkt) {
                        extTrace()<<"[info] Rerouting packet";
                        try {
                            next_hop=getNextHop(selectPathid(false,false).pathid);
                            if(next_hop == std::string(data_pkt->getSource())) {
                                extTrace()<<"[error] Possible loop due to reroute, aborting, removing route: "<<next_hop;
                                removeRoute(next_hop);

                                removeRreqEntry(next_hop);
                                try {
                                    markRinvEntryFail(next_hop);
                                } catch ( no_available_entry &e) {
                                    extTrace()<<"[error] "<<e.what();
                                }


                                if(getRoutingTableSize() == 0) {
                                    extTrace()<<"[error] Routing table empty";
                                    try {
                                        if(fp.rt_fallb_wo_qos) {
                                            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, 0.0, true /* update */);
                                        } else {
                                            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /* update */);
                                        }
                                    } catch (exception &e) {
                                    extTrace()<<"[error] "<<e.what()<<" returning to INIT";
                                    if(fp.send_pfail_rwarn) {
                                        sendRwarn(msr2mrpWarnDef::PATH_FAILURE_EVENT,data_pkt->getPathid());
                                    }
                                    setState(msr2mrpStateDef::INIT);
                                    }

                                }
                                break;
                            }
                            reroute=true;
                        } catch (no_available_entry &e) {
                            extTrace()<<"[error] "<<e.what();
                            if(fp.send_pfail_rwarn) {
                                sendRwarn(msr2mrpWarnDef::PATH_FAILURE_EVENT,data_pkt->getPathid());
                            }

                            break;
                        } catch (routing_table_empty &e) {
                            extTrace()<<"[error] "<<e.what();
                            break;
                        }
                    } else {
                        break;
                    }
                } catch (routing_table_empty &e) {
                    extTrace()<<"[error] Routing table empty, giving up";
                    break;
                }
                if(fp.coll_pkt_at_border) {
                    if(hop_pkt_table.find(data_pkt->getHop()) != hop_pkt_table.end()) {
                        hop_pkt_table[data_pkt->getHop()]++;
                    } else {
                        hop_pkt_table[data_pkt->getHop()]=1;
                    }
                }
                incPktCountInRoutingTable(next_hop);
                forwardData(data_pkt->dup(),next_hop,reroute);
            }
            break;
        }
        case msr2mrpPacketDef::UNDEF_PACKET: {
            extTrace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            extTrace()<<"[error] Unknown packet received with msr2mrpPacketKind value: "<<net_pkt->getMsr2mrpPacketKind();
            break;
        }
    }
}

msr2mrpRingDef msr2mrp_engine::getRingStatus() const {
    if(0 == getHop()) {
        return msr2mrpRingDef::CENTRAL;
    } else if(fp.ring_radius > getHop()) {
        return msr2mrpRingDef::INTERNAL;
    } else if(fp.ring_radius == getHop()) {
        return msr2mrpRingDef::BORDER;
    }
    return msr2mrpRingDef::EXTERNAL;
}

std::string msr2mrp_engine::ringToStr(msr2mrpRingDef pos) const {
    switch (pos) {
        case msr2mrpRingDef::CENTRAL: {
            return string("central");
        }
        case msr2mrpRingDef::INTERNAL: {
            return string("internal");
        }
        case msr2mrpRingDef::BORDER: {
            return string("border");
        }
        case msr2mrpRingDef::EXTERNAL: {
            return string("external");
        }
    }
    return string("unkown");
}

void msr2mrp_engine::serializeRoutingTable() {
    serializeRoutingTable(routing_table);
}
void msr2mrp_engine::serializeRoutingTable(std::map<std::string,msr2mrp_node_entry> table) {
    y_out<<YAML::BeginSeq;
    for(auto i : table) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i.second.nw_address;
        y_out<<YAML::Key<<"pathid";
        y_out<<YAML::Value;

        y_out<<YAML::BeginSeq;
        for(auto p: i.second.pathid) {
            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"pathid";
            y_out<<YAML::Value<<p.pathid;
            y_out<<YAML::Key<<"nmas";
            y_out<<YAML::Value<<p.nmas;
            y_out<<YAML::Key<<"secl";
            y_out<<YAML::Value<<p.secl;
            y_out<<YAML::Key<<"secl_performed";
            y_out<<YAML::Key<<p.secl_performed;
            y_out<<YAML::EndMap;;

        }
        y_out<<YAML::EndSeq;



        y_out<<YAML::Key<<"hop";
        y_out<<YAML::Value<<i.second.hop;
        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::Key<<"ack_count";          
        y_out<<YAML::Value<<i.second.ack_count;
        y_out<<YAML::Key<<"fail_count";
        y_out<<YAML::Value<<i.second.fail_count;
        y_out<<YAML::Key<<"fail";
        y_out<<YAML::Value<<i.second.fail;
        y_out<<YAML::Key<<"secl";
        y_out<<YAML::Value<<i.second.secl;
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;
}

void msr2mrp_engine::serializeRecvTable() {
    serializeRecvTable(recv_table);
}

void msr2mrp_engine::serializeRecvTable(std::map<std::string,msr2mrp_node_entry> table) {
    y_out<<YAML::BeginSeq;
    for(auto i : table) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i.second.nw_address;
        y_out<<YAML::Key<<"pathid";

        y_out<<YAML::Value;

        y_out<<YAML::BeginSeq;
        for(auto p: i.second.pathid) {
            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"pathid";
            y_out<<YAML::Value<<p.pathid;
            y_out<<YAML::Key<<"nmas";
            y_out<<YAML::Value<<p.nmas;
            y_out<<YAML::EndMap;;

        }
        y_out<<YAML::EndSeq;


        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;

}

void msr2mrp_engine::serializeRadioStats(PktBreakdown stats) {
    y_out<<YAML::BeginMap;
    y_out<<YAML::Key<<"TX_pkt";
    y_out<<YAML::Value<<stats.transmissions;
    y_out<<YAML::Key<<"RX_ok_no_interf";
    y_out<<YAML::Value<<stats.RxReachedNoInterference;
    y_out<<YAML::Key<<"RX_ok_interf";
    y_out<<YAML::Value<<stats.RxReachedInterference;
    y_out<<YAML::Key<<"RX_fail_no_interf";
    y_out<<YAML::Value<<stats.RxFailedNoInterference;	
    y_out<<YAML::Key<<"RX_fail_interf";
    y_out<<YAML::Value<<stats.RxFailedInterference;
    y_out<<YAML::Key<<"RX_fail_below_sensitivity";
    y_out<<YAML::Value<<stats.RxFailedSensitivity;
    y_out<<YAML::Key<<"RX_fail_wrong_modulation";
    y_out<<YAML::Value<<stats.RxFailedModulation;
    y_out<<YAML::Key<<"RX_fail_not_rx_state";
    y_out<<YAML::Value<<stats.RxFailedNoRxState;
    y_out<<YAML::Key<<"buffer_overflow";
    y_out<<YAML::Value<<stats.bufferOverflow;
    y_out<<YAML::EndMap;
}


void msr2mrp_engine::finishSpecific() {
    if (isSink() && resolveNetworkAddress(SELF_NETWORK_ADDRESS) == 0) {
        cTopology *topo;        // temp variable to access energy spent by other nodes
        topo = new cTopology("topo");
        topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());

        set<int> paths;
        auto sink_pos=dynamic_cast<VirtualMobilityManager *>(topo->getNode(0)->getModule()->getSubmodule("MobilityManager"))->getLocation();

        y_out<<YAML::BeginSeq;
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<"0";
        y_out<<YAML::Key<<"recv_table";
        y_out<<YAML::Value;
        serializeRecvTable();
        y_out<<YAML::Key<<"state";
        y_out<<YAML::Value;
        y_out<<stateToStr(getState());


        auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(0)->getModule()->getSubmodule("MobilityManager"));

        auto radio=dynamic_cast<Radio *>(topo->getNode(0)->getModule()->getSubmodule("Communication")->getSubmodule("Radio"));

        auto loc=mob_mgr->getLocation();
        y_out<<YAML::Key<<"x";
        y_out<<YAML::Value;
        y_out<<loc.x;
        y_out<<YAML::Key<<"y";
        y_out<<YAML::Value;
        y_out<<loc.y;
        y_out<<YAML::Key<<"role";
        y_out<<YAML::Value;
        y_out<<ringToStr(getRingStatus());
        y_out<<YAML::Key<<"master";
        y_out<<YAML::Value<<isMaster();
        y_out<<YAML::Key<<"pong_table";
        y_out<<YAML::Value<<getPongTableSize();
        y_out<<YAML::Key<<"radio";
        y_out<<YAML::Value;
        serializeRadioStats(radio->getStats());
            y_out<<YAML::Key<<"traffic_table";
            y_out<<YAML::Value;
            y_out<<YAML::BeginSeq;
            for(auto ne: getTrafficTable()) {
                y_out<<YAML::BeginMap;
                y_out<<YAML::Key<<"node";
                y_out<<YAML::Value<<ne.first;
                y_out<<YAML::Key<<"pkt";
                y_out<<YAML::Value<<ne.second.pkt_count;
                y_out<<YAML::Key<<"pathid";
                y_out<<YAML::Value;
                y_out<<YAML::BeginSeq;
                for(auto p: ne.second.pathid) {
                    y_out<<YAML::BeginMap;
                    y_out<<YAML::Key<<"pathid";
                    y_out<<YAML::Value<<p.pathid;
                    y_out<<YAML::EndMap;
                }
                y_out<<YAML::EndSeq;
                y_out<<YAML::Key<<"reroute_count";
                y_out<<YAML::Value<<ne.second.reroute_count;
                y_out<<YAML::EndMap;
            }
            y_out<<YAML::EndSeq;
            y_out<<YAML::Key<<"forw_data_pkt_count";
            y_out<<YAML::Value<<"0";
            y_out<<YAML::Key<<"hop";
            y_out<<YAML::Value<<"0";

        y_out<<YAML::EndMap;

        for (int i = 1; i < topo->getNumNodes(); ++i) {
            msr2mrp *msr2mrp_instance = dynamic_cast<msr2mrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
            auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"));

            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"node";
            y_out<<YAML::Value<<i;
            try {
                auto table=msr2mrp_instance->getRecvTable();
                y_out<<YAML::Key<<"recv_table";
                y_out<<YAML::Value;
                serializeRecvTable(table);
            } catch (exception &e) {
                extTrace()<<"[error] No recv table: "<<e.what();
            }
            try {
                auto table=msr2mrp_instance->getRoutingTable();
                y_out<<YAML::Key<<"routing_table";
                y_out<<YAML::Value;
                serializeRoutingTable(table);
            } catch (exception &e) {
                extTrace()<<"[error] No routing table: "<<e.what();
            }
            try {
                auto table=msr2mrp_instance->getRreqTable();
                y_out<<YAML::Key<<"rreq_table";
                y_out<<YAML::Value;
                serializeRoutingTable(table);
            } catch (exception &e) {
                extTrace()<<"[error] No rreq table: "<<e.what();
            }
            try {
                auto table=msr2mrp_instance->getRinvTable();
                y_out<<YAML::Key<<"rinv_table";
                y_out<<YAML::Value;
                serializeRoutingTable(table);
            } catch (exception &e) {
                extTrace()<<"[error] No rinv table: "<<e.what();
            }
            auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));

            y_out<<YAML::Key<<"state";
            y_out<<YAML::Value<<(res_mgr->isDead()?"DEAD":stateToStr(msr2mrp_instance->getState()));
            y_out<<YAML::Key<<"round";
            y_out<<YAML::Value<<msr2mrp_instance->getRound();
            y_out<<YAML::Key<<"second_learn";
            y_out<<YAML::Value<<msr2mrp_instance->getSecL();

            auto radio=dynamic_cast<Radio *>(topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Radio"));
            y_out<<YAML::Key<<"radio";
            y_out<<YAML::Value;
            serializeRadioStats(radio->getStats());

            

            // Format: {'Node1': [0,0], 'Node2': [0,10],'Node3':[10,0]}
            auto loc=mob_mgr->getLocation();
            y_out<<YAML::Key<<"x";
            y_out<<loc.x;
            y_out<<YAML::Key<<"y";
            y_out<<loc.y;
            y_out<<YAML::Key;
            y_out<<"role";
            y_out<<YAML::Value;
            y_out<<(res_mgr->isDead()?"dead":ringToStr(msr2mrp_instance->getRingStatus()));
            y_out<<YAML::Key<<"remaining_energy";
            y_out<<YAML::Value<<res_mgr->getRemainingEnergy();
            y_out<<YAML::Key<<"spent_energy";
            y_out<<YAML::Value<<res_mgr->getSpentEnergy();
            y_out<<YAML::Key<<"pong_table";
            y_out<<YAML::Value<<msr2mrp_instance->getPongTableSize();
            y_out<<YAML::Key;
            y_out<<"hop";
            y_out<<YAML::Value;
            y_out<<msr2mrp_instance->getHop();
            y_out<<YAML::Key;
            y_out<<"master";
            y_out<<YAML::Value;
            y_out<<msr2mrp_instance->isMaster();
            y_out<<YAML::Key<<"traffic_table";
            y_out<<YAML::Value;
            y_out<<YAML::BeginSeq;
            for(auto ne: msr2mrp_instance->getTrafficTable()) {
                y_out<<YAML::BeginMap;
                y_out<<YAML::Key<<"node";
                y_out<<YAML::Value<<ne.first;
                y_out<<YAML::Key<<"pkt";
                y_out<<YAML::Value<<ne.second.pkt_count;
                y_out<<YAML::Key<<"pathid";
                y_out<<YAML::Value;
                y_out<<YAML::BeginSeq;
                for(auto p: ne.second.pathid) {
                    y_out<<YAML::BeginMap;
                    y_out<<YAML::Key<<"pathid";
                    y_out<<YAML::Value<<p.pathid;
                    y_out<<YAML::EndMap;
                }
                y_out<<YAML::EndSeq;
                y_out<<YAML::Key<<"reroute_count";
                y_out<<YAML::Value<<ne.second.reroute_count;
                y_out<<YAML::EndMap;
            }
            y_out<<YAML::EndSeq;
            y_out<<YAML::Key<<"forw_data_pkt_count";
            y_out<<YAML::Value<<msr2mrp_instance->getForwDataPkt();
            y_out<<YAML::EndMap;

            // seek back one character

        }
        y_out<<YAML::EndSeq;
        ofstream loc_pdr_file("loc_pdr.yaml");
        loc_pdr_file<<y_out.c_str();
        loc_pdr_file.close();

        YAML::Emitter ys_out;
        ys_out<<YAML::BeginSeq;
        for(auto se: state_chng_log) {
            ys_out<<YAML::BeginMap;
            ys_out<<YAML::Key<<"node";
            ys_out<<YAML::Value<<se.node;
            ys_out<<YAML::Key<<"timestamp";
            ys_out<<YAML::Value<<se.timestamp;
            ys_out<<YAML::Key<<"state";
            ys_out<<YAML::Value<<stateToStr(se.state);
            ys_out<<YAML::Key<<"energy";
            ys_out<<YAML::Value<<se.energy;
            ys_out<<YAML::Key<<"total_energy";
            ys_out<<YAML::Value<<se.total_energy;
            ys_out<<YAML::EndMap;

        }
        ys_out<<YAML::EndSeq;
        ofstream state_file("state_chng.yaml");
        state_file<<ys_out.c_str();
        state_file.close();


        delete(topo);
    }
    delete stimer;
    return;
}

void msr2mrp_engine::handleMacControlMessage(cMessage *msg) {
    extTrace()<<"[info] Entering handleMacControlMessage()";
    TMacControlMessage *mac_msg=check_and_cast<TMacControlMessage *>(msg);
    if(!mac_msg) {
        extTrace()<<"[error] Not TMacControlMessage";
        throw cRuntimeError("[error] Message not a TMacControlMessage");
    }
    extTrace()<<"[info] Event: "<<mac_msg->getMacControlMessageKind()<<" Node: "<<mac_msg->getDestination()<<" Seqnum: "<<mac_msg->getSeq_num();
    std::string nw_address = std::to_string(mac_msg->getDestination());
    switch (mac_msg->getMacControlMessageKind()) {
        case MacControlMessage_type::ACK_RECV: {
            extTrace()<<"[info] ACK_RECV received.";
            if(routing_table.find(nw_address) != routing_table.end()) {
                routing_table[nw_address].ack_count++;
                if(fp.detect_link_fail) {
                    extTrace()<<"[info] Resetting fail_count";
                    routing_table[nw_address].fail_count = 0;
                }
            }
            break;
        }
        case MacControlMessage_type::PKT_FAIL: {
            extTrace()<<"[info] PKT_FAIL received.";
            if(routing_table.find(nw_address) != routing_table.end()) {
                routing_table[nw_address].fail_count++;
                if(fp.detect_link_fail && fp.fail_count <= static_cast<int>(static_cast<double>(routing_table[nw_address].fail_count))/fp.qos_pdr && getHop() > 2 && getState() == msr2mrpStateDef::WORK) { // Ugly :-( - do not check for secL
                    extTrace()<<"[info] Link "<<nw_address<<" failed, removing";
                    auto p=routing_table[nw_address].pathid;
                    removeRoute(nw_address);
                    removeRreqEntry(nw_address);
                    try {
                        markRinvEntryFail(nw_address);
                    } catch ( no_available_entry &e) {
                        extTrace()<<"[error] "<<e.what();
                    }
    
                    handleLinkFailure(p[0].pathid);
                }
            }
            break;
        }
    }
}

bool msr2mrp_engine::secLPerformed(int round, int pathid) {
    extTrace()<<"[info] Entering secLPerformed(round="<<round<<", pathid="<<pathid<<")";
    bool performed=true;
    for(auto ne: recv_table) {
        if(ne.second.round == round) {
            for(auto p: ne.second.pathid) {
                if(p.pathid==pathid && ne.second.secl == false) {
                    extTrace()<<"[info] "<<ne.second.nw_address<<" second learn still missing";
                    performed=false;
                }
            }
        }
    }
    return performed;
}



void msr2mrp_engine::handleNetworkControlCommand(cMessage *msg) {
    extTrace()<<"[info] Entering handleNetworkControlCommand()";
    EmergencyMessage *app_msg=check_and_cast<EmergencyMessage *>(msg);
    if(!msg) {
        extTrace()<<"[error] Unknown Network Control Command Message";
    }
    switch (app_msg->getEvent()) {
        case MsgType::EMERGENCY: {
            extTrace()<<"[info] Application in Emergency state";
            break;
        }
        case MsgType::RELEARN: {
            extTrace()<<"[info] Application finished mobility";
            if(isSink()) {
                extTrace()<<"[info] Sink starts new round after mobility";
                setRound(1+getRound());
                sendRinv(getRound(),SELF_NETWORK_ADDRESS);
                SetTimer(msr2mrpTimerDef::T_REPEAT,fp.t_start*10.0);
                if(fp.periodic_restart) {
                    SetTimer(msr2mrpTimerDef::T_RESTART,fp.t_restart);
                }
            }
            break;
        }
    }
}


void msr2mrp_engine::sendRwarn() {
    extTrace()<<"[info] Entering sendRwarn()";
    sendRwarn(msr2mrpWarnDef::EMERGENCY_EVENT,selectPathid(true,false).pathid);
}

void msr2mrp_engine::sendRwarn(msr2mrpWarnDef cause, int pathid) {
    extTrace()<<"[info] Entering sendRwarn(cause="<<cause<<", pathid="<<pathid<<")";
    msr2mrpRwarnPacket *warn_pkt=new msr2mrpRwarnPacket("MSR2MRP RWARN packet", NETWORK_LAYER_PACKET);
    warn_pkt->setByteLength(netDataFrameOverhead);
    warn_pkt->setSink(getSinkAddress().c_str());
    warn_pkt->setCause(cause);
    warn_pkt->setMsr2mrpPacketKind(msr2mrpPacketDef::RWARN_PACKET);
    warn_pkt->setSource(SELF_NETWORK_ADDRESS);
    warn_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    warn_pkt->setRound(getRound());
    warn_pkt->setPathid(pathid);
    warn_pkt->setHop(getHop());
    warn_pkt->setEmerg(nw_layer->getEnergyValue());
    warn_pkt->setEnrgy(nw_layer->getEmergencyValue());
    warn_pkt->setSequenceNumber(currentSequenceNumber++);
    nw_layer->extToMacLayer(warn_pkt, BROADCAST_MAC_ADDRESS);

}


void msr2mrp_engine::incPktCountInTrafficTable(std::string node, int pathid, int reroute) {
    if(traffic_table.find(node) == traffic_table.end()) {
        traffic_table[node].nw_address=node;
        traffic_table[node].pkt_count=1;
        traffic_table[node].reroute_count=reroute;
        traffic_table[node].pathid.push_back({pathid,0});
    } else {
        ++(traffic_table[node].pkt_count);
        traffic_table[node].reroute_count+=reroute;
        bool add=true;
        for(auto ne: traffic_table[node].pathid) {
            if(ne.pathid==pathid) {
                add=false;
            }
        }
        if(add) {
            traffic_table[node].pathid.push_back({pathid,0});
        }
    }
}

void msr2mrp_engine::handleLinkFailure(int p) {
    extTrace()<<"[info] Entering handleLinkFailure(p="<<p<<")";
    try {
        if(fp.rt_fallb_wo_qos) {
            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, 0, true /* update */);
        } else {
            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /* update */);
        }
    } catch (exception &e) {
        extTrace()<<"[error] "<<e.what()<<" returning to INIT";
        setState(msr2mrpStateDef::INIT);
        if(fp.send_pfail_rwarn) {
            sendRwarn(msr2mrpWarnDef::PATH_FAILURE_EVENT,p);
        }
    }
}



