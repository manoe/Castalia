//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2022                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************

package node.communication.routing.hdmrp;

simple hdmrp like node.communication.routing.iRouting
//====================================================================================
// Network_GenericFrame has the following real-world
// (non-simulation-specific) fields:
//    unsigned short int frameType; --> 2bytes
//    string source;  ----------------> 2bytes
//    string destinationCtrl; --------> 2bytes
//    string lastHop; ------------> 2bytes
//    string nextHop; ------------> 2bytes
//    unsigned short int ttl; ----> 2bytes
//    string applicationID; ------> 2bytes
// Total bytes = 7*2 = 14 (|*|)
// From these 14bytes, BypassRoutingModule doesn't use everything.
// It doesn't use the ttl and applicationID fields.
// Concluding the calculations, the Network_GenericFrame for
// BypassRoutingModule has a total overhead of:
// 14-(2+2) = 10 bytes
//====================================================================================
{
 parameters:
	bool collectTraceInfo = default (false);
	int maxNetFrameSize = default (0);	// bytes
	int netDataFrameOverhead = default (10);	// bytes
	int netBufferSize = default (32);	// number of messages
    int t_l = default (1); // Tl timer, default 1 sec
    double t_relay = default(0.1); // relay timer
    int t_rreq = default (10); // Trreq timer, default 10 sec
    int t_start = default (1); // sink start timer
    double min_rreq_rssi = default(-100.0); // Minimum RSSI to accept a (S)RREQ
    int t_pkt_hist  = default (5); // keep pkt history entry at least this long
    double t_rsnd = default(1); // Resend timer
    int rep_limit = default(3); // give after rep_limit and send path failure // give after rep_limit and send path failure
    int report_ack_period = default(10);
    int event_ack_req_period = default (10);
    bool send_path_failure = default(true);
    bool store_all_paths   = default(false);
    bool path_confirm      = default(true);
    bool minor_rreq        = default(true);
    int  resel_limit       = default(3);
    bool default_ack       = default(false);
    int  fail_limit        = default(10);
    bool reset_fail        = default(false);
    bool static_route      = default(false);
    int  static_next_hop   = default(0);
    int  static_path       = default(0);

 gates:
	output toCommunicationModule;
	output toMacModule;
	input fromCommunicationModule;
	input fromMacModule;
	input fromCommModuleResourceMgr;
}

