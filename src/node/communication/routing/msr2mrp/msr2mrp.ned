//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************

package node.communication.routing.msr2mrp;

simple msr2mrp like node.communication.routing.iRouting
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
	bool   collectTraceInfo = default (false);
	int    maxNetFrameSize = default (0);	    // bytes
	int    netDataFrameOverhead = default (10); // bytes
	int    netBufferSize = default (32);	    // number of messages
    double t_l = default (1);                   // Tl timer, default 1 sec
    double t_est = default(2);                  // Test timer, default 2 sec
    double t_meas = default(2);                 // Tmeas timer, default 2 sec
    int    ring_radius = default(2);            // Root radius in hops

    double t_relay = default(0.1);              // relay timer
    double t_start = default (1);               // sink start timer
    double t_restart = default(7200);           // Restart network timer
    string sink_address = default("0");         // sink's network address

    bool   f_rresp_required   = default(false); // Expect RRESP to use route
    bool   f_measure_w_rreq   = default(false); // Measure with subsequent RREQ messages
    int    f_meas_rreq_count  = default(10);    // How many RREQs should we send
    bool   f_restart_learning = default(false); // Restart learning if isolated
    bool   f_replay_rinv      = default(false); // Ask neighbours to re-send RINVs within round
    string f_cost_function    = default("hop"); // Cost Functions, available:
                                                                  // o hop
                                                                  // o hop_and_interf
                                                                  // o hop_emerg_and_interf
                                                                  // o hop_and_pdr
                                                                  // o hop_pdr_and_interf
                                                                  // o hop_emerg_pdr_and_interf
                                                                  // o xpr_interf
                                                                  // o xpr_hop_and_pdr
                                                                  // o xpr_hop_pdr_and_interf, 
                                                                  // o sum_hop_enrgy_emerg_pdr_and_interf
    double f_cost_func_epsilon= default(1.0);   // Epsilon coeff. in cost function,  controls emerg term
    double f_cost_func_iota   = default(1.0);   // Iota coeff. in cost function, controls interference term
    double f_cost_func_pi     = default(1.0);   // Pi coeff. in cost function, controls pdr term
    double f_cost_func_phi    = default(1.0);   // Phi coeff. in cost function, controls hop term
    double f_cost_func_mu     = default(1.0);   // Mu coeff. in cost function, controls nmas term
    double f_cost_func_eta    = default(1.0);   // Eta coeff. in cost function, controls energy term
    bool   f_cf_after_rresp   = default(false); // Apply cost function based selection after RREQ/RRESP

    bool   f_random_t_l       = default(false); // Randomize Tl timer
    double f_random_t_l_sigma = default(0.2);   // Sigma parameter of random Tl timer, where Tl is mu
    string f_rinv_table_admin = default("erase_on_learn"); // How to handle RINV table, available: erase_on_learn, erase_on_round, never_erase 
    bool   f_interf_ping      = default(false); // Perform interference measurement with PING messages
    bool   f_round_keep_pong  = default(false); // Keep PONG messages received in the same round

    bool   f_rand_ring_hop    = default(false); // Select random nextHop inside ring

    bool   f_static_routing   = default(false); // Use static routing
    bool   f_calc_max_hop     = default(false); // Calculate maximum possible hop
    double f_qos_pdr          = default(0.0);   // Apply QoS limit to measured PDR
    string f_routing_file     = default("routes.yaml"); // YAMl file describing static routing
    string f_loc_rt_strat     = default("rreq_tbl"); // Routing table construction stategy during local update: rreq_table (based on rreq_table), rreq
    bool   f_rt_recalc_warn   = default(false); // Recalculate routing table with emergency parameter, in case of WARN message
    bool   f_reroute_pkt      = default(false); // Reroute data pkt, if routing table entry is missing
    string f_second_learn     = default("off"); // Perform a second learn, off: broadcast, unicast
    double f_t_sec_l          = default(10.0);  // Second learn's start timer. Restarts after every RINV
    double f_t_sec_l_repeat   = default(2.0);   // Second learn's repeat timer in case of unicast
    int    f_t_sec_l_timeout  = default(5);     // Timeout counter for second_learn, give after f_sec_l_timet tries
    double f_t_sec_l_start    = default(120);   // Grand delay to defer second learn's start

    bool   f_detect_link_fail = default(false); // Detect failing links
    bool   f_rt_fallb_wo_qos  = default(false); // Create routing table during fallback without enforcing QoS
    bool   f_send_pfail_rwarn = default(false); // Send Path failure RWARN message
    int    f_fail_count       = default(10);    // Declare a link failing after f_fail_count fail msgs.
    int    f_path_sel         = default(0);     // How to select path (initially): 0 - random, 1 - only primary - 2 - prefer second
    double f_e2e_qos_pdr      = default(0.0);   // Defines the e2e QoS PDR that has to be met. Based on the value,
                                                // the link's hop number and the link-level QoS PDR a packet
                                                // repetition count is identified.
    double f_t_send_pkt       = default(1);     // Timer to repeatedly send scheduled packets to meet e2e QoS PDR
    bool   f_rep_m_pdr        = default(false); // Calculate packet repetition (e2e pdr) based on measured link PDR
    bool   f_drop_1st_rt_c    = default(false); // Drop the 1st candidate from routing table
    double f_drop_prob        = default(0.1);   // Drop probability
    bool   f_e2e_cost         = default(false); // Use end-to-end values for pdr, energy and emergency
    bool   f_periodic_restart = default(false); // Restart network periodically

    int t_rreq = default (10); // Trreq timer, default 10 sec
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

