//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2023                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************

package node.communication.routing.smrp;

simple smrp like node.communication.routing.iRouting
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

    string sink_address = default("0");

    double t_start      = default(2);    // Sink start timer
    double t_ttl        = default(20);
    double t_field      = default(5);
    double t_query      = default(2);
    double t_env_c      = default(10);
    double t_d_update   = default(600);
    double t_restart    = default(7800);

    double p_alpha      = default(0.2);
    double p_beta       = default(0.2);
    int    p_pnum       = default(2); // Number of paths to establish
    double p_gamma      = default(0.5);
    double p_n_lim      = default(0.1);
    bool   p_periodic_restart = default(false);
    bool   p_a_paths    = default(true); // Consider only available paths
    bool   p_c_dead     = default(false); // Consider dead paths, to stop re-try of path establishment


 gates:
	output toCommunicationModule;
	output toMacModule;
	input fromCommunicationModule;
	input fromMacModule;
	input fromCommModuleResourceMgr;
}

