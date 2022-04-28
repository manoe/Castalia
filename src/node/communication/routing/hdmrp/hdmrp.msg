//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2022                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


cplusplus {{
#include "node/communication/routing/RoutingPacket_m.h"
}}

class RoutingPacket;

enum hdmrpPacketDef {
    DATA_PACKET         = 1;
    RREQ_PACKET         = 2;
    SINK_RREQ_PACKET    = 4;
    ACK_PACKET          = 8;
    PATH_CONFIRM_PACKET = 16;
}

enum hdmrpRoleDef {
    SINK        = 1;
    ROOT        = 2;
    SUB_ROOT    = 3;
    NON_ROOT    = 4;
}

enum hdmrpStateDef {
    WORK    = 1;
    INIT    = 2;
    LEARN   = 3;
    RELAY   = 4;
}

enum hdmrpTimerDef {
    SINK_START       = 1;
    T_L              = 2;
    ROOT_GUARD       = 3;
    SUB_ROOT_GUARD   = 4;
    NEW_ROUND        = 5;
    T_RELAY          = 6;
    ACK_HIST_PURGE   = 7;
    PACKET_TIMER_1   = 8;
}

// RREQ = { R, S, P, len, nmas }
packet hdmrpPacket extends RoutingPacket {
    int hdmrpPacketKind enum (hdmrpPacketDef); // 1 byte
    int round;      // 2 byte
    int path_id;    // 2 byte
    int len;        // 1 byte?
    int nmas;       // 1 byte?
    bool ack_req;
    int rep_count;
    int orig;
    int l_seq;      // 1 byte
}

