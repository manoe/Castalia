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

enum shmrpPacketDef {
    UNDEF_PACKET        = 0;
    DATA_PACKET         = 1;
    RINV_PACKET         = 2;
    RREQ_PACKET         = 4;
    RRESP_PACKET        = 8;
}

//enum hdmrpRoleDef {
//    SINK        = 1;
//    ROOT        = 2;
//    SUB_ROOT    = 3;
//    NON_ROOT    = 4;
//}

enum shmrpStateDef {
    WORK    = 1;
    INIT    = 2;
    LEARN   = 3;
    RELAY   = 5;
}

enum shmrpTimerDef {
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
packet shmrpPacket extends RoutingPacket {
    int shmrpPacketKind enum (shmrpPacketDef); // 1 byte
}

packet shmrpRinvPacket extends shmrpPacket {
    int round;      // 2 byte
    int path_id;    // 1 byte
    int hop;        // 2 byte
}

packet shmrpRreqPacket extends shmrpPacket {
    int round;      // 2 byte
    int path_id;    // 1 byte
    int hop;        // 2 byte
}

packet shmrpRrespPacket extends shmrpPacket {
    int round;      // 2 byte
    int path_id;    // 1 byte
    int hop;        // 2 byte
}

packet shmrpDataPacket extends shmrpPacket {
    int path_id;
    int hop;
    double data;
}
