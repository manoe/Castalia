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
    PING_PACKET         = 16;
    PONG_PACKET         = 32;
    RWARN_PACKET        = 64;
    LREQ_PACKET         = 128;
    LRESP_PACKET        = 256;
}

enum shmrpWarnDef {
    UNDEF_WARN          = 0;
    EMERGENCY_EVENT     = 1;
    NODE_FAILURE_EVENT  = 2;
}


// RREQ = { R, S, P, len, nmas }
packet shmrpPacket extends RoutingPacket {
    int shmrpPacketKind enum (shmrpPacketDef); // 1 byte
    string origin;
}

packet shmrpRinvPacket extends shmrpPacket {
    int  round;      // 2 byte
    int  pathid;     // 1 byte
    int  hop;        // 2 byte
    int  interf;     // 1 byte
    int  emerg;      // 1 byte
    bool local;      // 1 byte
    int  localid;    // 1 byte
}

packet shmrpRwarnPacket extends shmrpPacket {
    int cause enum (shmrpWarnDef); // 1 byte
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
}

packet shmrpRreqPacket extends shmrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
//    int hop;        // 2 byte probably makes no sense
}

packet shmrpRrespPacket extends shmrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
}

// shmrp learn request
packet shmrpLreqPacket extends shmrpPacket {
    int round;
    int pathid;
    int hop;
}

// shmrp learn response
packet shmrpLrespPacket extends shmrpPacket {
    int round;
    int pathid;
    int hop;
}

packet shmrpDataPacket extends shmrpPacket {
    int pathid;
    int hop;
    int reroute;
}

packet shmrpPingPacket extends shmrpPacket {
    int round;
}

packet shmrpPongPacket extends shmrpPacket {
    int round;
}
