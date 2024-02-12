//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
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

enum r2mrpPacketDef {
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

enum r2mrpWarnDef {
    UNDEF_WARN          = 0;
    EMERGENCY_EVENT     = 1;
    NODE_FAILURE_EVENT  = 2;
    PATH_FAILURE_EVENT  = 3;
}

struct r2mrpPathDef {
    int    pathid;
    int    nmas;
    double enrgy;
    double emerg;    // 1 byte
    double pdr;
}

// RREQ = { R, S, P, len, nmas }
packet r2mrpPacket extends RoutingPacket {
    int r2mrpPacketKind enum (r2mrpPacketDef); // 1 byte
    string origin;
}

packet r2mrpRinvPacket extends r2mrpPacket {
    int          round;    // 2 byte
    r2mrpPathDef pathid[]; // 1 byte
    int          hop;      // 2 byte
    int          interf;   // 1 byte
    int          nmas;     // 1 byte
    bool         local;    // 1 byte
    int          localid;  // 1 byte
}

packet r2mrpRwarnPacket extends r2mrpPacket {
    int    cause enum (r2mrpWarnDef); // 1 byte
    int    round;      // 2 byte
    int    pathid;     // 1 byte
    int    hop;        // 2 byte
    double emerg;   // 1 byte
    double enrgy;   // 1 byte
}

packet r2mrpRreqPacket extends r2mrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
//    int hop;        // 2 byte probably makes no sense
}

packet r2mrpRrespPacket extends r2mrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
}

// r2mrp learn request
packet r2mrpLreqPacket extends r2mrpPacket {
    int round;
    int pathid;
    int hop;
}

// r2mrp learn response
packet r2mrpLrespPacket extends r2mrpPacket {
    int round;
    int pathid;
    int hop;
}

packet r2mrpDataPacket extends r2mrpPacket {
    int pathid;
    int hop;
    int reroute;
    int repeat;
}

packet r2mrpPingPacket extends r2mrpPacket {
    int round;
}

packet r2mrpPongPacket extends r2mrpPacket {
    int round;
}
