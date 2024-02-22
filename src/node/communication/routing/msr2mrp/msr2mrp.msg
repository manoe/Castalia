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

enum msr2mrpPacketDef {
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

enum msr2mrpWarnDef {
    UNDEF_WARN          = 0;
    EMERGENCY_EVENT     = 1;
    NODE_FAILURE_EVENT  = 2;
    PATH_FAILURE_EVENT  = 3;
}

struct msr2mrpPathDef {
    int    origin;
    int    pathid;
    int    nmas;
    double enrgy;
    double emerg;    // 1 byte
    double pdr;
}

// RREQ = { R, S, P, len, nmas }
packet msr2mrpPacket extends RoutingPacket {
    int msr2mrpPacketKind enum (msr2mrpPacketDef); // 1 byte
    string origin;
    string sink;
}

packet msr2mrpRinvPacket extends msr2mrpPacket {
    int          round;    // 2 byte
    msr2mrpPathDef pathid[]; // 1 byte
    int          hop;      // 2 byte
    int          interf;   // 1 byte
    int          nmas;     // 1 byte
    bool         local;    // 1 byte
    int          localid;  // 1 byte
}

packet msr2mrpRwarnPacket extends msr2mrpPacket {
    int    cause enum (msr2mrpWarnDef); // 1 byte
    int    round;      // 2 byte
    int    pathid;     // 1 byte
    int    hop;        // 2 byte
    double emerg;   // 1 byte
    double enrgy;   // 1 byte
}

packet msr2mrpRreqPacket extends msr2mrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
//    int hop;        // 2 byte probably makes no sense
}

packet msr2mrpRrespPacket extends msr2mrpPacket {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
}

// msr2mrp learn request
packet msr2mrpLreqPacket extends msr2mrpPacket {
    int round;
    int pathid;
    int hop;
}

// msr2mrp learn response
packet msr2mrpLrespPacket extends msr2mrpPacket {
    int round;
    int pathid;
    int hop;
}

packet msr2mrpDataPacket extends msr2mrpPacket {
    int pathid;
    int hop;
    int reroute;
    int repeat;
}

packet msr2mrpPingPacket extends msr2mrpPacket {
    int round;
}

packet msr2mrpPongPacket extends msr2mrpPacket {
    int round;
}
