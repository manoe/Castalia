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

enum shmrp_v1PacketDef {
    UNDEF_PACKET        = 0;
    DATA_PACKET         = 1;
    RINV_PACKET         = 2;
    RREQ_PACKET         = 4;
    RRESP_PACKET        = 8;
    PING_PACKET         = 16;
    PONG_PACKET         = 32;
}


// RREQ = { R, S, P, len, nmas }
packet shmrp_v1Packet extends RoutingPacket {
    int shmrp_v1PacketKind enum (shmrp_v1PacketDef); // 1 byte
    string origin;
}

packet shmrp_v1RinvPacket extends shmrp_v1Packet {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
    int interf;     // 1 byte
    int emerg;      // 1 byte
}

packet shmrp_v1RreqPacket extends shmrp_v1Packet {
    int round;      // 2 byte
    int pathid;     // 1 byte
//    int hop;        // 2 byte probably makes no sense
}

packet shmrp_v1RrespPacket extends shmrp_v1Packet {
    int round;      // 2 byte
    int pathid;     // 1 byte
    int hop;        // 2 byte
}

packet shmrp_v1DataPacket extends shmrp_v1Packet {
    int pathid;
    int hop;
}

packet shmrp_v1PingPacket extends shmrp_v1Packet {
    int round;
}

packet shmrp_v1PongPacket extends shmrp_v1Packet {
    int round;
}
