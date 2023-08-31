enum MsgType {
    EMERGENCY   = 1;
    STEADY      = 2;
}

message EmergencyMessage {
    int event enum(MsgType);
    double environment;
    double energy;
}
