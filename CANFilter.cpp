
namespace CANFilter {
    
    //Have to keep track of how many IDs we have
    unsigned short stdCANCount = 0;     //Word Size: 1/2
    unsigned short stdGrpCANCount = 0;  //Word Size: 1
    unsigned short extCANCount = 0;     //Word Size: 1
    unsigned short extGrpCANCount = 0;  //Word Size: 2
    
    void setFilterMode(FilterMode mode) {
        LPC_CANAF->AFMR = static_cast<int>(mode);
    }
    
    void resetFilter() {
        //Reset our item count
        stdCANCount = 0;
        stdGrpCANCount = 0;
        extCANCount = 0;
        extGrpCANCount = 0;
        
        //Set the Mode Register to bypass, leave to accept all messages
        LPC_CANAF->AFMR = 1;
        
        //Set the Look-Up Table starts to 0 (All off)
        LPC_CANAF->SFF_sa = 0;
        LPC_CANAF->SFF_GRP_sa = 0;
        LPC_CANAF->EFF_sa = 0;
        LPC_CANAF->EFF_GRP_sa = 0;
        LPC_CANAF->ENDofTable = 0;
    }
    
    void calculateAddresses() {
        //Set the Mode Register to bypass, allow writes to registers
        LPC_CANAF->AFMR = 1;
        
        //Beginning of the Standard (11-bit) CAN message filter 
        LPC_CANAF->SFF_sa       = 0;
        //Beginning of the Standard (11-bit) Group CAN message filter
        LPC_CANAF->SFF_GRP_sa   = LPC_CANAF->SFF_sa + (((stdCANCount + 1) / 2) * 4);
        //Beginning of the Etended (29-bit) CAN message filter 
        LPC_CANAF->EFF_sa       = LPC_CANAF->SFF_GRP_sa + (stdGrpCANCount * 4);
        //Beginning of the Standard (29-bit) Group CAN message filter
        LPC_CANAF->EFF_GRP_sa   = LPC_CANAF->EFF_sa + (extCANCount * 4);
        //End of the acceptance filter
        LPC_CANAF->ENDofTable   = LPC_CANAF->EFF_GRP_sa + ((extGrpCANCount * 2) * 4);
        
        //Set the Mode Register to operating, use the filter
        LPC_CANAF->AFMR = 0;
    }
}
