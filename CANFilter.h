#ifndef CANFILTER_H
#define CANFILTER_H

#include "mbed.h"

namespace CANFilter {
    /**
     * Filter modes for the CAN acceptance filter of the LPC1768.
     * off - Typically used for initializations. Allows r/w access to all 
     *  registers and to the Look-Up Table RAM. No messages accepted.
     * bypass - Can be used, for example, to change acceptance filter config 
     *  during a running system. All messages accepted.
     * operating - Will filter messages to only accept messages in the Look-Up 
     *  Table RAM.
     */
    enum struct FilterMode {
        off         = 0b10, //2 = 0b10
        bypass      = 0b01, //1 = 0b01
        operating   = 0b00  //0 = 0b00
    };
    
    /**
     * Since each CAN controller can have its own filter, we must specify which
     * controller we want to use. The three MSB of the address are used to 
     * identify the controller.
     */
    enum struct CANController {
        CAN1 = 0b000,
        CAN2 = 0b001
    };
    
    /**
     * Set the AFMR register to one of the filtering modes.
     */
    void setFilterMode(FilterMode mode);
    
    /**
     * Reset the filter. Allow all messages to come in, delete old filters.
     */
    void resetFilter();
}
#endif