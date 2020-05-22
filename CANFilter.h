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

    /**
     * Insert a standard filter to the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be whitelisted
     */
    int insertStandardFilter(CANController SCC, uint32_t mask);
    /**
     * Insert a standard group filter to the CAN acceptance filter. Will trigger
     * the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be whitelisted
     */
    int insertStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end);
    /**
     * Insert a extended filter to the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be whitelisted
     */
    int insertExtendedFilter(CANController SCC, uint32_t mask);
    /**
     * Insert a extended group filter to the CAN acceptance filter. Will trigger
     * the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be whitelisted
     */
    int insertExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end);

    /**
     * Update a standard filter to the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be updated
     */
    int updateStandardFilter(CANController SCC, uint32_t mask);
    /**
     * Update a standard group filter to the CAN acceptance filter. Will trigger
     * the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be updated
     */
    int updateStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end);
    /**
     * Update a extended filter to the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be updated
     */
    int updateExtendedFilter(CANController SCC, uint32_t mask);
    /**
     * Update a extended group filter to the CAN acceptance filter. Will trigger
     * the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be updated
     */
    int updateExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end);

    /**
     * Delete a standard filter from the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be deleted
     */
    int deleteStandardFilter(CANController SCC, uint32_t mask);
    /**
     * Delete a standard group filter from the CAN acceptance filter. Will
     * trigger the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be deleted
     */
    int deleteStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end);
    /**
     * Delete a extended filter from the CAN acceptance filter. Will trigger the
     * use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be deleted
     */
    int deleteExtendedFilter(CANController SCC, uint32_t mask);
    /**
     * Delete a extended group filter from the CAN acceptance filter. Will
     * trigger the use of the acceptance filter.
     * @param SCC Which CAN controller will be affected by this filter
     * @param mask The filter id that will be deleted
     */
    int deleteExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end);
}
#endif
