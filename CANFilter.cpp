#include "CANFilter.h"

namespace CANFilter
{
    /* Private variables/functions */
    namespace
    {
        //Have to keep track of how many IDs we have
        unsigned short stdCANCount = 0;     //Word Size: 1/2
        unsigned short stdGrpCANCount = 0;  //Word Size: 1
        unsigned short extCANCount = 0;     //Word Size: 1
        unsigned short extGrpCANCount = 0;  //Word Size: 2

        /**
         * Calculate the start and end points of the memory sections. You must
         * take the size, in words * 4, that will be occupied by each section.
         * So, offset + (#words * 4) is the equation used for this.
         */
        void calculateAddresses()
        {
            //Set the Mode Register to bypass, allow writes to registers
            LPC_CANAF->AFMR = 1;

            //Beginning of the Standard (11-bit) CAN message filter
            LPC_CANAF->SFF_sa       = 0;
            //Beginning of the Standard (11-bit) Group CAN message filter
            LPC_CANAF->SFF_GRP_sa   = LPC_CANAF->SFF_sa + (((stdCANCount + 1) / 2) * 4);
            //Beginning of the Extended (29-bit) CAN message filter
            LPC_CANAF->EFF_sa       = LPC_CANAF->SFF_GRP_sa + (stdGrpCANCount * 4);
            //Beginning of the Extended (29-bit) Group CAN message filter
            LPC_CANAF->EFF_GRP_sa   = LPC_CANAF->EFF_sa + (extCANCount * 4);
            //End of the acceptance filter
            LPC_CANAF->ENDofTable   = LPC_CANAF->EFF_GRP_sa + ((extGrpCANCount * 2) * 4);

            //Set the Mode Register to operating, use the filter
            LPC_CANAF->AFMR = 0;
        }

        /**
         * Shift all of the items up in the filter by 1.
         */
        void upShiftFilter(unsigned short index) {
            //The starting data to be modified
            uint32_t bufferStart = LPC_CANAF_RAM->mask[index];
            //The next data to be modified
            uint32_t bufferNext;

            //Go through all the standard filters
            while(index < (LPC_CANAF->SFF_GRP_sa / 4)) {
                index++;
                //Save the current state of the memory we are modifying
                bufferNext = LPC_CANAF_RAM->mask[index];
                //[index+1] = [index+1] >> 16 | [index] << 16
                LPC_CANAF_RAM->mask[index] = (bufferNext >> 16) | (bufferStart << 16);
                //The next item is now the starting data we use
                bufferStart = bufferNext;
            }

            //Go through all other filters
            while(index < (LPC_CANAF->ENDofTable / 4)){
                index++;
                //Save the current state of the memory we are modifying
                bufferNext = LPC_CANAF_RAM->mask[index];
                //Move the previous value into the current address
                LPC_CANAF_RAM->mask[index] = bufferStart;
                //The next item is now the starting data we use
                bufferStart = bufferNext;
            }
        }

        /**
         * Shift only the standard filters up in the filter by 1.
         */
        void upShiftFilterStd(unsigned short index) {
            //The starting data to be modified
            uint32_t bufferStart = LPC_CANAF_RAM->mask[index];
            //The next data to be modified
            uint32_t bufferNext;

            //Go through all the standard filters but not passed it (since we
            //increment the index immediately, we include the -1)
            while(index < (LPC_CANAF->SFF_GRP_sa / 4) - 1) {
                index++;
                //Save the current state of the memory we are modifying
                bufferNext = LPC_CANAF_RAM->mask[index];
                //[index+1] = [index+1] >> 16 | [index] << 16
                LPC_CANAF_RAM->mask[index] = (bufferNext >> 16) | (bufferStart << 16);
                //The next item is now the starting data we use
                bufferStart = bufferNext;
            }
        }

        /**
         * Shift all of the items, not standard, down in the filter by 1. This
         * will delete the item at index.
         */
        void downShiftFilter(unsigned short index) {
            //Note, don't need a +1 on the while loop because we don't need to
            //overwrite the last item, just change how many items are in the
            //array. So if we put index = end, nothing happens.

            while(index < (LPC_CANAF->ENDofTable / 4)){
                index++;
                //Take the next item, and put it at index
                LPC_CANAF_RAM->mask[index-1] = LPC_CANAF_RAM->mask[index];
            }
        }

        /**
         * Shift only the standard filters down in the filter by 1. This will
         * delete the MSB at index.
         */
        void downShiftFilterStd(unsigned short index) {
            //Note, need a +1 so that the LSB of the end index are shifted to
            //the MSB. This will bring in "random" data to the filter. So, if
            //index = end, the LSB will be shifted to the MSB.
            while(index < (LPC_CANAF->SFF_GRP_sa / 4) + 1){
                index++;
                //[index] = [index] << 16 | [index+1] >> 16
                LPC_CANAF_RAM->mask[index-1] = (LPC_CANAF_RAM->mask[index-1] << 16) | (LPC_CANAF_RAM->mask[index] >> 16);
            }
        }

        /**
         * Sanitize the standard filters being input, so that any extra bits are removed
         * and the controller bits are placed correctly.
         */
        void sanitizeStdMask(CANController SCC, uint32_t & mask) {
            //Only have 11 bits for the mask, remove any possible extra
            mask &= 0x000007FF;
            //The controller bits begin at bit 13
            mask |= (static_cast<int>(SCC) << 13);
        }

        /**
         * Sanitize the extended filters being input, so that any extra bits are removed
         * and the controller bits are placed correctly.
         */
        void sanitizeExtMask(CANController SCC, uint32_t & mask) {
            //Only have 29 bits for the mask, remove any possible extra
            mask &= 0x1FFFFFFF;
            //The controller bits begin at bit 29
            mask |= (static_cast<int>(SCC) << 29);
        }
    }

    void setFilterMode(FilterMode mode)
    {
        LPC_CANAF->AFMR = static_cast<int>(mode);
    }

    void resetFilter()
    {
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

    int insertStandardFilter(CANController SCC, uint32_t mask) {
        //Make sure the table isn't full
        if((LPC_CANAF->ENDofTable / 4) >= 512)
            return -1;

        //Sanitize inputs
        sanitizeStdMask(SCC, mask);

        return 0;
    }
    int insertStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Make sure the table isn't full
        if((LPC_CANAF->ENDofTable / 4) >= 512)
            return -1;

        //Sanitize inputs
        sanitizeStdMask(SCC, start);
        sanitizeStdMask(SCC, end);

        return 0;
    }
    int insertExtendedFilter(CANController SCC, uint32_t mask) {
        //Make sure the table isn't full
        if((LPC_CANAF->ENDofTable / 4) >= 512)
            return -1;

        //Sanitize inputs
        sanitizeExtMask(SCC, mask);

        return 0;
    }
    int insertExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Make sure the table isn't full, with 2 open spaces
        if((LPC_CANAF->ENDofTable / 4) + 1 >= 512)
            return -1;

        //Sanitize inputs
        sanitizeExtMask(SCC, start);
        sanitizeExtMask(SCC, end);

        return 0;
    }

    int updateStandardFilter(CANController SCC, uint32_t mask) {
        //Sanitize inputs
        sanitizeStdMask(SCC, mask);

        return 0;
    }
    int updateStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Sanitize inputs
        sanitizeStdMask(SCC, start);
        sanitizeStdMask(SCC, end);

        return 0;
    }
    int updateExtendedFilter(CANController SCC, uint32_t mask) {
        //Sanitize inputs
        sanitizeExtMask(SCC, mask);

        return 0;
    }
    int updateExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Sanitize inputs
        sanitizeExtMask(SCC, start);
        sanitizeExtMask(SCC, end);

        return 0;
    }

    int deleteStandardFilter(CANController SCC, uint32_t mask) {
        //Make sure the table isn't empty
        if(LPC_CANAF->ENDofTable == 0)
            return -1;

        //Sanitize inputs
        sanitizeStdMask(SCC, mask);

        return 0;
    }
    int deleteStandardGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Make sure the table isn't empty
        if(LPC_CANAF->ENDofTable == 0)
            return -1;

        //Sanitize inputs
        sanitizeStdMask(SCC, start);
        sanitizeStdMask(SCC, end);

        return 0;
    }
    int deleteExtendedFilter(CANController SCC, uint32_t mask) {
        //Make sure the table isn't empty
        if(LPC_CANAF->ENDofTable == 0)
            return -1;

        //Sanitize inputs
        sanitizeExtMask(SCC, mask);

        return 0;
    }
    int deleteExtendedGroupFilter(CANController SCC, uint32_t start, uint32_t end) {
        //Make sure the table isn't empty
        if(LPC_CANAF->ENDofTable == 0)
            return -1;

        //Sanitize inputs
        sanitizeExtMask(SCC, start);
        sanitizeExtMask(SCC, end);

        return 0;
    }
}
