//
//  conC_Encoder_initialize.c
//  
//
//  Created by JOSEPH L GARBINI on 12/28/17.
//

#include "conC_Encoder_initialize.h"
NiFpga_Status    conC_Encoder_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encCp, int iE) {
    
    //  Initialize either of the Connector-C encoders
    //        Inputs:    NiFpga_Session    myrio_session     -- session
    //                MyRio_Encoder *encCp            -- encoder definition structure
    //                int iE                            -- encoder number on connector-C: (0 or 1)
    //        Returns:    NiFpga_Status
    
    NiFpga_Status status;
    int    selShift;
    uint8_t selectReg;
    
    /* Initialize the encoder struct with registers from the FPGA personality. */
    switch (iE) {
        default:    // encoder #0
            encCp->cnfg = ENCC_0CNFG;
            encCp->stat = ENCC_0STAT;
            encCp->cntr = ENCC_0CNTR;
            selShift=0;
            break;
        case 1:        // encoder #1
            encCp->cnfg = ENCC_1CNFG;
            encCp->stat = ENCC_1STAT;
            encCp->cntr = ENCC_1CNTR;
            selShift=2;
            break;
    }
    
    /* Encoder inputs are on pins shared with other onboard devices. To input
     * from a physical pin, select the encoder on the appropriate SELECT
     * register.  Read the value of the SYSSELECTC register. */
    status = NiFpga_ReadU8(myrio_session, SYSSELECTC, &selectReg);
    MyRio_ReturnValueIfNotSuccess(status, status,
                                  "Could not read from the SYSSELECTC register!");
    
    /* Set bit 0 of the SYSSELECTC register to enable ENC0 functionality.
     * The functionality of these bits is specified in the documentation.*/
    // Note: Here bit-2 is on for encoder-1, and ORed with the selectReg
    // See page 10 of the Personality Reference
    selectReg = selectReg | (1 << selShift);
    
    /* Write the updated value of the SYSSELECTC register. */
    status = NiFpga_WriteU8(myrio_session, SYSSELECTC, selectReg);
    MyRio_ReturnValueIfNotSuccess(status, status,
                                  "Could not write to the SYSSELECTC register!")
    
    /* Enable the encoder and configure to read quadrature signals. */
    Encoder_Configure(    encCp,
                      Encoder_Enable | Encoder_SignalMode,
                      Encoder_Enabled | Encoder_QuadPhase);
    
    return status;
}
