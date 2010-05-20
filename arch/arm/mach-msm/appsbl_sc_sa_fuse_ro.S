//------------------------------------------------------------------------------
// QUALCOMM Proprietary
// Copyright (c) 2004-2009, QUALCOMM Incorporated. All rights reserved.
//
// All data and information contained in or disclosed by this document are
// confidential and proprietary information of QUALCOMM Incorporated, and
// all rights therein are expressly reserved. By accepting this material,
// the recipient agrees that this material and the information contained
// therein are held in confidence and in trust and will not be used,
// copied, reproduced in whole or in part, nor its contents revealed in
// any manner to others without the express written permission of QUALCOMM
// Incorporated.
//
// This technology was exported from the United States in accordance with
// the Export Administration Regulations. Diversion contrary to U.S. law
// prohibited.
// -----------------------------------------------------------------------------
// FileName    : sc_sa_fuse.s
// Description : Sets L1/L2 sense amp settings determined by the fuses blown.
//               These fuses can not be read by Scorpion so they are read by
//               osbl_hw_init.c and put into the shared reg TCSR_SPARE2.
// TCSR_SPARE2 format
//    * 15:12   Signature (A)     /* no longer needed */
//    * 11:9    Reserved
//    * 8       L1 ACC_2 Fuse     /* Absolute bit 123 */
//    * 7:4     Speed Bin Fuse    0000  = 998 MHz default for legacy parts
//                                0001  = 1267 MHz
//                                0011  = 998 MHz
//                                0111  = 768 MHz
//    * 3       L2 ACC_1 Fuse     /* Absolute bit 125 */
//    * 2       L1 ACC_1 Fuse     /* Absolute bit 82 */
//    * 1       L2 ACC_0 Fuse     /* Absolute bit 219 */
//    * 0       L1 ACC_0 Fuse     /* Absolute bit 137 */
//  Author    : Kedar Athawale
//-----------------------------------------------------------------------------
//
//                           EDIT HISTORY FOR FILE
//
// This section contains comments describing changes made to the module.
// Notice that changes are listed in reverse chronological order.
//
// $Header: //source/qcom/qct/core/boot/romboot/appsbl/rel/2H08/shared/src/appsbl_sc_sa_fuse.s#1 $
//
// when       who     what, where, why
// --------   ---     ---------------------------------------------------
// 05/01/09   kedar  Further changes to L1/L2 cache setting values
// 04/17/09   kedar  filename changed - previously appsbl_sc_sa_ro.s
// 04/07/09   kedar  Add Cache sense amp settings. Given by mike heilman.
// 04/07/09   kedar  Creation. 
//*====*====*====*====*====*====*====*====*====*====*====*====*====*====*


#include <linux/linkage.h>
#include <asm/assembler.h>
//#include "scorpion_sysini.h"


//======================================================================


//------------------------------------------------------------------------------
// Scorpion L2 Array Sense Amp settings.
// Determined during module test and blown to indicate the ACC setting
// on the L2 array that is required for proper operation of the Scorpion core.
//  SCORPION_L2_ACC_1;SCORPION_L2_ACC_0;
//  00 - ACC setting = 010102	
//  01 - ACC setting = 010102
//  10 - ACC setting = 010101	
//  11 - ACC setting = 212102

// Scorpion L1 Array Sense Amp settings.
// Determined during module test and blown to indicate the ACC setting
// on the L1 array that is required for proper operation of the Scorpion core.                                                                          SCORPION_L1_ACC_1;SCORPION_L1_ACC_0;
//SCORPION_L1_ACC_2;SCORPION_L1_ACC_1;SCORPION_L1_ACC_0;
//        000 - ACC setting = FC00
//        001 - ACC setting = FC00
//        010 - ACC setting = 7C00
//        011 - ACC setting = FC00	
//        100 - ACC setting = 3C00
//        101 - ACC setting = 0400
//        110 - ACC setting = 0C00
//        111 - ACC setting = 1C00
//
//-------------------------------------------------------------------------------
//=======================================================================
//
//                             MODULE DEFINES
//
//=======================================================================
// Raptor addresses
.equ TCSR_SPARE2, 0xA8700060

//=======================================================================
//
//                             MODULE IMPORTS
//
//=======================================================================

//=======================================================================
//
//                             MODULE EXPORTS
//
//=======================================================================


ENTRY(Set_SA)
//=======================================================================
//	Set_SA code to set cache settings based on ring oscillator/sense amps
//=======================================================================
//        AREA            Set_SA, CODE, READONLY
//        ARM
        LDR     r6, =Set_SA
        ADR     r7, Set_SA
        sub     r7, r7, r6   // r7: the offset of linked address and the runtime address

        // no stack at this point and any registers we use will be 0'd 
        // after we return
        LDR     r0, =TCSR_SPARE2
        LDR     r1, [r0]
        LDR     r0, = 0x010F
        AND     r2, r1, r0   // concerned with bits [8, 3:0]

        //----------------------------------------------------------------------
        // Fuse bits used to determine sense amp settings
        //----------------------------------------------------------------------

        LDR     r0, = 0x0105
        AND     r4, r2, r0    //  mask off all but L1 ACC2, L1 ACC1 and L1 ACC0
        //set to default of FC00
        LDR     r5, =PVR0F0_6bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_6bits    // point to PVR2F0
        add     r3, r3, r7

ck_0:
        // if L1_[2:0] == 000 then ACC setting = FC00
        LDR     r1, = 0x0
        CMP     r4, r1
        BNE     ck_1
		B       WRITE_L1_SA_SETTINGS
ck_1:
        // if L1_[2:0] == 001 then ACC setting = FC00
        LDR     r1, = 0x01
        CMP     r4, r1
        BNE     ck_2
        B       WRITE_L1_SA_SETTINGS
ck_2:
        // if L1_[2:0] == 010 then ACC setting = 7C00
        LDR     r1, = 0x04
        CMP     r4, r1
        BNE     ck_3
        LDR     r5, =PVR0F0_5bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_5bits    // point to PVR2F0
        add     r3, r3, r7
        B       WRITE_L1_SA_SETTINGS
ck_3:
        // if L1_[2:0] == 011 then ACC setting = FC00
        LDR     r1, = 0x05
        CMP     r4, r1
        BNE     ck_4
        LDR     r5, =PVR0F0_6bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_6bits    // point to PVR2F0
        add     r3, r3, r7
        B       WRITE_L1_SA_SETTINGS

ck_4:
        // if L1_[2:0] == 0100 then ACC setting = 3C00
        LDR     r1, = 0x0100
        CMP     r4, r1
        BNE     ck_5
        LDR     r5, =PVR0F0_4bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_4bits    // point to PVR2F0
        add     r3, r3, r7
        B       WRITE_L1_SA_SETTINGS

ck_5:
       // if L1_[2:0] == 0101 then ACC setting = 0400
        LDR     r1, = 0x0101
        CMP     r4, r1
        BNE     ck_6
        LDR     r5, =PVR0F0_1bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_1bits    // point to PVR2F0
        add     r3, r3, r7
        B       WRITE_L1_SA_SETTINGS
ck_6:
       // if L1_[2:0] == 0110 then ACC setting = 0C00
        LDR     r1, = 0x0104
        CMP     r4, r1
        BNE     ck_7
        LDR     r5, =PVR0F0_2bits    // point to PVR0F0
        add     r5, r5, r7
        LDR     r3, =PVR2F0_2bits    // point to PVR2F0
        add     r3, r3, r7
        B       WRITE_L1_SA_SETTINGS
ck_7:
       // if L1_[2:0] == 0111 then ACC setting = 1C00
        LDR     r1, = 0x0105
        CMP     r4, r1
        LDREQ   r5, =PVR0F0_3bits    // point to PVR0F0
        addeq   r5, r5, r7
        LDREQ   r3, =PVR2F0_3bits    // point to PVR2F0
        addeq   r3, r3, r7

WRITE_L1_SA_SETTINGS:

        LDR     r5, [r5]
        LDR     r3, [r3]

        //WCP15_PVR0F0   r5
        MCR     p15,0x0,r5,c15,c15,0   // write R5 to PVR0F0

        //WCP15_PVR2F0   r3
        MCR     p15,0x2,r3,c15,c15,0   // write R3 to PVR2F0

        AND     r4, r2, #0x000A    //  mask off all but L2 array SA settings
        LDR     r5, =HVT_010102    // point to L2VR3F1 setting
        add     r5, r5, r7
                       //it gets ovewritten if its one of the other two cases
        // if L2_1 and L2_0 == 0     ACC setting = 010102
        LDR     r1, = 0x0000
        CMP     r4, r1
        BEQ     WRITE_L2_SA_SETTINGS

       // if L2_1 = 0 & L2_0 = 1    ACC setting = 010102
        LDR     R1, = 0x0002
        CMP     r4, r1
        BEQ     WRITE_L2_SA_SETTINGS

        // if L2_1 = 1 & L2_0 = 0    ACC setting = 010101
        LDR     r5, =HVT_010101    // point to L2VR3F1 setting
        add     r5, r5, r7
        LDR     r1, = 0x0008
        CMP     r4, r1
        BEQ     WRITE_L2_SA_SETTINGS
        
       // else L2_1 = 1 & L2_0 = 1   ACC setting = 212102
        LDR     r5, =HVT_212102
        add     r5, r5, r7


WRITE_L2_SA_SETTINGS:
        //WCP15_L2VR3F1  r4
        LDR     r5, [r5]
        MCR     p15,0x3,r5,c15,c15,1      //write r4 to L2VR3F1

        LDR     r0, =0                   //make sure the registers we touched
        LDR     r1, =0                   //are cleared when we return
        LDR     r2, =0
        LDR     r3, =0
        LDR     r4, =0
        LDR     r5, =0

        // routine complete
        BX      LR

// L1 SA settings according to LVT speed
PVR0F0_0bits:
        .word     0x38000000 // PVR0F0
PVR2F0_0bits:
        .word     0x00000000 // PVR2F0  0 bits set

PVR0F0_1bits:
        .word     0x38000400 // PVR0F0
PVR2F0_1bits:
        .word     0x04000000 // PVR2F0  1 bits set

PVR0F0_2bits:
        .word     0x38000C00 // PVR0F0
PVR2F0_2bits:
        .word     0x0C000000 // PVR2F0  2 bits set

PVR0F0_3bits:
        .word     0x38001C00 // PVR0F0
PVR2F0_3bits:
        .word     0x1C000000 // PVR2F0  3 bits set

PVR0F0_4bits:
        .word     0x38003C00 // PVR0F0
PVR2F0_4bits:
        .word     0x3C000000 // PVR2F0  4 bits set

PVR0F0_5bits:
        .word     0x38007C00 // PVR0F0
PVR2F0_5bits:
        .word     0x7C000000 // PVR2F0  5 bits set

PVR0F0_6bits:
        .word     0x3800FC00 // PVR0F0
PVR2F0_6bits:
        .word     0xFC000000 // PVR2F0  6 bits set

// L2 SA settings according to HVT speed
HVT_212102:
        .word     0x00212102 // L2VR3F1

HVT_010102:
        .word     0x00010102 // L2VR3F1

HVT_010101:
        .word     0x00010101 // L2VR3F1


//        END
