#include "basic/basic.h"
#include "basic/config.h"
#include "core/ssp/ssp.h"
#include "enc28j60.h"


/******************************************************************************/
/** \file enc28j60.c
 *  \brief Driver code for enc28j60.
 *  \author Iain Derrington (www.kandi-electronics.com)
 *  \date  0.1 20/06/07 First Draft \n
 *         0.2 11/07/07 Removed CS check macros. Fixed bug in writePhy
 *         0.3 12/07/07 Altered for uIP 1.0 
 *         *   2011-12-26 <mh at sponc dot de> adapted for r0ket
 */
/*******************************************************************************/



// define private variables

/** MAC address. Should  be set using a function.*/
//const uint8_t bytMacAddress[6] = {0x00,0xa0,0xc9,0x14,0xc8,0x29};
const uint8_t bytMacAddress[6] = {0,'C','C','_','C','P'};
TXSTATUS TxStatus;
uint8_t ether_buf[ETHER_BUFSIZE];
uint16_t ether_len;

// define private functions
static uint8_t ReadETHReg(uint8_t);         // read a ETX reg
static uint8_t ReadMacReg(uint8_t);         // read a MAC reg
static uint16_t ReadPhyReg(uint8_t);         // read a PHY reg
static uint16_t ReadMacBuffer(uint8_t * ,uint8_t);    //read the mac buffer (ptrBuffer, no. of bytes)
static uint8_t WriteCtrReg(uint8_t,uint8_t);               // write to control reg
static uint8_t ReadCtrReg(uint8_t bytAddress);
static uint8_t WritePhyReg(uint8_t,uint16_t);               // write to a phy reg
static uint16_t WriteMacBuffer(uint8_t *,uint16_t);    // write to mac buffer
static void ResetMac(void);

static uint8_t SetBitField(uint8_t, uint8_t);
static uint8_t ClrBitField(uint8_t, uint8_t);
static void BankSel(uint8_t);

#define SPI_CS RB_SPI_SS2


/** MACRO for selecting or deselecting chip select for the ENC28J60. Some HW dependancy.*/
#define SEL_MAC(x)    do {delayms(1); gpioSetValue(SPI_CS, 1-x);} while(0)
/** MACRO for rev B5 fix.*/
#define ERRATAFIX   do {SetBitField(ECON1, ECON1_TXRST); \
    ClrBitField(ECON1, ECON1_TXRST); \
    ClrBitField(EIR, EIR_TXERIF | EIR_TXIF);} while (0)

#define spi_send(buf, len) sspSend(0, buf, len)
#define spi_recv(buf, len) sspReceive(0, buf, len)

/***********************************************************************/
/** \brief Initialise the MAC.
 *
 * Description: \n
 * a) Setup SPI device. Assume Reb B5 for sub 8MHz operation \n
 * b) Setup buffer ptrs to devide memory in In and Out mem \n
 * c) Setup receive filters (accept only unicast).\n
 * d) Setup MACON registers (MAC control registers)\n
 * e) Setup MAC address
 * f) Setup Phy registers
 * \author Iain Derrington
 * \date 0.1 20/06/07 First draft
 */
/**********************************************************************/
void initMAC(void)
{
    gpioSetDir(SPI_CS, gpioDirection_Output);

    ResetMac();       // erm. Resets the MAC.

    // setup memory by defining ERXST and ERXND
    BankSel(0);       // select bank 0
    WriteCtrReg(ERXSTL,(uint8_t)( RXSTART & 0x00ff));    
    WriteCtrReg(ERXSTH,(uint8_t)((RXSTART & 0xff00)>> 8));
    WriteCtrReg(ERXNDL,(uint8_t)( RXEND   & 0x00ff));
    WriteCtrReg(ERXNDH,(uint8_t)((RXEND   & 0xff00)>>8));

    // Make sure Rx Read ptr is at the start of Rx segment
    WriteCtrReg(ERXRDPTL, (uint8_t)( RXSTART & 0x00ff));
    WriteCtrReg(ERXRDPTH, (uint8_t)((RXSTART & 0xff00)>> 8));

    BankSel(1);                             // select bank 1
    //WriteCtrReg(ERXFCON,( ERXFCON_UCEN + ERXFCON_CRCEN + ERXFCON_BCEN));
    WriteCtrReg(ERXFCON,( 0)); // dont filter for now




    // Initialise the MAC registers
    BankSel(2);                             // select bank 2
    SetBitField(MACON1, MACON1_MARXEN);     // Enable reception of frames
    WriteCtrReg(MACLCON2, 63);
    WriteCtrReg(MACON3, MACON3_FRMLNEN +    // Type / len field will be checked
            MACON3_TXCRCEN +    // MAC will append valid CRC
            MACON3_PADCFG0);    // All small packets will be padded


    SetBitField(MACON4, MACON4_DEFER);      
    WriteCtrReg(MAMXFLL, (uint8_t)( MAXFRAMELEN & 0x00ff));     // set max frame len
    WriteCtrReg(MAMXFLH, (uint8_t)((MAXFRAMELEN & 0xff00)>>8));
    WriteCtrReg(MABBIPG, 0x12);             // back to back interpacket gap. set as per data sheet
    WriteCtrReg(MAIPGL , 0x12);             // non back to back interpacket gap. set as per data sheet
    WriteCtrReg(MAIPGH , 0x0C);

    //Program our MAC address
    BankSel(3);              
    WriteCtrReg(MAADR1,bytMacAddress[0]);   
    WriteCtrReg(MAADR2,bytMacAddress[1]);  
    WriteCtrReg(MAADR3,bytMacAddress[2]);
    WriteCtrReg(MAADR4,bytMacAddress[3]);
    WriteCtrReg(MAADR5,bytMacAddress[4]);
    WriteCtrReg(MAADR6,bytMacAddress[5]);

    //test = ReadCtrReg(MAADR6);
    //lcdPrintIntHex(test); lcdDisplay(); delayms(1000);


    // Initialise the PHY registes
    WritePhyReg(PHCON1, 0x000);

    WritePhyReg(PHCON2, PHCON2_HDLDIS);
    WriteCtrReg(ECON1,  ECON1_RXEN);     //Enable the chip for reception of packets

}

/***********************************************************************/
/** \brief Writes a packet to the ENC28J60.
 *
 * Description: Writes ui_len bytes of data from ptrBufffer into ENC28J60.
 *              puts the necessary padding around the packet to make it a legit
 MAC packet.\n \n 
 1) Program ETXST.   \n
 2) Write per packet control byte.\n
 3) Program ETXND.\n
 4) Set ECON1.TXRTS.\n
 5) Check ESTAT.TXABRT. \n

 * \author Iain Derrington
 * \param ptrBuffer ptr to byte buffer. 
 * \param ui_Len Number of bytes to write from buffer. 
 * \return uint True or false. 
 */
/**********************************************************************/
uint16_t MACWrite()
{
    volatile uint16_t address = TXSTART;
    uint8_t  bytControl;

    bytControl = 0x00;

    BankSel(0);                                          // select bank 0
    WriteCtrReg(ETXSTL,(uint8_t)( TXSTART & 0x00ff));        // write ptr to start of Tx packet
    WriteCtrReg(ETXSTH,(uint8_t)((TXSTART & 0xff00)>>8));

    WriteCtrReg(EWRPTL,(uint8_t)( TXSTART & 0x00ff));        // Set write buffer to point to start of Tx Buffer
    WriteCtrReg(EWRPTH,(uint8_t)((TXSTART & 0xff00)>>8));

    WriteMacBuffer(&bytControl,1);                       // write per packet control byte
    address++;

    address+=WriteMacBuffer(ether_buf, ether_len);

    WriteCtrReg(ETXNDL, (uint8_t)( address & 0x00ff));       // Tell MAC when the end of the packet is
    WriteCtrReg(ETXNDH, (uint8_t)((address & 0xff00)>>8));


    ClrBitField(EIR,EIR_TXIF);
    SetBitField(EIE, EIE_TXIE |EIE_INTIE);

    ERRATAFIX;    
    SetBitField(ECON1, ECON1_TXRTS);                     // begin transmitting;

    do
    {      
    }while (!(ReadETHReg(EIR) & (EIR_TXIF)));             // kill some time. Note: Nice place to block?             // kill some time. Note: Nice place to block?

    ClrBitField(ECON1, ECON1_TXRTS);

    BankSel(0);                                         // read tx status bytes
    address++;                                          // increment ptr to address to start of status struc
    WriteCtrReg(ERDPTL, (uint8_t)( address & 0x00ff));      // Setup the buffer read ptr to read status struc
    WriteCtrReg(ERDPTH, (uint8_t)((address & 0xff00)>>8));
    ReadMacBuffer(&TxStatus.v[0],7);

    if (ReadETHReg(ESTAT) & ESTAT_TXABRT)                // did transmission get interrupted?
    {
        if (TxStatus.bits.LateCollision)
        {
            ClrBitField(ECON1, ECON1_TXRTS);
            SetBitField(ECON1, ECON1_TXRTS);

            ClrBitField(ESTAT,ESTAT_TXABRT | ESTAT_LATECOL);
        }
        ClrBitField(EIR, EIR_TXERIF | EIR_TXIF);
        ClrBitField(ESTAT,ESTAT_TXABRT);

        return FALSE;                                          // packet transmit failed. Inform calling function
    }                                                        // calling function may inquire why packet failed by calling [TO DO] function
    else
    {
        return TRUE;                                           // all fan dabby dozy
    }
}

/***********************************************************************/
/** \brief Tries to read a packet from the ENC28J60. 
 *
 * Description: If a valid packet is available in the ENC28J60 this function reads the packet into a
 *              buffer. The memory within the ENC28J60 will then be released. This version of the
 driver does not use interrupts so this function needs to be polled.\n \n
 * 
 * 1) Read packet count register. If >0 then continue else return. \n
 * 2) Read the current ERXRDPTR value. \n           
 * 3) Write this value into ERDPT.     \n
 * 4) First two bytes contain the ptr to the start of next packet. Read this value in. \n
 * 5) Calculate length of packet. \n
 * 6) Read in status byte into private variable. \n
 * 7) Read in packet and place into buffer. \n
 * 8) Free up memory in the ENC. \n
 *
 * \author Iain Derrington
 * \param ptrBuffer ptr to buffer of bytes where the packet should be read into. 
 * \return uint16_t, the number of complete packets in the buffer -1.

*/
/**********************************************************************/
uint16_t MACRead()
{
    static uint16_t nextpckptr = RXSTART;
    volatile RXSTATUS ptrRxStatus;
    volatile uint8_t bytPacket;

    BankSel(1);

    bytPacket = ReadETHReg(EPKTCNT);          // How many packets have been received

    if(bytPacket == 0)
        return bytPacket;                       // No full packets received

    BankSel(0);

    WriteCtrReg(ERDPTL,(uint8_t)( nextpckptr & 0x00ff));   //write this value to read buffer ptr
    WriteCtrReg(ERDPTH,(uint8_t)((nextpckptr & 0xff00)>>8));

    ReadMacBuffer((uint8_t*)&ptrRxStatus.v[0],6);             // read next packet ptr + 4 status bytes
    nextpckptr = ptrRxStatus.bits.NextPacket;

    ether_len=ptrRxStatus.bits.ByteCount;
    ReadMacBuffer(ether_buf,ether_len);   // read packet into buffer

    // ptrBuffer should now contain a MAC packet
    BankSel(0);
    WriteCtrReg(ERXRDPTL,ptrRxStatus.v[0]);  // free up ENC memory my adjustng the Rx Read ptr
    WriteCtrReg(ERXRDPTH,ptrRxStatus.v[1]);

    // decrement packet counter
    SetBitField(ECON2, ECON2_PKTDEC);


    return ether_len;
}

/*------------------------Private Functions-----------------------------*/

/***********************************************************************/
/** \brief ReadETHReg.
 *
 * Description: Reads contents of the addressed ETH reg over SPI bus. Assumes correct bank selected.
 *              
 *              
 * \author Iain Derrington
 * \param bytAddress Address of register to be read
 * \return byte Value of register.
 */
/**********************************************************************/
static uint8_t ReadETHReg(uint8_t bytAddress)
{
    uint8_t bytData;

#ifdef EXTRA_SAFE
    if (bytAddress > 0x1F)    
        return FALSE;                 // address invalid, [TO DO] 
#endif

    SEL_MAC(TRUE);                 // ENC CS low
    spi_send(&bytAddress,1);       // write opcode
    spi_recv(&bytData, 1);          // read value
    SEL_MAC(FALSE);

    return bytData;

}

/***********************************************************************/
/** \brief ReadMacReg.
 *
 * Description: Read contents of addressed MAC register over SPI bus. Assumes correct bank selected.
 *                    
 * \author Iain Derrington
 * \param bytAddress Address of register to read.
 * \return byte Contens of register just read.
 */
/**********************************************************************/
static uint8_t ReadMacReg(uint8_t bytAddress)
{
    uint8_t bytData;

#ifdef EXTRA_SAFE
    if (bytAddress > 0x1F)    
        return FALSE;                 // address invalid, [TO DO] 
#endif

    SEL_MAC(TRUE);                 // ENC CS low

    spi_send(&bytAddress,1);    // write opcode
    spi_recv(&bytData, 1);       // read dummy byte
    spi_recv(&bytData, 1);       // read value
    SEL_MAC(FALSE);


    return bytData;
}

/***********************************************************************/
/** \brief Write to phy Reg. 
 *
 * Description:  Writing to PHY registers is different to writing the other regeisters in that
 the registers can not be accessed directly. This function wraps up the requirements
 for writing to the PHY reg. \n \n

 1) Write address of phy reg to MIREGADR. \n
 2) Write lower 8 bits of data to MIWRL. \n
 3) Write upper 8 bits of data to MIWRL. \n \n            
 *              
 *              
 * \author Iain Derrington
 * \param address
 * \param data
 * \return byte
 */
/**********************************************************************/
static uint8_t WritePhyReg(uint8_t address, uint16_t data)
{ 
#ifdef EXTRA_SAFE
    if (address > 0x14)
        return FALSE;
#endif

    BankSel(2);

    WriteCtrReg(MIREGADR,address);        // Write address of Phy reg 
    WriteCtrReg(MIWRL,(uint8_t)data);        // lower phy data 
    WriteCtrReg(MIWRH,((uint8_t)(data >>8)));    // Upper phydata

    return TRUE;
}

/***********************************************************************/
/** \brief Read from PHY reg.
 *
 * Description: No direct access allowed to phy registers so the folling process must take place. \n \n
 *              1) Write address of phy reg to read from into MIREGADR. \n
 *              2) Set MICMD.MIIRD bit and wait 10.4uS. \n
 *              3) Clear MICMD.MIIRD bit. \n
 *              4) Read data from MIRDL and MIRDH reg. \n
 * \author Iain Derrington
 * \param address
 * \return uint
 */
/**********************************************************************/
static uint16_t ReadPhyReg(uint8_t address)
{
    volatile uint16_t uiData;
    volatile uint8_t bytStat;

    BankSel(2);
    WriteCtrReg(MIREGADR,address);    // Write address of phy register to read
    SetBitField(MICMD, MICMD_MIIRD);  // Set rd bit
    do                                  
    {
        bytStat = ReadMacReg(MISTAT);
    }while(bytStat & MISTAT_BUSY);

    ClrBitField(MICMD,MICMD_MIIRD);   // Clear rd bit
    uiData = (uint16_t)ReadMacReg(MIRDL);       // Read low data byte
    uiData |=((uint16_t)ReadMacReg(MIRDH)<<8); // Read high data byte

    return uiData;
}

/***********************************************************************/
/** \brief Write to a control reg .
 *
 * Description: Writes a byte to the address register. Assumes that correct bank has
 *              all ready been selected
 *              
 * \author Iain Derrington
 * \param bytAddress Address of register to be written to. 
 * \param bytData    Data to be written. 
 * \returns byte
 */
/**********************************************************************/
static uint8_t WriteCtrReg(uint8_t bytAddress,uint8_t bytData)
{
#ifdef EXTRA_SAFE
    if (bytAddress > 0x1f)
    {
        return FALSE;
    }
#endif

    bytAddress |= WCR_OP;       // Set opcode
    SEL_MAC(TRUE);              // ENC CS low
    spi_send(&bytAddress,1);    // Tx opcode and address
    spi_send(&bytData,1);       // Tx data
    SEL_MAC(FALSE);

    return TRUE;
}
static uint8_t ReadCtrReg(uint8_t bytAddress) {
    uint8_t data;
    SEL_MAC(TRUE);
    spi_send(&bytAddress,1);    // Tx opcode and address
    spi_recv(&data, 1);
    SEL_MAC(FALSE);
    return data;
}

/***********************************************************************/
/** \brief Read bytes from MAC data buffer.
 *
 * Description: Reads a number of bytes from the ENC28J60 internal memory. Assumes auto increment
 *              is on. 
 *              
 * \author Iain Derrington
 * \param bytBuffer  Buffer to place data in. 
 * \param byt_length Number of bytes to read.
 * \return uint  Number of bytes read.
 */
/**********************************************************************/
static uint16_t ReadMacBuffer(uint8_t * bytBuffer,uint8_t byt_length)
{
    uint8_t bytOpcode;

    bytOpcode = RBM_OP;
    SEL_MAC(TRUE);            // ENC CS low

    spi_send(&bytOpcode,1);   // Tx opcode 
    spi_recv(bytBuffer, byt_length);   // read bytes into buffer
    SEL_MAC(FALSE);           // release CS


    return byt_length;

}
/***********************************************************************/
/** \brief Write bytes to MAC data buffer.
 *
 * Description: Reads a number of bytes from the ENC28J60 internal memory. Assumes auto increment
 *              is on.
 *             
 * \author Iain Derrington
 * \param bytBuffer
 * \param ui_len
 * \return uint
 * \date WIP
 */
/**********************************************************************/
static uint16_t WriteMacBuffer(uint8_t * bytBuffer,uint16_t ui_len)
{
    uint8_t bytOpcode;

    bytOpcode = WBM_OP;
    SEL_MAC(TRUE);            // ENC CS low

    spi_send(&bytOpcode,1);   // Tx opcode 
    spi_send(bytBuffer, ui_len);   // read bytes into buffer
    SEL_MAC(FALSE);           // release CS


    return ui_len;

}

/***********************************************************************/
/** \brief Set bit field. 
 *
 * Description: Sets the bit/s at the address register. Assumes correct bank has been selected.
 *                           
 * \author Iain Derrington
 * \param bytAddress Address of registed where bit is to be set
 * \param bytData    Sets all the bits high.
 * \return byte      True or false
 */
/**********************************************************************/
static uint8_t SetBitField(uint8_t bytAddress, uint8_t bytData)
{
#ifdef EXTRA_SAFE
    if (bytAddress > 0x1f)
    {
        return FALSE;
    }
#endif

    bytAddress |= BFS_OP;       // Set opcode
    SEL_MAC(TRUE);              // ENC CS low

    spi_send(&bytAddress,1);    // Tx opcode and address
    spi_send(&bytData,1);       // Tx data
    SEL_MAC(FALSE);

    return TRUE;
}

/***********************************************************************/
/** \brief Clear bit field on ctr registers.
 *
 * Description: Sets the bit/s at the address register. Assumes correct bank has been selected.
 *             
 * \author Iain Derrington
 * \param bytAddress Address of registed where bit is to be set
 * \param bytData    Sets all the bits high.
 * \return byte      True or false
 */
/**********************************************************************/
static uint8_t ClrBitField(uint8_t bytAddress, uint8_t bytData)
{
#ifdef EXTRA_SAFE
    if (bytAddress > 0x1f)
    {
        return FALSE;
    }
#endif

    bytAddress |= BFC_OP;       // Set opcode
    SEL_MAC(TRUE);              // ENC CS low

    spi_send(&bytAddress,1);    // Tx opcode and address
    spi_send(&bytData,1);       // Tx data
    SEL_MAC(FALSE);

    return TRUE;
}

/***********************************************************************/
/** \brief Bank Select.
 *
 * Description: Select the required bank within the ENC28J60
 *              
 *              
 * \author Iain Derrington
 * \param bank Value between 0 and 3.
 * \date 0.1 09/06/07 First draft
 */
/**********************************************************************/
static void BankSel(uint8_t bank)
{
    volatile uint8_t temp;

#ifdef EXTRA_SAFE
    if (bank >3)
        return;
#endif

    temp = ReadETHReg(ECON1);       // Read ECON1 register
    temp &= ~ECON1_BSEL;            // mask off the BSEL bits
    temp |= bank;                   // set BSEL bits
    WriteCtrReg(ECON1, temp);       // write new values back to ENC28J60
}
/***********************************************************************/
/** \brief ResetMac.
 *
 * Description: Sends command to reset the MAC.
 *              
 *              
 * \author Iain Derrington
 */
/**********************************************************************/
static void ResetMac(void)
{
    uint8_t bytOpcode = RESET_OP;
    SEL_MAC(TRUE);              // ENC CS low

    spi_send(&bytOpcode,1);     // Tx opcode and address
    SEL_MAC(FALSE);

}


