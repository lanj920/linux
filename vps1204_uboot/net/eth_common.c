/*
 * (C) Copyright 2001-2015
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 * Joe Hershberger, National Instruments
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <miiphy.h>
#include <net.h>
#include "eth_internal.h"

void eth_parse_enetaddr(const char *addr, uchar *enetaddr)
{
	char *end;
	int i;

	for (i = 0; i < 6; ++i) {
		enetaddr[i] = addr ? simple_strtoul(addr, &end, 16) : 0;
		if (addr)
			addr = (*end) ? end + 1 : end;
	}
}

int eth_getenv_enetaddr(const char *name, uchar *enetaddr)
{
	eth_parse_enetaddr(getenv(name), enetaddr);
	return is_valid_ethaddr(enetaddr);
}

int eth_setenv_enetaddr(const char *name, const uchar *enetaddr)
{
	char buf[20];

	sprintf(buf, "%pM", enetaddr);

	return setenv(name, buf);
}

int eth_getenv_enetaddr_by_index(const char *base_name, int index,
				 uchar *enetaddr)
{
	char enetvar[32];
	sprintf(enetvar, index ? "%s%daddr" : "%saddr", base_name, index);
	return eth_getenv_enetaddr(enetvar, enetaddr);
}

int eth_setenv_enetaddr_by_index(const char *base_name, int index,
				 uchar *enetaddr)
{
	char enetvar[32];
	sprintf(enetvar, index ? "%s%daddr" : "%saddr", base_name, index);
	return eth_setenv_enetaddr(enetvar, enetaddr);
}

void eth_common_init(void)
{
	bootstage_mark(BOOTSTAGE_ID_NET_ETH_START);
#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII) || defined(CONFIG_PHYLIB)
	miiphy_init();
#endif

#ifdef CONFIG_PHYLIB
	phy_init();
#endif
}

int eth_mac_skip(int index)
{
	char enetvar[15];
	char *skip_state;

	sprintf(enetvar, index ? "eth%dmacskip" : "ethmacskip", index);
	skip_state = getenv(enetvar);
	return skip_state != NULL;
}

void eth_current_changed(void)
{
	char *act = getenv("ethact");
	char *ethrotate;

	/*
	 * The call to eth_get_dev() below has a side effect of rotating
	 * ethernet device if uc_priv->current == NULL. This is not what
	 * we want when 'ethrotate' variable is 'no'.
	 */
	ethrotate = getenv("ethrotate");
	if ((ethrotate != NULL) && (strcmp(ethrotate, "no") == 0))
		return;

	/* update current ethernet name */
	if (eth_get_dev()) {
		if (act == NULL || strcmp(act, eth_get_name()) != 0)
			setenv("ethact", eth_get_name());
	}
	/*
	 * remove the variable completely if there is no active
	 * interface
	 */
	else if (act != NULL)
		setenv("ethact", NULL);
}

void eth_try_another(int first_restart)
{
	static void *first_failed;
	char *ethrotate;

	/*
	 * Do not rotate between network interfaces when
	 * 'ethrotate' variable is set to 'no'.
	 */
	ethrotate = getenv("ethrotate");
	if ((ethrotate != NULL) && (strcmp(ethrotate, "no") == 0))
		return;

	if (!eth_get_dev())
		return;

	if (first_restart)
		first_failed = eth_get_dev();

	eth_set_current_to_next();

	eth_current_changed();

	if (first_failed == eth_get_dev())
		net_restart_wrap = 1;
}

void eth_set_current(void)
{
	static char *act;
	static int  env_changed_id;
	int	env_id;

	env_id = get_env_id();
	if ((act == NULL) || (env_changed_id != env_id)) {
		act = getenv("ethact");
		env_changed_id = env_id;
	}

	if (act == NULL) {
		char *ethprime = getenv("ethprime");
		void *dev = NULL;

		if (ethprime)
			dev = eth_get_dev_by_name(ethprime);
		if (dev)
			eth_set_dev(dev);
		else
			eth_set_dev(NULL);
	} else {
		eth_set_dev(eth_get_dev_by_name(act));
	}

	eth_current_changed();
}

const char *eth_get_name(void)
{
	return eth_get_dev() ? eth_get_dev()->name : "unknown";
}

/*---------------------------------------------------------------------*/
#define XSPIPS_CR_OFFSET	0x00  /**< Configuration */
#define XSPIPS_SR_OFFSET	0x04  /**< Interrupt Status */
#define XSPIPS_IER_OFFSET	0x08  /**< Interrupt Enable */
#define XSPIPS_IDR_OFFSET	0x0c  /**< Interrupt Disable */
#define XSPIPS_IMR_OFFSET	0x10  /**< Interrupt Enabled Mask */
#define XSPIPS_ER_OFFSET	0x14  /**< Enable/Disable Register */
#define XSPIPS_DR_OFFSET	0x18  /**< Delay Register */
#define XSPIPS_TXD_OFFSET	0x1C  /**< Data Transmit Register */
#define XSPIPS_RXD_OFFSET	0x20  /**< Data Receive Register */
#define XSPIPS_SICR_OFFSET	0x24  /**< Slave Idle Count */
#define XSPIPS_TXWR_OFFSET	0x28  /**< Transmit FIFO Watermark */
#define XSPIPS_RXWR_OFFSET	0x2C  /**< Receive FIFO Watermark */

#define XSPIPS_MASTER_OPTION		0x1  /**< Master mode option */
#define XSPIPS_CLK_ACTIVE_LOW_OPTION	0x2  /**< Active Low Clock option */
#define XSPIPS_CLK_PHASE_1_OPTION	0x4  /**< Clock Phase one option */
#define XSPIPS_DECODE_SSELECT_OPTION	0x8  /**< Select 16 slaves Option */
#define XSPIPS_FORCE_SSELECT_OPTION	0x10 /**< Force Slave Select */
#define XSPIPS_MANUAL_START_OPTION	0x20 /**< Manual Start mode option */

#define XSPIPS_CR_MODF_GEN_EN_MASK 0x00020000 /**< Modefail Generation
						 Enable */
#define XSPIPS_CR_MANSTRT_MASK   0x00010000 /**< Manual Transmission Start */
#define XSPIPS_CR_MANSTRTEN_MASK 0x00008000 /**< Manual Transmission Start
						 Enable */
#define XSPIPS_CR_SSFORCE_MASK   0x00004000 /**< Force Slave Select */
#define XSPIPS_CR_SSCTRL_MASK    0x00003C00 /**< Slave Select Decode */
#define XSPIPS_CR_SSCTRL_SHIFT   10	    /**< Slave Select Decode shift */
#define XSPIPS_CR_SSCTRL_MAXIMUM 0xF	    /**< Slave Select maximum value */
#define XSPIPS_CR_SSDECEN_MASK   0x00000200 /**< Slave Select Decode Enable */

#define XSPIPS_CR_PRESC_MASK     0x00000038 /**< Prescaler Setting */
#define XSPIPS_CR_PRESC_SHIFT    3	    /**< Prescaler shift */
#define XSPIPS_CR_PRESC_MAXIMUM  0x07	    /**< Prescaler maximum value */

#define XSPIPS_CR_CPHA_MASK      0x00000004 /**< Phase Configuration */
#define XSPIPS_CR_CPOL_MASK      0x00000002 /**< Polarity Configuration */

#define XSPIPS_CR_MSTREN_MASK    0x00000001 /**< Master Mode Enable */
#define XSPIPS_CR_RESET_STATE    0x00020000 /**< Mode Fail Generation Enable */

#define XSPIPS_IXR_TXUF_MASK		0x00000040  /**< Tx FIFO Underflow */
#define XSPIPS_IXR_RXFULL_MASK		0x00000020  /**< Rx FIFO Full */
#define XSPIPS_IXR_RXNEMPTY_MASK	0x00000010  /**< Rx FIFO Not Empty */
#define XSPIPS_IXR_TXFULL_MASK		0x00000008  /**< Tx FIFO Full */
#define XSPIPS_IXR_TXOW_MASK		0x00000004  /**< Tx FIFO Overwater */
#define XSPIPS_IXR_MODF_MASK		0x00000002  /**< Mode Fault */
#define XSPIPS_IXR_RXOVR_MASK		0x00000001  /**< Rx FIFO Overrun */
#define XSPIPS_IXR_DFLT_MASK		0x00000027  /**< Default interrupts
							 mask */
#define XSPIPS_IXR_WR_TO_CLR_MASK	0x00000043  /**< Interrupts which
							 need write to clear */
#define XSPIPS_ISR_RESET_STATE		0x04	    /**< Default to tx/rx
						       * reg empty */
#define XSPIPS_IXR_DISABLE_ALL_MASK	0x00000043  /**< Disable all
						       * interrupts */

#define XSPIPS_FIFO_DEPTH	128 /**< FIFO depth of Tx and Rx */

#define NREAD	(0x60)
#define NWRITE	(0x61)

#define SIO (0xF0)
#define STS (0xFE)
#define SPG (0xFF)

#define SPIF	(0x80)
#define RACK	(0x20)
#define RXRDY	(0x02)
#define TXRDY	(0x01)

#define XST_SUCCESS                     0L
#define XST_FAILURE                     1L

#define XSpiPs_WriteReg(BaseAddress, RegOffset, RegisterValue) \
	*(volatile u32 *) ((BaseAddress) + RegOffset) = RegisterValue

#define XSpiPs_ReadReg(BaseAddress, RegOffset) \
	(*(volatile u32 * )((BaseAddress) + (RegOffset)))


#define XSpiPs_Enable(InstancePtr)					\
	*(volatile u32 *)((InstancePtr->Config.BaseAddress) + XSPIPS_ER_OFFSET) = 1

#define XSpiPs_Disable(InstancePtr)					\
	*(volatile u32 *)((InstancePtr->Config.BaseAddress) + XSPIPS_ER_OFFSET) = 0

typedef void (*XSpiPs_StatusHandler) (void *CallBackRef, u32 StatusEvent,
					unsigned ByteCount);


typedef struct {
	u32 Option;
	u32 Mask;
} OptionsMap;

static OptionsMap OptionsTable[] = {
	{XSPIPS_MASTER_OPTION, XSPIPS_CR_MSTREN_MASK},
	{XSPIPS_CLK_ACTIVE_LOW_OPTION, XSPIPS_CR_CPOL_MASK},
	{XSPIPS_CLK_PHASE_1_OPTION, XSPIPS_CR_CPHA_MASK},
	{XSPIPS_DECODE_SSELECT_OPTION, XSPIPS_CR_SSDECEN_MASK},
	{XSPIPS_FORCE_SSELECT_OPTION, XSPIPS_CR_SSFORCE_MASK},
	{XSPIPS_MANUAL_START_OPTION, XSPIPS_CR_MANSTRTEN_MASK}
};

#define XSPIPS_NUM_OPTIONS	(sizeof(OptionsTable) / sizeof(OptionsMap))

typedef struct {
	u16 DeviceId;		/**< Unique ID  of device */
	u32 BaseAddress;	/**< Base address of the device */
	u32 InputClockHz;	/**< Input clock frequency */
} XSpiPs_Config;


typedef struct {
	XSpiPs_Config Config;	 /**< Configuration structure */
	u32 IsReady;		 /**< Device is initialized and ready */

	u8 *SendBufferPtr;	 /**< Buffer to send (state) */
	u8 *RecvBufferPtr;	 /**< Buffer to receive (state) */
	unsigned RequestedBytes; /**< Number of bytes to transfer (state) */
	unsigned RemainingBytes; /**< Number of bytes left to transfer(state) */
	u32 IsBusy;		 /**< A transfer is in progress (state) */
	u32 SlaveSelect;     /**< The slave select value when
					 XSPIPS_FORCE_SSELECT_OPTION is set */

	XSpiPs_StatusHandler StatusHandler;
	void *StatusRef;  	 /**< Callback reference for status handler */

} XSpiPs;

XSpiPs_Config XSpiPs_ConfigTable[] =
{
	{
		0,
		0xE0006000,
		166666672
	}
};


XSpiPs_Config *g_SpiConfig;

void Spi_SetOptions(XSpiPs *InstancePtr, u32 Options)
{
	u32 ConfigReg;
	unsigned int Index;
	u32 CurrentConfigReg;	

	/*
	 * Do not allow the slave select to change while a transfer is in
	 * progress. Not thread-safe.
	 */
	if (InstancePtr->IsBusy) {
		return;
	}

	ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress,
				 XSPIPS_CR_OFFSET);

	CurrentConfigReg = ConfigReg;

	/*
	 * Loop through the options table, turning the option on or off
	 * depending on whether the bit is set in the incoming options flag.
	 */
	for (Index = 0; Index < XSPIPS_NUM_OPTIONS; Index++) {
		if (Options & OptionsTable[Index].Option) {
			/* Turn it on */
			ConfigReg |= OptionsTable[Index].Mask;
		}
		else {
			/* Turn it off */
			ConfigReg &= ~(OptionsTable[Index].Mask);
		}
	}


	/*
	 * If CPOL-CPHA bits are toggled from previous state,
	 * disable before writing the configuration register and then enable.
	 */
	if( ((CurrentConfigReg & XSPIPS_CR_CPOL_MASK) !=
		(ConfigReg & XSPIPS_CR_CPOL_MASK)) ||
		((CurrentConfigReg & XSPIPS_CR_CPHA_MASK) !=
		(ConfigReg & XSPIPS_CR_CPHA_MASK)) ) {
			XSpiPs_WriteReg((InstancePtr->Config.BaseAddress), XSPIPS_ER_OFFSET, 0);
		}

	/*
	 * Now write the Config register. Leave it to the upper layers
	 * to restart the device.
	 */
	XSpiPs_WriteReg(InstancePtr->Config.BaseAddress,
				XSPIPS_CR_OFFSET, ConfigReg);

	/*
	 * Enable
	 */
	if( ((CurrentConfigReg & XSPIPS_CR_CPOL_MASK) !=
		(ConfigReg & XSPIPS_CR_CPOL_MASK)) ||
		((CurrentConfigReg & XSPIPS_CR_CPHA_MASK) !=
		(ConfigReg & XSPIPS_CR_CPHA_MASK)) ) {
			XSpiPs_WriteReg((InstancePtr->Config.BaseAddress), XSPIPS_ER_OFFSET, 1);
		}
}


u32 Spi_GetOptions(XSpiPs *InstancePtr)
{
	u32 OptionsFlag = 0;
	u32 ConfigReg;
	unsigned int Index;	

	/*
	 * Get the current options
	 */
	ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);

	/*
	 * Loop through the options table to grab options
	 */
	for (Index = 0; Index < XSPIPS_NUM_OPTIONS; Index++) {
		if (ConfigReg & OptionsTable[Index].Mask) {
			OptionsFlag |= OptionsTable[Index].Option;
		}
	}

	return OptionsFlag;
}

#define XSpiPs_IsMaster(InstancePtr) \
		((Spi_GetOptions(InstancePtr) & \
		  XSPIPS_MASTER_OPTION) ? 1 : 0)

#define XSpiPs_IsDecodeSSelect(InstancePtr) \
		((Spi_GetOptions(InstancePtr) & \
		  XSPIPS_DECODE_SSELECT_OPTION) ? 1 : 0)

#define XSpiPs_IsManualStart(InstancePtr) \
		((Spi_GetOptions(InstancePtr) & \
		  XSPIPS_MANUAL_START_OPTION) ? 1 : 0)

#define XSpiPs_IsManualChipSelect(InstancePtr) \
		((Spi_GetOptions(InstancePtr) & \
		  XSPIPS_FORCE_SSELECT_OPTION) ? 1 : 0)

#define XSpiPs_SendByte(BaseAddress, Data) \
		*(volatile u32 *)((BaseAddress) + XSPIPS_TXD_OFFSET) = Data;

#define XSpiPs_RecvByte(BaseAddress) \
		(u8)(*(volatile u32 * )((BaseAddress) + XSPIPS_RXD_OFFSET))

void XSpiPs_SetSlaveSelect(XSpiPs *InstancePtr, u8 SlaveSel)
{
	u32 ConfigReg;	

	/*
	 * Do not allow the slave select to change while a transfer is in
	 * progress. Not thread-safe.
	 */
	if (InstancePtr->IsBusy) {
		return;
	}
	/*
	 * If decode slave select option is set,
	 * then set slave select value directly.
	 * Update the Instance structure member.
	 */
	if ( XSpiPs_IsDecodeSSelect( InstancePtr ) ) {
		InstancePtr->SlaveSelect = SlaveSel << XSPIPS_CR_SSCTRL_SHIFT;
	}
	else {
	/*
	 * Set the bit position to low using SlaveSel. Update the Instance
	 * structure member.
	 */
		InstancePtr->SlaveSelect = ((~(1 << SlaveSel)) & \
			XSPIPS_CR_SSCTRL_MAXIMUM) << XSPIPS_CR_SSCTRL_SHIFT;
	}

	/*
	 * Read the config register, update the slave select value and write
	 * back to config register.
	 */
	ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);
	ConfigReg &= (~XSPIPS_CR_SSCTRL_MASK);
	ConfigReg |= InstancePtr->SlaveSelect;
	XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET, ConfigReg);

	
}

void XSpiPs_SetClkPrescaler(XSpiPs *InstancePtr, u8 Prescaler)
{
	u32 ConfigReg;	

	/*
	 * Do not allow the prescaler to be changed while a transfer is in
	 * progress. Not thread-safe.
	 */
	if (InstancePtr->IsBusy) {
		return;
	}

	/*
	 * Read the Config register, mask out the interesting bits, and set
	 * them with the shifted value passed into the function. Write the
	 * results back to the Config register.
	 */
	ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);

	ConfigReg &= ~XSPIPS_CR_PRESC_MASK;
	ConfigReg |= (u32) (Prescaler & XSPIPS_CR_PRESC_MAXIMUM) << XSPIPS_CR_PRESC_SHIFT;

	XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET, ConfigReg);	
}

int XSpiPs_PolledTransfer(XSpiPs *InstancePtr, u8 *SendBufPtr,
				u8 *RecvBufPtr, u32 ByteCount)
{
	u32 StatusReg;
	u32 ConfigReg;
	u32 TransCount;

	/*
	 * Check whether there is another transfer in progress. Not thread-safe.
	 */
	if (InstancePtr->IsBusy) {
		return 1;
	}

	/*
	 * Set the busy flag, which will be cleared when the transfer is
	 * entirely done.
	 */
	InstancePtr->IsBusy = 1;

	/*
	 * Set up buffer pointers.
	 */
	InstancePtr->SendBufferPtr = SendBufPtr;
	InstancePtr->RecvBufferPtr = RecvBufPtr;

	InstancePtr->RequestedBytes = ByteCount;
	InstancePtr->RemainingBytes = ByteCount;	
	/*
	 * If manual chip select mode, initialize the slave select value.
	 */
	if (XSpiPs_IsManualChipSelect(InstancePtr)) {
		ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);
		/*
		 * Set the slave select value.
		 */
		ConfigReg &= ~XSPIPS_CR_SSCTRL_MASK;
		ConfigReg |= InstancePtr->SlaveSelect;
		XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET, ConfigReg);
	}

	/*
	 * Enable the device.
	 */
	XSpiPs_Enable(InstancePtr);
	while((InstancePtr->RemainingBytes > 0) || (InstancePtr->RequestedBytes > 0)) {
		TransCount = 0;
		/*
		 * Fill the TXFIFO with as many bytes as it will take (or as
		 * many as we have to send).
		 */
		while ((InstancePtr->RemainingBytes > 0) && (TransCount < XSPIPS_FIFO_DEPTH)) {
			XSpiPs_SendByte(InstancePtr->Config.BaseAddress, *InstancePtr->SendBufferPtr);
			InstancePtr->SendBufferPtr++;
			InstancePtr->RemainingBytes--;
			++TransCount;
		}

		/*
		 * If master mode and manual start mode, issue manual start
		 * command to start the transfer.
		 */
		if (XSpiPs_IsManualStart(InstancePtr) && XSpiPs_IsMaster(InstancePtr)) {
			ConfigReg = XSpiPs_ReadReg( InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);
			ConfigReg |= XSPIPS_CR_MANSTRT_MASK;

			XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET, ConfigReg);
		}
		/*
		 * Wait for the transfer to finish by polling Tx fifo status.
		 */
		do {
			StatusReg = XSpiPs_ReadReg( InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
			if ( StatusReg & XSPIPS_IXR_MODF_MASK )
			{
				/*
				 * Clear the mode fail bit
				 */
				XSpiPs_WriteReg( InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET, XSPIPS_IXR_MODF_MASK);			
				return 1;
			}
		} while ((StatusReg & XSPIPS_IXR_TXOW_MASK) == 0);

		/*
		 * A transmit has just completed. Process received data and
		 * check for more data to transmit.
		 * First get the data received as a result of the transmit
		 * that just completed. Receive data based on the
		 * count obtained while filling tx fifo. Always get the
		 * received data, but only fill the receive buffer if it
		 * points to something (the upper layer software may not
		 * care to receive data).
		 */
		while (TransCount) {
			u8 TempData;
			TempData = XSpiPs_RecvByte(InstancePtr->Config.BaseAddress);
			if (InstancePtr->RecvBufferPtr != NULL) {
				*InstancePtr->RecvBufferPtr++ = (u8) TempData;
			}
			InstancePtr->RequestedBytes--;
			--TransCount;
		}
	}

	/*
	 * Clear the slave selects now, before terminating the transfer.
	 */
	if (XSpiPs_IsManualChipSelect(InstancePtr)) {
		ConfigReg = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET);
		ConfigReg |= XSPIPS_CR_SSCTRL_MASK;
		XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_CR_OFFSET, ConfigReg);
	}
	/*
	 * Clear the busy flag.
	 */
	InstancePtr->IsBusy = 0;

	/*
	 * Disable the device.
	 */
	XSpiPs_Disable(InstancePtr);

	return 0;
}

void delay_SL( u32 delayCount )
{
	do{
		__asm__("nop");
		delayCount--;
	}while(delayCount>0);
}

#if 0
int writeBCM5396( XSpiPs *Spi_ptr,u8 page, u8 offset, u8 *pBuffer )
{
	u8 data[20];
	u32 retVal;
	u32 u32SendNum, u32ReqRetNum;
	int i;

	for(i=0;i<20;i++)
		data[i]=i;

	// Set Page
	data[0] = NWRITE;
	data[1] = SPG;
	data[2] = page;
	u32SendNum = 3;
	u32ReqRetNum = 0;

	retVal = XSpiPs_PolledTransfer(Spi_ptr,data,NULL,u32SendNum);//pBuffer
	if( retVal != 0 )
	{
		printf("Call XSpiPs_PT_SL 1 Failed\n\r");
	}

	// Read STS
READ_STS_1:
	data[0] = NREAD;
	data[1] = STS;
	u32SendNum = 2;
	u32ReqRetNum = 1;
	retVal = XSpiPs_PolledTransfer(Spi_ptr,data,NULL,u32SendNum+u32ReqRetNum);

	if( retVal == 0 )
	{
		if((pBuffer[2] & SPIF)==0)//( workBuf[2] & SPIF )
		{			
			// Set Page
			data[0] = NWRITE;
			data[1] = SPG;
			data[2] = page;
			u32SendNum = 3;
			u32ReqRetNum = 0;
			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,NULL,u32SendNum+u32ReqRetNum);
			if( retVal != 0 )
			{
				printf("Call XSpiPs_PT_SL 1 Failed\n\r");
			}

			// Write Data
			data[0] = NWRITE;
			data[1] = offset;
			data[2] = pBuffer[0];
			data[3] = pBuffer[1];
			u32SendNum = 4;
			u32ReqRetNum = 0;//1;

			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,NULL,u32SendNum+u32ReqRetNum);
			if( retVal != 0 )
			{
				printf("Call XSpiPs_PT_SL 2 Failed\n\r");
			}

		}
		else
		{
			retVal = 1;
			printf( "writeBCM5396 Timeout 1 Occured!\n" );
			delay_SL(0x100000);

			// Set Page
			data[0] = NWRITE;
			data[1] = SPG;
			data[2] = page;
			u32SendNum = 3;
			u32ReqRetNum = 0;
			
			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
			if( retVal != 0 )
			{
				printf("Call XSpiPs_PT_SL 1 Failed\n\r");
			}

			goto READ_STS_1;
		}
	}
	else
		printf("Call XSpiPs_PT_SL 4 Failed\n");

	return retVal;
}

int readBCM5396(XSpiPs *Spi_ptr, u8 page, u8 offset, u8 *pBuffer )
{
	u8 data[8];
	u32 retVal;
	u32 u32SendNum, u32ReqRetNum;


	// Set Page
	data[0] = NWRITE;
	data[1] = SPG;
	data[2] = page;

	u32SendNum = 3;
	u32ReqRetNum = 0;

	retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum);
	if( retVal != XST_SUCCESS )
	{
		printf("Call XSpiPs_PT_SL 1 Failed\n\r");
	}
	
	// Read STS
READ_STS_1:
    data[0] = NREAD;
	data[1] = STS;
	u32SendNum = 2;
	u32ReqRetNum = 1;

	retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);

	if( retVal == XST_SUCCESS )
	{
		if( (pBuffer[2] & SPIF)==0 )
		{
			// Set Page
			data[0] = NWRITE;
			data[1] = SPG;
			data[2] = page;
			u32SendNum = 3;
			u32ReqRetNum = 0;

			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
			if( retVal != XST_SUCCESS )
			{
				printf("Call XSpiPs_PT_SL 1 Failed\n\r");
			}

			data[0] = NREAD;
			data[1] = offset;
			u32SendNum = 2;
			u32ReqRetNum = 1;

			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
			if( retVal != XST_SUCCESS )
			{
				printf("Call XSpiPs_PT_SL 2 Failed\n\r");
			}


			// Read STS
READ_STS_2:
			data[0] = NREAD;
			data[1] = STS;
			u32SendNum = 2;
			u32ReqRetNum = 1;

			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);

			if( retVal == XST_SUCCESS )
			{
				if( pBuffer[2] & RACK )
				{
					
					data[0] = NREAD;
				    	data[1] = SIO;
					u32SendNum = 2;
					u32ReqRetNum = 4;
					
					retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
				}
				else
				{
					retVal = XST_FAILURE;
					printf( "Timeout 2 Occured!\n\r" );
					delay_SL(0x100000);
					// Set Page					
					data[0] = NWRITE;
					data[1] = SPG;
					data[2] = page;
					u32SendNum = 3;
					u32ReqRetNum = 0;
					retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
					if( retVal != XST_SUCCESS )
					{
						printf("Call XSpiPs_PT_SL 1 Failed\n");
					}

					goto READ_STS_2;
				}

			}
			else
				printf("Call XSpiPs_PT_SL 3 Failed\n\r");
		}
		else
		{
			retVal = XST_FAILURE;
			printf( "Timeout 1 Occured!\n" );
			delay_SL(0x100000);

			// Set Page			
			data[0] = NWRITE;
			data[1] = SPG;
			data[2] = page;
			u32SendNum = 3;
			u32ReqRetNum = 0;

			retVal = XSpiPs_PolledTransfer(Spi_ptr,data,pBuffer,u32SendNum+u32ReqRetNum);
			if( retVal != XST_SUCCESS )
			{
				printf("Call XSpiPs_PT_SL 1 Failed\n");
			}

			goto READ_STS_1;
		}
	}
	else
		printf("Call XSpiPs_PT_SL 4 Failed\n");

	return retVal;
}
#endif

#if 1

int writeBCM5396(XSpiPs SpiInstance, u8 page, u8 offset, u8 *pBuffer, u8 num)
{
	u8 wr_buf[20],rd_buf[10], iLoop;
	s32 retVal;
	u32 uiSendNum, uiRecvNum;

	memset(rd_buf,0,10);

	// Set Page
	wr_buf[0] = NWRITE;
	wr_buf[1] = SPG;
	wr_buf[2] = page;
	uiSendNum = 3;
	uiRecvNum = 0;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,NULL,uiSendNum+uiRecvNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 write 1 Failed\r\n");
	}

	delay_SL(1000);

	wr_buf[0] = NWRITE;
	wr_buf[1] = offset;
	for (iLoop = 0; iLoop < num; iLoop++)
	{
		wr_buf[2+iLoop] = pBuffer[iLoop];
	}
	uiSendNum = 2 + num;
	uiRecvNum = 0;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,rd_buf,uiSendNum+uiRecvNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 write 2 Failed\r\n");
	}

	delay_SL(10000);

	return retVal;
}

int readBCM5396(XSpiPs SpiInstance, u8 page, u8 offset, u8 num)
{
	u8 wr_buf[8], rd_buf[8];
	u32 retVal;
	u32 uiSendNum, uiRecvNum;
	u32 result = 0;

	memset(wr_buf, 0, sizeof(wr_buf));
	memset(rd_buf, 0, sizeof(rd_buf));

	// Set Page
	wr_buf[0] = NWRITE;
	wr_buf[1] = SPG;
	wr_buf[2] = page;
	uiSendNum = 3;
	uiRecvNum = 0;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,rd_buf,uiSendNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 read 1 Failed\r\n");
	}

	delay_SL(1000);

	// Set offset
    wr_buf[0] = NREAD;
	wr_buf[1] = offset;
	uiSendNum = 2;
	uiRecvNum = 1;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,rd_buf,uiSendNum+uiRecvNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 read 2 Failed\r\n");
	}

	delay_SL(1000);

	// Read
    wr_buf[0] = NREAD;
	wr_buf[1] = STS;
	uiSendNum = 2;
	uiRecvNum = 1;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,rd_buf,uiSendNum+uiRecvNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 read 3 Failed\r\n");
	}

	delay_SL(1000);

	// Read
    wr_buf[0] = NREAD;
	wr_buf[1] = SIO;
	uiSendNum = 2;
	uiRecvNum = num;
	retVal = XSpiPs_PolledTransfer(&SpiInstance,wr_buf,rd_buf,uiSendNum+uiRecvNum);
	if( retVal != XST_SUCCESS )
	{
		printf("bcm5396 read 4 Failed\r\n");
	}

	memcpy(&result, rd_buf + 2, num);
//	printf("read result is 0x%x\n",result);
	delay_SL(10000);

	return result;
}

#endif


void mySPI_init()
{
	XSpiPs Spi;
	int i32Option,status;
	int i;
	unsigned char workBuf[10];

	workBuf[0]=0xf0;
	workBuf[1]=0x1;

	g_SpiConfig = &XSpiPs_ConfigTable[0];

	//Initialize the SPI device.
	Spi.IsBusy = 0;

	Spi.Config.BaseAddress = g_SpiConfig->BaseAddress;
	//Spi.StatusHandler = StubStatusHandler;

	Spi.SendBufferPtr = NULL;
	Spi.RecvBufferPtr = NULL;
	Spi.RequestedBytes = 0;
	Spi.RemainingBytes = 0;
	Spi.IsReady = 0x11111111;

	XSpiPs_WriteReg(Spi.Config.BaseAddress,0,0x00020000);
	//Initialize the SPI device. end
	
	i32Option = 0x1 | 0x2 | 0x4 | 0x20;
	Spi_SetOptions(&Spi, i32Option);

	XSpiPs_SetSlaveSelect(&Spi,0); // 0-2

	XSpiPs_SetClkPrescaler(&Spi, 0x7);

	workBuf[0]=0xf0;
	workBuf[1]=0x1;
	for(i=0x10;i<=0x1f;i++)
	{
		status = writeBCM5396(Spi, i, 0x20, workBuf, 2);
	}
}
