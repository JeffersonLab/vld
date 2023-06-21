/**
 * @copyright Copyright 2022, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *            Fax:   (757) 269-5800             Newport News, VA 23606
 *
 * @file      vldLib.c
 * @brief     Library for the JLab VME LED Driver
 *
 */

/** \cond PRIVATE */

#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include "jvme.h"
#include "vldLib.h"
#include "vldShm.h"

/* Mutex to guard register read/writes */
pthread_mutex_t vldMutex = PTHREAD_MUTEX_INITIALIZER;
#define VLOCK   if(pthread_mutex_lock(&vldMutex)<0) perror("pthread_mutex_lock");
#define VUNLOCK if(pthread_mutex_unlock(&vldMutex)<0) perror("pthread_mutex_unlock");

#define CHECKID(id)							\
  if((id<0) || (id>=MAX_VME_SLOTS) || (VLDp[id] == NULL))		\
    {									\
      printf("%s: ERROR : VLD id %d is not initialized \n",		\
	     __func__, id);						\
      return ERROR;							\
    }

/** \endcond */

/* Globals */
int32_t nVLD = 0;                        /* Number of initialized modules */
volatile vldRegs *VLDp[MAX_VME_SLOTS+1];  /* pointer to memory maps, index = slotID */
volatile vldSerialRegs *VLDJTAGp[MAX_VME_SLOTS+1];
volatile vldSerialRegs *VLDI2Cp[MAX_VME_SLOTS+1];
int32_t vldID[MAX_VME_SLOTS+1];                    /**< array of slot numbers */
unsigned long vldA24Offset = 0;          /* Difference in CPU A24 Base and VME A24 Base */
uint32_t vldAddrList[MAX_VME_SLOTS+1];     /**< array of a24 addresses */
uint16_t vldFWVers[MAX_VME_SLOTS+1];


/**
 * @brief Check the register map
 * @details DEBUG Routine to check the register map with the vldRegs structure
 * @return OK if successful, otherwise ERROR
 */
int32_t
vldCheckAddresses()
{
  int32_t rval = OK;
  uintptr_t offset = 0, expected = 0, base = 0;
  vldRegs fbase;

  printf("%s:\n\t ---------- Checking VLD memory map ---------- \n",
	 __func__);

  base = (uintptr_t) &fbase;
/** \cond PRIVATE */
#ifndef CHECKOFFSET
#define CHECKOFFSET(_x, _y)						\
  offset = ((uintptr_t) &fbase._y) - base;				\
  expected = _x;							\
  if(offset != expected)						\
  {									\
  printf("%s: ERROR ->%s not at offset = 0x%lx (@ 0x%lx)\n",		\
	 __func__, #_y ,expected, offset);				\
  rval = ERROR;								\
  }
#endif
/** \endcond */

  CHECKOFFSET(0x0C, trigDelay);
  CHECKOFFSET(0x68, bleachTime);
  CHECKOFFSET(0x100, reset);

  return rval;
}

/**
 * @brief Initialize the VLD Library
 *
 * @details Increment through A24 addresses and initialize library
 * with modules that match the VLD boardID and supported firmware version(s).
 *
 * @param[in] addr First address to check
 * @param[in] addr_inc The inc of addr
 * @param[in] nfind number of addr_inc
 *
 * @param[in] iFlag Initialization mask
 *       bit| desc
 *       ---|-------------------------
 *        0 | No module initialization
 *        1 | Skip the firmware check
 *        2 | Increment Using user initialized vldAddrList array
 * @return OK if successful, otherwise ERROR
 */
int32_t
vldInit(uint32_t addr, uint32_t addr_inc, uint32_t nfind, uint32_t iFlag)
{
  int32_t useList=0, noBoardInit=0;
  int32_t islot, ivld;
  int32_t res;
  uint32_t rdata, boardID;
  uintptr_t laddr, laddr_inc;
  uint32_t firmwareInfo=0, vldVersion=0, vldType=0;
  volatile vldRegs *vld;


  /* Check if we're skipping initialization, and just mapping the structure pointer */
  if(iFlag&VLD_INIT_NO_INIT)
    {
      noBoardInit = 1;
    }

  /* Check if we're initializing using a list */
  if(iFlag&VLD_INIT_USE_ADDR_LIST)
    {
      useList=1;
    }

  /* Check for valid address */
  if( (addr==0) && (useList==0) )
    { /* Scan through valid slot -> A24 address */
      useList=1;
      nfind=16;

      /* Loop through JLab VXS Weiner Crate GEOADDR to VME addresses to make a list */
      for(islot=3; islot<11; islot++) /* First 8 */
	vldAddrList[islot-3] = (islot<<19);

      /* Skip Switch Slots */

      for(islot=13; islot<21; islot++) /* Last 8 */
	vldAddrList[islot-5] = (islot<<19);

    }
  else if(addr > 0x00ffffff)
    { /* A32 Addressing */
      printf("%s: ERROR: A32 Addressing not allowed for VLD configuration space\n",
	     __func__);
      return(ERROR);
    }
  else
    { /* A24 Addressing */
      if(addr < 22)
	{ /* First argument is a slot number, instead of VME address */
	  printf("%s: Initializing using slot number %d (VME address 0x%x)\n",
		 __func__,addr,addr<<19);
	  addr = addr<<19; // Shift to VME A24 address;

	  /* If addr_inc is also in slot number form, shift it */
	  if((addr_inc<22) && (addr_inc>0))
	    addr_inc = addr_inc<<19;

	  /* Check and shift the address list, if it's used */
	  if(useList==1)
	    {
	      for(ivld=0; ivld<nfind; ivld++)
		{
		  if(vldAddrList[ivld] < 22)
		    {
		      vldAddrList[ivld] = vldAddrList[ivld]<<19;
		    }
		}
	    }
	}

      if( ((addr_inc==0)||(nfind==0)) && (useList==0) )
	{ /* assume only one VLD to initialize */
	  nfind = 1;
	  vldAddrList[0] = addr;
	}
    }

  /* get the VLD address */
#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x39,(char *)(unsigned long)addr,(char **)&laddr);
#else
  res = vmeBusToLocalAdrs(0x39,(char *)(unsigned long)addr,(char **)&laddr);
#endif

#ifndef SHOWERROR
#ifndef VXWORKS
  vmeSetQuietFlag(1);
#endif
#endif

  if (res != 0)
    {
#ifdef VXWORKS
      printf("%s: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",
	     __func__,addr);
#else
      printf("%s: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",
	     __func__,addr);
#endif
      return(ERROR);
    }
  vldA24Offset = laddr - addr;

  for (ivld=0;ivld<nfind;ivld++)
    {
      if(useList==1)
	{
	  laddr_inc = vldAddrList[ivld] + vldA24Offset;
	}
      else
	{
	  laddr_inc = laddr +ivld*addr_inc;
	}

      vld = (vldRegs *)laddr_inc;
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(vld->boardID),VX_READ,4,(char *)&rdata);
#else
      res = vmeMemProbe((char *) &(vld->boardID),4,(char *)&rdata);
#endif

      if(res < 0)
	{
#ifdef SHOWERRORS
#ifdef VXWORKS
	  printf("%s: ERROR: No addressable board at addr=0x%x\n",
		 __func__,(UINT32) vld);
#else
	  printf("%s: ERROR: No addressable board at VME (Local) addr=0x%x (0x%x)\n",
		 __func__,
		 (UINT32) laddr_inc-vldA24Offset, (UINT32) vld);
#endif
#endif /* SUPPRESSERRORSES */
	}
      else
	{
	  /* Check that it is a VLD */
	  if(((rdata&VLD_BOARDID_TYPE_MASK)>>16) != VLD_BOARDID_TYPE_VLD)
	    {
	      printf(" WARN: For board at VME addr=0x%x, Invalid Board ID: 0x%x\n",
		     (UINT32)(laddr_inc - vldA24Offset), rdata);
	      continue;
	    }
	  else
	    {
	      /* Check if this is board has a valid slot number */
	      boardID =  (rdata&VLD_BOARDID_GEOADR_MASK)>>8;
	      if((boardID <= 0)||(boardID >21))
		{
		  printf(" WARN: Board Slot ID is not in range: %d (this module ignored)\n"
			 ,boardID);
		  continue;
		}
	      else
		{
		  VLDp[boardID] = (vldRegs *)(laddr_inc);
		  vldID[nVLD] = boardID;
		  firmwareInfo = vmeRead32(&VLDp[boardID]->firmwareVersion) & VLD_FIRMWARE_ID_MASK;
		  vldFWVers[boardID] = firmwareInfo;

		  if(firmwareInfo <= 0)
		    {
		      printf("%s:  ERROR: Invalid firmware 0x%08x\n",
			     __func__,firmwareInfo);
		      return ERROR;
		    }

		  printf("Initialized VLD %2d  FW 0x%2x Slot #%d at address 0x%08x \n",
			 nVLD, firmwareInfo, vldID[nVLD],
			 (uint32_t)((unsigned long)VLDp[(vldID[nVLD])]-vldA24Offset));
		}
	    }
	  nVLD++;
	}
    }

#ifndef SHOWERROR
#ifndef VXWORKS
  vmeSetQuietFlag(0);
#endif
#endif

  if(noBoardInit)
    {
      if(nVLD>0)
	{
	  printf("%s: %d VLD(s) successfully mapped (not initialized)\n",
		 __func__,nVLD);
	  return OK;
	}
    }

  if(nVLD==0)
    {
      printf("%s: ERROR: Unable to initialize any VLD modules\n",
	     __func__);
      return ERROR;
    }

  vldShmCreateLockShm();

  return OK;

}

/**
 * @brief Return the slot ID
 * @details Given the order of initialization, return the slot ID of the provided index
 * @param[in] index
 * @return slot ID if successful, otherwise ERROR
 */
int32_t
vldSlot(uint32_t index)
{
  if(index >= nVLD)
    {
      printf("%s: ERROR: Invalid index (%d)\n",
	     __func__, index);
      return ERROR;
    }
  return vldID[index];
}

/**
 * @brief Return a mask of initialized VLD slotIDs
 * @return a mask of initialized VLD slotIDs
 */
uint32_t
vldSlotMask()
{
  int32_t iv=0;
  uint32_t dmask=0;

  for(iv = 0; iv < nVLD; iv++)
    {
      dmask |= (1 << (vldID[iv]));
    }

  return dmask;
}

int32_t
vldGetNVLD()
{
  return nVLD;
}

/**
 * @brief Return the geographic address
 * @details Return the geographic address of the specified module
 * @param[in] id Slot ID
 * @return Geographic address, if successful.  Otherwise ERROR.
 */
int32_t
vldGetGeoAddress(int id)
{
  int32_t rval = 0;
  CHECKID(id);

  VLOCK;
  rval = (vmeRead32(&VLDp[id]->boardID) & VLD_BOARDID_GEOADR_MASK)>>8;
  VUNLOCK;

  return rval;
}

/**
 * @brief Show the settings and status of the initialized VLD
 * @param[in] pFlag unused
 */
void
vldGStatus(int32_t pFlag)
{
  vldRegs *rb;
  uint32_t iv, slot;

  rb = (vldRegs *)malloc((MAX_VME_SLOTS + 1) * sizeof(vldRegs));

/** \cond PRIVATE */
#ifndef READVLD
#define READVLD(_id, _reg)			\
  rb[_id]._reg = vmeRead32(&VLDp[_id]->_reg);
#endif
/** \endcond */

  VLOCK;
  for(iv = 0; iv < nVLD; iv++)
    {
      slot = vldSlot(iv);
      READVLD(slot, boardID);
      READVLD(slot, trigDelay);
      READVLD(slot, trigSrc);
      READVLD(slot, clockSrc);
      READVLD(slot, bleachTimeRB);
      READVLD(slot, calibrationWidth);
      READVLD(slot, analogCtrl);
      READVLD(slot, randomTrig);
      READVLD(slot, periodicTrig);
      READVLD(slot, trigCnt);
    }
  VUNLOCK;

  printf("VLD Module Status Summary\n");

  printf("         Firmware  Trigger Source........................            Clock...\n");
  printf("Slot     Version   Periodic  Random    Sequence  External            Source\n");
  printf("--------------------------------------------------------------------------------\n");
  /* printf("13       0x1234    Enabled   Enabled   Enabled   Enabled             External"); */

  for(iv = 0; iv < nVLD; iv++)
    {
      slot = vldSlot(iv);

      /* Slot */
      printf("%2d       ", slot);

      printf("0x%02x      ", vldFWVers[slot]);

      printf("%s  ", (rb[slot].trigSrc & VLD_TRIGSRC_INTERNAL_PERIODIC_ENABLE) ?
	     "Enabled " : "Disabled");

      printf("%s  ", (rb[slot].trigSrc & VLD_TRIGSRC_INTERNAL_RANDOM_ENABLE) ?
	     "Enabled " : "Disabled");

      printf("%s  ", (rb[slot].trigSrc & VLD_TRIGSRC_INTERNAL_SEQUENCE_ENABLE) ?
	     "Enabled " : "Disabled");

      printf("%s            ", (rb[slot].trigSrc & VLD_TRIGSRC_EXTERNAL_ENABLE) ?
	     "Enabled " : "Disabled");

      printf("%s", (rb[slot].clockSrc & VLD_CLOCK_EXTERNAL) ?
	     "External" : "Internal");

      printf("\n");
    }

  printf("\n");
  printf("         Bleach Timer.....   Calibration.....    Analog Switch.....\n");
  printf("Slot     Time[ms]  Status    Pulse Width[ns]     Delay[ns] Width[ns]\n");
  printf("--------------------------------------------------------------------------------\n");
  /* printf("13       123456789 Enabled   4092                1024      508"); */

  for(iv = 0; iv < nVLD; iv++)
    {
      slot = vldSlot(iv);

      /* Slot */
      printf("%2d       ",slot);

      printf("%10u ", (int)
	     ((rb[slot].bleachTime & VLD_BLEACHTIME_TIMER_MASK) * 20 * 1024 * 1024) / 1000);

      printf("%s  ",
	     ((rb[slot].bleachTime & VLD_BLEACHTIME_ENABLE_MASK) == VLD_BLEACHTIME_ENABLE) ?
	     "Enabled " : "Disabled");

      printf("%4d                ",
	     (rb[slot].calibrationWidth & VLD_CALIBRATIONWIDTH_MASK) * 4);

      printf("%4d      ",
	     (rb[slot].analogCtrl & VLD_ANALOGCTRL_DELAY_MASK) * 4);

      printf("%3d",
	     ((rb[slot].analogCtrl & VLD_ANALOGCTRL_WIDTH_MASK) >> 9) * 4);

      printf("\n");
    }

  printf("\n");
  printf("         Random Pulser..............   Periodic Pulser............\n");
  printf("Slot     Prescale  Rate[kHz] Status    Period    npulses\n");
  printf("--------------------------------------------------------------------------------\n");
  /* printf("13       7         700000    Enabled   1234.123  12345151"); */

  for(iv = 0; iv < nVLD; iv++)
    {
      slot = vldSlot(iv);

      /* Slot */
      printf("%2d       ", slot);

      printf("%d         ",
	     (rb[slot].randomTrig & VLD_RANDOMTRIG_PRESCALE_MASK));

      printf("%6d   ",
	     700000 >> (rb[slot].randomTrig & VLD_RANDOMTRIG_PRESCALE_MASK) );

      printf("%s  ",
	     (rb[slot].randomTrig & VLD_RANDOMTRIG_ENABLE) ? "Enabled " : "Disabled");

      printf("%7d  ",
	     120 + (30 * (rb[slot].periodicTrig & VLD_PERIODICTRIG_PERIOD_MASK)>>16));

      printf("%d",
	     rb[slot].periodicTrig & VLD_PERIODICTRIG_NPULSES_MASK);

      printf("\n");
    }

  if(rb)
    free(rb);

}

/**
 * @brief Set the trigger delay and pulse width

 * @details For the specified slot id, update the trigger delay and
 * pulse width.
 * The output pulse delay is determined by
 *
 *     out_delay [ns] = 1024 * (delaystep) + (delay + 1) * 4 * 4**(delaystep)
 *
 * The output pulse width is determined by
 *
 *     out_width [ns] = (width + 1) * 4
 *
 * @param[in] id Slot ID
 * @param[in] delay `[0,127]` Delay value, units determined by delaystep
 * @param[in] delaystep `[0,1]` Delay units
 *     delaystep | value
 *     ----------|-------
 *      0        | 4ns
 *      1        | 16ns
 * @param[in] width `[0, 31]` Width value, units of `4ns`
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetTriggerDelayWidth(int32_t id, int32_t delay, int32_t delaystep, int32_t width)
{
  uint32_t wval=0;
  CHECKID(id);

  if((delay<0) || (delay>0x7F))
    {
      printf("%s(%d): ERROR: Invalid delay (%d).  Must be <= %d\n",
	     __func__, id,delay, 0x7f);
      return ERROR;
    }
  if((width<0) || (width> 0x1F))
    {
      printf("%s(%d): ERROR: Invalid width (%d).  Must be <= %d\n",
	     __func__, id,width, 0x1F);
    }

  delaystep = (delaystep ? VLD_TRIGDELAY_16NS_STEP_ENABLE : 0);

  VLOCK;
  wval = (delay) | (delaystep) | (width << 8);

  vmeWrite32(&VLDp[id]->trigDelay, wval);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the trigger pulse delay and width parameters
 * @details For the specified slot id, return the trigger delay and
 * pulse width parameters.
 * The output pulse delay is determined by
 *
 *     out_delay [ns] = 1024 * (delaystep) + (delay + 1) * 4 * 4**(delaystep)
 *
 * The output pulse width is determined by
 *
 *     out_width [ns] = (width + 1) * 4
 *
 * @param[in] id Slot ID
 * @param[out] delay Delay value, units determined by delaystep
 * @param[out] delaystep Delay units
 * @param[out] width Width value
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetTriggerDelayWidth(int32_t id, int32_t *delay, int32_t *delaystep, int32_t *width)
{
  uint32_t rval=0;
  CHECKID(id);

  VLOCK;
  rval = vmeRead32(&VLDp[id]->trigDelay);

  *delay = rval & VLD_TRIGDELAY_DELAY_MASK;
  *delaystep = (rval & VLD_TRIGDELAY_16NS_STEP_ENABLE) ? 1 : 0;
  *width = (rval & VLD_TRIGDELAY_WIDTH_MASK) >> 8;

  VUNLOCK;

  return OK;
}

/**
 * @brief Set the trigger source mask
 * @details Set the trigger source of the specified VLD using the bits:
 *    bit | trigger source
 *       -|-
 *     0  | internal periodic
 *     1  | internal random
 *     2  | internal sequence
 *     4  | external input
 *
 * @param[in] id Slot ID
 * @param[in] trigSrc trigger source mask
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetTriggerSourceMask(int32_t id, uint32_t trigSrc)
{
  CHECKID(id);

  if((trigSrc & ~VLD_TRIGSRC_MASK) != 0)
    {
      printf("%s(%d): ERROR: Invalid trigSrc Mask (0x%x).  Allowed bits in mask 0x%x\n",
	     __func__, id, trigSrc, VLD_TRIGSRC_MASK);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->trigSrc, trigSrc);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the trigger source mask
 * @details Get the trigger source of the specified VLD. Trigger bits are:
 *    bit | trigger source
 *       -|-
 *     0  | internal periodic
 *     1  | internal random
 *     2  | internal sequence
 *     4  | external input
 * @param[in] id Slot ID
 * @param[out] trigSrc Enabled Trigger Source mask
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetTriggerSourceMask(int32_t id, uint32_t *trigSrc)
{
  CHECKID(id);

  VLOCK;
  *trigSrc = vmeRead32(&VLDp[id]->trigSrc) & VLD_TRIGSRC_MASK;
  VUNLOCK;

  return OK;
}


/**
 * @brief Set the clock source
 * @details Set the clock source for the specified VLD modlue
 * @param[in] id Slot ID
 * @param[in] clkSrc `[0,1]` Selected Clock Source
 *       clkSrc | desc
 *             -|-
 *         0    | Onboard Oscillator
 *         1    | External LEMO connector input
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetClockSource(int32_t id, uint32_t clkSrc)
{
  CHECKID(id);

  if(clkSrc > 1)
    {
      printf("%s(%d): ERROR: Invalid clkSrc (%d)\n",
	     __func__, id, clkSrc);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->clockSrc, clkSrc);
  sleep(1);
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_CLK);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the clock source
 * @details Get the clock source for the specified VLD modlue
 * @param[in] id Slot ID
 * @param[out] clkSrc clkSrc `[0,1]` Selected Clock Source
 *       clkSrc | desc
 *             -|-
 *         0    | Onboard Oscillator
 *         1    | External LEMO connector input
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetClockSource(int32_t id, uint32_t *clkSrc)
{
  CHECKID(id);

  VLOCK;
  *clkSrc = vmeRead32(&VLDp[id]->clockSrc) & VLD_CLOCK_MASK;
  VUNLOCK;

  return OK;
}

/**
 * @brief Control the bleach current setting
 * @details Control the beach current setting for the specified slot ID and connector
 * @param[in] id Slot ID
 * @param[in] connector `[0,4]` Connector ID
 * @param[in] ctrlLDO `[0,7]` Bleach current setting
 * @param[in] enableLDO `[0,1]` Disable (0) / Enable (1) LDO Regulator
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetBleachCurrent(int32_t id, uint32_t connector,
		    uint32_t ctrlLDO, uint32_t enableLDO)
{
  uint32_t rval = 0;
  CHECKID(id);

  if(connector > 4)
    {
      printf("%s(%d): ERROR: Invalid connector (%d).\n",
	     __func__, id, connector);
      return ERROR;
    }

  if(ctrlLDO > 0x7)
    {
      printf("%s(%d): ERROR: Invalid ctrlLDO (0x%x)\n",
	     __func__, id, ctrlLDO);
      return ERROR;
    }

  enableLDO = enableLDO ? (LED_CONTROL_BLEACH_REG_ENABLE | LED_CONTROL_BLEACH_ENABLE) : 0;

  VLOCK;
  /* Disable calibration mode */
  vmeWrite32(&VLDp[id]->output[connector].high, 0);

  /* Disable calibration mode, Set LDO control, and bleaching enable */
  vmeWrite32(&VLDp[id]->output[connector].low_ctrl,
	     (ctrlLDO << 24) | enableLDO);

  VUNLOCK;
  return OK;
}

/**
 * @brief Get the bleach current setting
 * @details Get the beach current setting for the specified slot ID and connector
 * @param[in] id Slot ID
 * @param[in] connector `[0,4]` Connector ID
 * @param[in] ctrlLDO `[0,7]` Bleach current setting
 * @param[in] enableLDO `[0,1]` Disable (0) / Enable (1) LDO Regulator
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetBleachCurrent(int32_t id, uint32_t connector,
		    uint32_t *ctrlLDO, uint32_t *enableLDO)
{
  uint32_t rval = 0, reg = 0;
  CHECKID(id);

  if(connector > 4)
    {
      printf("%s(%d): ERROR: Invalid connector (%d).\n",
	     __func__, id, connector);
      return ERROR;
    }

  VLOCK;
  /* Get LDO control, and bleaching enable */
  reg = vmeRead32(&VLDp[id]->output[connector].low_ctrl);

  *ctrlLDO = (reg & LED_CONTROL_BLEACH_CTRL_MASK) >> 24;
  *enableLDO = (reg & LED_CONTROL_BLEACH_ENABLE_MASK) == LED_CONTROL_BLEACH_ENABLE ? 1 : 0;

  VUNLOCK;
  return OK;
}

/**
 * @brief Set the bleaching timer
 * @details Set the bleaching timer for the specified module
 * @param[in] id Slot ID
 *
 * @param[in] timer [0, 0x0FFFFFFF] Bleaching time.  If 0, keep the
 * currently stored value.  Otherwise set the tmie in units of `20ns * 1024 * 1024`
 *
 * @param[in] enable `[0,1]` Disable (0) / Enable (1) bleaching timer
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetBleachTime(int32_t id, uint32_t timer, uint32_t enable)
{
  uint32_t wval = 0;
  CHECKID(id);

  if(timer > VLD_BLEACHTIME_TIMER_MASK)
    {
      printf("%s(%d): WARN: Invalid Bleach time %u (0x%x).  Setting to Max (0x%x)\n",
	     __func__, id, timer, timer, VLD_BLEACHTIME_TIMER_MASK);
      timer = VLD_BLEACHTIME_TIMER_MASK;
    }

  enable = enable ? VLD_BLEACHTIME_ENABLE : 0;

  VLOCK;
  if(timer == 0)
    timer = vmeRead32(&VLDp[id]->bleachTime) & VLD_BLEACHTIME_TIMER_MASK;

  wval = timer | enable;

  /*
    "just want to make sure that the next write (data 0xB.....) will generate a rising edge"
   */
  if(enable)
    vmeWrite32(&VLDp[id]->bleachTime, 0);

  vmeWrite32(&VLDp[id]->bleachTime, wval);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the status of the bleaching timer
 * @details Get the status of the bleaching timer for the specified module
 * @param[in] id Slot ID
 *
 * @param[out] timer Timer value, units of `20ns * 1024 * 1024`
 * @param[out] enable Timer enabled status
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetBleachTime(int32_t id, uint32_t *timer, uint32_t *enable)
{
  uint32_t rval = 0;
  CHECKID(id);

  VLOCK;
  rval = vmeRead32(&VLDp[id]->bleachTimeRB);

  *timer = rval & VLD_BLEACHTIME_TIMER_MASK;

  *enable = ((rval & VLD_BLEACHTIME_ENABLE_MASK) == VLD_BLEACHTIME_ENABLE );
  VUNLOCK;

  return OK;
}

/**
 * @brief Set the channel masks for the specified connector
 * @details Set the channel masks for the specified connector
 * @param[in] id Slot ID
 * @param[in] connector `[0,4]` Connector ID
 * @param[in] lochanEnableMask [0x3FFFFF] Enable mask for lower 18 channels
 * @param[in] hichanEnableMask [0x3FFFFF] Enable mask for upper 18 channels
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetChannelMask(int32_t id,
		  uint32_t connector, uint32_t lochanEnableMask, uint32_t hichanEnableMask)
{
  uint32_t rval = 0;
  CHECKID(id);

  if(connector > 4)
    {
      printf("%s(%d): ERROR: Invalid connector (%d).\n",
	     __func__, id, connector);
      return ERROR;
    }

  if(lochanEnableMask > 0x0003FFFF)
    {
      printf("%s(%d): ERROR: Invalid lochanEnableMask (0x%x).\n",
	     __func__, id, lochanEnableMask);
      return ERROR;
    }

  if(hichanEnableMask > 0x0003FFFF)
    {
      printf("%s(%d): ERROR: Invalid hichanEnableMask (0x%x).\n",
	     __func__, id, hichanEnableMask);
      return ERROR;
    }

  VLOCK;
  /* Set enable mask for channels #19 - #36 */
  vmeWrite32(&VLDp[id]->output[connector].high,
	     LED_CONTROL_CALIBRATION_ENABLE |
	     (hichanEnableMask << 1));

  /* Set enable mask for channels #1 - #18 */
  vmeWrite32(&VLDp[id]->output[connector].low_ctrl,
	     LED_CONTROL_CALIBRATION_ENABLE |
	     (lochanEnableMask << 1));

  VUNLOCK;

  vldShmSetChannelMask(id, connector, lochanEnableMask, hichanEnableMask);

  return OK;

}

/**
 * @brief Get the channel masks for the specified connector
 * @details Get the channel masks for the specified connector
 * @param[in] id Slot ID
 * @param[in] connector `[0,4]` Connector ID
 * @param[out] lochanEnableMask [0x3FFFFF] Enable mask for lower 18 channels
 * @param[out] hichanEnableMask [0x3FFFFF] Enable mask for upper 18 channels
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetChannelMask(int32_t id,
		  uint32_t connector, uint32_t *lochanEnableMask, uint32_t *hichanEnableMask)
{
  uint32_t rval = 0;
  CHECKID(id);

  if(connector > 4)
    {
      printf("%s(%d): ERROR: Invalid connector (%d).\n",
	     __func__, id, connector);
      return ERROR;
    }

  VLOCK;
  /* Get enable mask for channels #19 - #36 */
  *hichanEnableMask = (vmeRead32(&VLDp[id]->output[connector].high) & LED_CONTROL_CH_ENABLE_MASK) >> 1;

  /* Set enable mask for channels #1 - #18 */
  *lochanEnableMask = (vmeRead32(&VLDp[id]->output[connector].low_ctrl) & LED_CONTROL_CH_ENABLE_MASK) >> 1;

  VUNLOCK;
  return OK;
}

/**
 * @brief Pulse Shape loading routine
 * @details Load a pulse shape into the specified module
 * @param[in] id Slot ID
 * @param[in] dac_samples `[0, 0x3F]` Address of Array of DAC samples
 * (2ns) to load. For each sample:
 *      bits | desc
 *          -|-
 *      0:5  | DAC value
 *      6    | a base line setting for the DAC
 *
 * @param[in] nsamples `[0, 2048]` Number of samples in `dac_samples`
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldLoadPulse(int32_t id, uint8_t *dac_samples, uint32_t nsamples)
{
  uint32_t isample = 0, ibyte = 0, wval = 0;
  CHECKID(id);

  if(nsamples > 2048)
    {
      printf("%s(%d): ERROR: Invalid nsamples (%d)\n",
	     __func__, id, nsamples);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_PULSESHAPE_ADDR);

  /* loop through and write 4 byte values out of the 1 byte samples */
  while(isample < nsamples)
    {
      ibyte = (isample % 4);
      wval |= (dac_samples[isample]) << (ibyte*8);

      /* write if on the last byte of wval, or the last byte of array */
      if((ibyte == 3) || (isample == (nsamples - 1)))
	{
	  vmeWrite32(&VLDp[id]->pulseLoad, wval);
	  wval = 0; // clear for next samples */
	}
      isample++;
    }
  VUNLOCK;

  return OK;
}

/**
 * @brief 32bit Pulse Shape loading routine
 * @details Load a 32bit pulse shape into the specified module.  Each
 * index, for this routine, represents 4 samples beginning with the
 * LSB.
 * @param[in] id Slot ID
 * @param[in] dac_samples Address of Array of 32bit DAC samples
 * @param[in] nsamples `[0, 512]` number of 32bit values to write
 * @return OK of successful, otherwise ERROR.
 */
int32_t
vldLoadPulse32(int32_t id, uint32_t *dac_samples, uint32_t nsamples)
{
  uint32_t isample = 0, wval = 0;
  CHECKID(id);

  if(nsamples > 512)
    {
      printf("%s(%d): ERROR: Invalid nsamples (%d)\n",
	     __func__, id, nsamples);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_PULSESHAPE_ADDR);

  /* loop through and write 4 byte values out of the 1 byte samples */
  while(isample < nsamples)
    {
      wval = dac_samples[isample];
      vmeWrite32(&VLDp[id]->pulseLoad, wval);
      isample++;
    }
  VUNLOCK;

  return OK;
}

/**
 * @brief Write example pulse into provided array
 * @details Load an example pulse into provided array.
 * @param[out] dac_samples Address to store example pulse
 * @param[out] nsamples number of samples loaded
 * @return OK of successful, otherwise ERROR.
 */

int32_t
vldGetExamplePulse(uint32_t *dac_samples, uint32_t *nsamples)
{
  int32_t idata;
  uint32_t DataLoad = 0;


  *nsamples = 512;
  for (idata = 0; idata < *nsamples; idata++)
    {
      if ((idata <4)  || (idata >143))
	DataLoad = 0x01010101;

      if ((idata > 3)  && (idata < 67))
	DataLoad = ((2*(idata-4)+3) <<24) + ((2*(idata-4)+2) << 16) +
	  ((2*(idata-4)+1) <<8) + 2*(idata-4);

      if ((idata > 66) && (idata < 80))
	DataLoad = 0xFFFFFFFF;

      if ((idata > 79) && (idata <144))
	DataLoad = ((2*(143-idata)+3) <<24) + ((2*(143-idata)+2) << 16) +
	  ((2*(143-idata)+1) <<8) + 2*(143-idata);

      dac_samples[idata] = DataLoad;
    }

  return OK;
}

/**
 * @brief Load an example pulse into specified VLD
 * @details Load an example pulse into specified VLD
 * @param[in] id Slot ID
 * @return OK of successful, otherwise ERROR.
 */
int32_t
vldLoadExamplePulse(int32_t id)
{
  int32_t rval = 0;
  uint32_t *dac_samples = NULL, nsamples = 512;

  dac_samples = (uint32_t *)malloc(nsamples * sizeof(uint32_t));

  vldGetExamplePulse(dac_samples, &nsamples);

  rval = vldLoadPulse32(id, dac_samples, nsamples);

  if(dac_samples)
    free(dac_samples);

  return rval;
}

/**
 * @brief Load an square pulse into specified VLD
 * @details Load an square pulse into specified VLD, with specified width and dac level
 * @param[in] id Slot ID
 * @param[in] width `[0, 2044]` Pulse width [2ns]
 * @param[in] dac `[0, 63]` DAC value
 * @return OK of successful, otherwise ERROR.
 */
int32_t
vldLoadSquarePulse(int32_t id, uint32_t width, uint8_t dac)
{
  int32_t rval = 0, isample = 0;
  uint8_t idata = 0, max_dac_background = 0xf, *dac_samples = NULL;
  uint32_t max_width = 4 * 512;

  if(width > (max_width - 4))
    {
      printf("%s(%d): ERROR: Invalid width (%d)\n",
	     __func__, id, width);
      return ERROR;
    }

  dac_samples = (uint8_t *)malloc(max_width * sizeof(uint8_t));

  dac_samples[0] = 0;
  dac_samples[1] = 0;
  dac_samples[2] = 0;
  dac_samples[3] = 0;

  for(idata = 4; idata < width+4; idata++)
    {
      dac_samples[idata] = dac;
    }

  /* Fill the reset of the samples with 0 */
  while(idata < max_width)
    {
      dac_samples[idata++] = 0;
    }

  rval = vldLoadPulse(id, dac_samples, max_width);

  if(dac_samples)
    free(dac_samples);

  return rval;

}

/**
 * @brief Set the calibration pulse width
 * @details Set the calibration pulse width of the specified module
 * @param[in] id Slot ID
 * @param[in] width `[0,1023]` Calibration pulse width in units of `4ns`
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetCalibrationPulseWidth(int32_t id, uint32_t width)
{
  CHECKID(id);

  if(width > VLD_CALIBRATIONWIDTH_MASK)
    {
      printf("%s(%d): ERROR: Invalid Calibration Pulse Width (0x%x).  Max = 0x%x\n",
	     __func__, id, width, VLD_CALIBRATIONWIDTH_MASK);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->calibrationWidth, width);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the calibration pulse width
 * @details Get the calibration pulse width of the specified module
 * @param[in] id Slot ID
 * @param[out] width Calibration pulse width in units of `4ns`
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetCalibrationPulseWidth(int32_t id, uint32_t *width)
{
  CHECKID(id);

  VLOCK;
  *width = vmeRead32(&VLDp[id]->calibrationWidth) & VLD_CALIBRATIONWIDTH_MASK;
  VUNLOCK;

  return OK;
}

/**
 * @brief Set the analog switch control
 * @details Set the analog switch control parameters for the specified module
 * @param[in] id Slot ID
 * @param[in] enableDelay `[0,255]` Switch Control delay, in units of 4ns
 * @param[in] enableWidth `[0,127]` Switch Control width, in units of 4ns.  If 0, `width = infinite`.
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetAnalogSwitchControl(int32_t id, uint32_t enableDelay, uint32_t enableWidth)
{
  uint32_t maxDelay = 0xFF, maxWidth = 0x7F;
  CHECKID(id);

  if(enableDelay > maxDelay)
    {
      printf("%s(%d): ERROR: Invalid enableDelay (0%x).  Max = 0x%x\n",
	     __func__, id, enableDelay, maxDelay);
      return ERROR;
    }

  if(enableWidth > maxWidth)
    {
      printf("%s(%d): ERROR: Invalid enableWidth (0%x).  Max = 0x%x\n",
	     __func__, id, enableWidth, maxWidth);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->analogCtrl, enableDelay | (enableWidth << 9));
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the analog switch control
 * @details Get the analog switch control parameters for the specified module
 * @param[in] id Slot ID
 * @param[out] enableDelay Switch Control delay, in units of 4ns
 * @param[out] enableWidth Switch Control width, in units of 4ns.  If 0, `width = infinite`.
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetAnalogSwitchControl(int32_t id, uint32_t *enableDelay, uint32_t *enableWidth)
{
  uint32_t rval = 0;
  CHECKID(id);

  VLOCK;
  rval = vmeRead32(&VLDp[id]->analogCtrl);

  *enableDelay = rval & VLD_ANALOGCTRL_DELAY_MASK;
  *enableWidth = (rval & VLD_ANALOGCTRL_WIDTH_MASK) >> 9;
  VUNLOCK;

  return OK;
}

/**
 * @brief Set the parameters of internal random pulser
 * @details Set the parameters of internal random pulser for the specified module
 * @param[in] id Slot ID
 * @param[in] prescale `[0,15]` Random Pulser Rate prescale. Rate is determined by:
 *      rate ~ (700 kHz) / (2 ** prescale)
 *      e.g. prescale = 5, rate ~ 20 kHz, prescale = 15, rate ~ 21Hz
 * @param[in] enable `[0,1]` Disable (0) / Enable (1) internal random pulser
 * @return Description
 */
int32_t
vldSetRandomPulser(int32_t id, uint32_t prescale, uint32_t enable)
{
  uint32_t maxPrescale = 0xF;
  CHECKID(id);

  if(prescale > maxPrescale)
    {
      printf("%s(%d): ERROR: Invalid prescale 0x%x. Max = %d\n",
	     __func__, id, prescale, maxPrescale);
    }

  enable = enable ? VLD_RANDOMTRIG_ENABLE : 0;

  VLOCK;
  if(prescale == 0)
    prescale = vmeRead32(&VLDp[id]->randomTrig) & VLD_RANDOMTRIG_PRESCALE_MASK;

  vmeWrite32(&VLDp[id]->randomTrig, prescale | (prescale << 4) | enable);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the parameters of internal random pulser
 * @details Get the parameters of internal random pulser for the specified module
 * @param[in] id Slot ID
 * @param[out] prescale Random Pulser Rate prescale. Rate is determined by:
 *      rate ~ (700 kHz) / (2 ** prescale)
 *      e.g. prescale = 5, rate ~ 20 kHz
 * @param[out] enable Disable (0) / Enable (1) internal random pulser
 * @return Description
 */
int32_t
vldGetRandomPulser(int32_t id, uint32_t *prescale, uint32_t *enable)
{
  uint32_t rval = 0;
  CHECKID(id);

  VLOCK;
  rval = vmeRead32(&VLDp[id]->randomTrig);

  *prescale = rval & VLD_RANDOMTRIG_PRESCALE_MASK;
  *enable = (rval & VLD_RANDOMTRIG_ENABLE) ? 1 : 0;
  VUNLOCK;

  return OK;
}


/**
 * @brief Set the parameters for the internal periodic pulser
 * @details Set the parameters for the internal periodic pulser for the specified module
 * @param[in] id Slot ID
 * @param[in] period `[0,32767]` Pulser Period
 * @param[in] period_range
 *     - 0: small period range (min: 32ns, increments of 8ns)
 *     - 1: large period range (min: 32ns, increments of 8.2us)
 * @param[in] npulses `[0,65535]` Number of pulses to generate.  If 65535 (0xffff), not limitted
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSetPeriodicPulser(int32_t id, uint32_t period, uint32_t period_range, uint32_t npulses)
{
  uint32_t maxPeriod = 0x7FFF, maxNpulses = 0xFFFF;
  uint32_t wval = 0;
  CHECKID(id);

  if(period > maxPeriod)
    {
      printf("%s(%d): ERROR: Invalid period (%u).  Max = %u.\n",
	     __func__, id, period, maxPeriod);
      return ERROR;
    }

  if(npulses > maxNpulses)
    {
      printf("%s(%d): WARN: Invalid npulses (%u).  Set to = %u.\n",
	     __func__, id, npulses, maxNpulses);
      npulses = maxNpulses;
    }

  period_range = (period_range) ? 1 : 0;

  VLOCK;
  wval = (npulses) | (period << 16) | (period_range << 31);

  vmeWrite32(&VLDp[id]->periodicTrig, wval);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the parameters for the internal periodic pulser
 * @details Get the parameters for the internal periodic pulser for the specified module
 * @param[in] id Slot ID
 * @param[out] period Pulser Period
 * @param[out] npulses Number of pulses to generate
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetPeriodicPulser(int32_t id, uint32_t *period, uint32_t *period_range, uint32_t *npulses)
{
  uint32_t rval = 0;
  CHECKID(id);

  VLOCK;
  rval = vmeRead32(&VLDp[id]->periodicTrig);

  *npulses = (rval & VLD_PERIODICTRIG_NPULSES_MASK);
  *period = (rval & VLD_PERIODICTRIG_PERIOD_MASK) >> 16;
  *period_range = (rval & VLD_PERIODICTRIG_PERIOD_MASK) >> 31;
  VUNLOCK;

  return OK;
}


/**
 * @brief Get the trigger count
 * @details Get the trigger count from the specified module
 * @param[in] id Slot ID
 * @param[out] trigCnt Trigger Count
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetTriggerCount(int32_t id, uint32_t *trigCnt)
{
  CHECKID(id);

  VLOCK;
  *trigCnt = vmeRead32(&VLDp[id]->trigCnt);
  VUNLOCK;

  return OK;
}

/**
 * @brief Reset based on specified reset bits
 * @details Reset the specified module based on the specified reset bits
 * @param[in] id Slot ID
 * @param[in] resetMask Reset Mask
 *     bit | desc
 *        -|-
 *       1 | I2C
 *       2 | JTAG
 *       4 | Clock
 *      10 | MGT
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldResetMask(int32_t id, uint32_t resetMask)
{
  CHECKID(id);

  if((resetMask & ~VLD_RESET_MASK) != 0)
    {
      printf("%s(%d): WARN: Of resetMask (0x%x), only these are defined (0x%x)\n",
	     __func__, id, resetMask, resetMask & VLD_RESET_MASK);
    }

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, resetMask & VLD_RESET_MASK);
  VUNLOCK;

  return OK;
}

/**
 * @brief I2C Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldResetI2C(int32_t id)
{
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_I2C);
  VUNLOCK;

  return OK;
}

/**
 * @brief JTAG Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldResetJTAG(int32_t id)
{
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_JTAG);
  VUNLOCK;

  return OK;
}

/**
 * @brief Soft Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldSoftReset(int32_t id)
  {
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_SOFT);
  VUNLOCK;

  return OK;
}

/**
 * @brief Clock DCM Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldResetClockDCM(int32_t id)
{
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_CLK);
  VUNLOCK;

  return OK;
}

/**
 * @brief MGT Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldResetMGT(int32_t id)
{
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_MGT);
  VUNLOCK;

  return OK;
}

/**
 * @brief Hard Clock Reset
 * @param[in] id Slot ID
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldHardClockReset(int32_t id)
{
  CHECKID(id);

  VLOCK;
  vmeWrite32(&VLDp[id]->reset, VLD_RESET_HARD_CLK);
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the module's board ID
 * @param[in] id Slot ID
 * @param[out] boardID from module
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetBoardID(int32_t id, uint32_t *boardID)
{
  CHECKID(id);

  VLOCK;
  *boardID = (vmeRead32(&VLDp[id]->boardID) & VLD_BOARDID_TYPE_MASK) >> 16;
  VUNLOCK;

  return OK;
}

/**
 * @brief Get the module's firmware version
 * @param[in] id Slot ID
 * @param[out] fw Firmware version from module
 * @return If successful, OK.  Otherwise ERROR.
 */
int32_t
vldGetFirmwareVersion(int32_t id, uint32_t *fw)
{
  CHECKID(id);

  VLOCK;
  *fw = vmeRead32(&VLDp[id]->firmwareVersion) & 0xFF;
  VUNLOCK;


  return OK;
}
