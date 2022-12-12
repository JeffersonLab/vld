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
 * @file      VLDtest6.c
 * @brief     Ported version of William Gu's VLDtest6
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include "jvme.h"
#include "vldLib.h"

void VLDtest6(unsigned int islot);

int32_t
main(int32_t argc, char* argv[])
{
  uint32_t slot = 0;
  int32_t status = 0;

  if(argc != 2)
    {
      printf("Usage:  %s  <slotnumber>\n",
	     argv[0]);
      return ERROR;
    }

  slot = strtol(argv[1], NULL, 10);

  status = vmeOpenDefaultWindows();
  if(status != OK)
    {
      vmeCloseDefaultWindows();
      return ERROR;
    }

  VLDtest6(slot);

  vmeCloseDefaultWindows();

  return OK;
}

// VLD individual connector and channel test, seems like test3 is
// doing this too.  (so maybe simplify the test3, and leave some to
// test6)

void
VLDtest6(unsigned int islot)
{
  unsigned int TestReg, DataLow, DataHigh, nsamples = 512;
  int iloop, iconnector, iwaitsum, ReadyNext, iBleach;
  uint32_t DataLoad, *dac_samples;

  printf(" Test the individual channels, \n");
  printf(" Move the LED cable from connector to connector. \n");

  /* Initialize VLD at islot */
  vldInit(islot << 19, 0, 1, 0);

  /* set to the Bleaching timer */
  vldSetBleachTime(islot, 0xabcc, 1); /* about 1000 seconds */

  /* set the a constant calibration pulse */
  dac_samples = (uint32_t *)malloc(nsamples * sizeof(uint32_t));

  for (iloop = 0; iloop < nsamples; iloop++)
    {
      if ((iloop <4)  || (iloop >143))
	DataLoad = 0x01;

      if ((iloop > 3)  && (iloop < 67))
	DataLoad = 2*(iloop-4);

      if ((iloop > 66) && (iloop < 80))
	DataLoad = 0xFF;

      if ((iloop > 79) && (iloop <144))
	DataLoad = 2*(143-iloop);

      dac_samples[iloop] =
	((DataLoad + 3) << 24) | ((DataLoad + 2) << 16) |
	((DataLoad + 1) << 8) | (DataLoad);
    }

  vldLoadPulse32(islot, dac_samples, nsamples);

  /*
    Set calibration trigger
  */

  /* slow trigger */
  vldSetRandomPulser(islot, 5, 1);

  /* Set the FP_trigger out to 128ns wide */
  vldSetTriggerDelayWidth(islot, 35, 0, 31);

  /*
     1: always high (should be Periodic trigger),
     2: random trigger only;
     4: trigger_sequence (not implemented yet)
     16: FP_trigger,
  */
  vldSetTriggerSourceMask(islot, 2);
  printf("\n enabled the random trigger at ~100 Hz \n");

  /* loop over connectors */
  for(iconnector = 0; iconnector < 5; iconnector++)
    {
      printf(" Connect the LED cable to connector# %1d (1 for Top_FP, 2 for Mid_FP, 3 for Bottom_FP, 4 for Top_in, 5 for Bottom_in) \n", iconnector + 1);
      printf(" Ready? (hit 'enter'): \n");
      getchar();

      for(iloop = 1; iloop < 38; iloop++)
	{			// loop over the 36 channels, and reset the register to 0 at the end
	  if(iloop < 19)
	    {
	      DataLow = (1 << iloop) + 1;
	      DataHigh = 0;
	    }
	  else
	    {
	      if(iloop < 37)
		{
		  DataLow = 0;
		  DataHigh = (1 << (iloop - 18)) + 1;
		}
	      else
		{
		  DataLow = 0;
		  DataHigh = 0;
		}
	    }

	  vldSetChannelMask(islot, iconnector, DataLow, DataHigh);

	  // pause the pulser
	  printf("\n Channel# %d is pulsing....  Ready to the next channel?... \n",
	     iloop);
	  getchar();

	}

      /* loop over the bleaching amplitude */
      printf(" To test the bleaching of the connector\n");
      printf(" All the LEDs should be OFF, then ON with brightness decreasing \n");

      for(iBleach = 7; iBleach < 17; iBleach++)
	{
	  /* start the bleaching of the connector */
	  vldSetBleachCurrent(islot, iconnector, (iBleach % 16), 1);

	  if(iBleach == 16)
	    vldSetBleachCurrent(islot, iconnector, 0, 0);

	  /* pause the pulser */
	  printf("\n Bleaching setting %02x....  Ready to the next value? (hit 'enter'): \n",
	     iBleach);
	  printf(" Bleaching OK? ... \n");
	  getchar();
	}
      printf(" Disabled the bleaching \n");
    }
  printf("\n All the channels are tested \n");

  if(dac_samples)
    free(dac_samples);

}
