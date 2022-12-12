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
 * @file      VLDtest7.c
 * @brief     Ported version of William Gu's VLDtest7
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include "jvme.h"
#include "vldLib.h"

void VLDtest7(unsigned int islot);

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

  VLDtest7(slot);

  vmeCloseDefaultWindows();

  return OK;
}



// VLD Front Panel in test
void
VLDtest7(unsigned int islot)
{
  unsigned long BaseAdd, *laddr, TestReg, DataLow, DataHigh, nsamples = 512;
  int iloop, iconnector, iwaitsum, ReadyNext;
  uint32_t DataLoad, *dac_samples;

  printf(" Calibration pulse amplitude test \n");
  printf(" Probe the Channel#1 of each connector with an oscilloscpoe \n");

  /* Initialize VLD at islot */
  vldInit(islot << 19, 0, 1, 0);

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

  /*
    Enable Calibration
  */

  DataLow  = 3;
  DataHigh = 0;
  for (iconnector = 0; iconnector < 5; iconnector++)
    {
      vldSetChannelMask(islot, iconnector, DataLow, DataHigh);
    }

  /* pause the pulser */
  printf("\n Does the Calibration pulse look OK? (hit 'enter'): \n");
  getchar();

  printf("\n Test the on-board oscillator \n");
  vldSetClockSource(islot, 0);
  printf("\n Does the Calibration pulse look OK? (hit 'enter'): \n");
  getchar();

  printf("\n Connect the FP_clock input to a clock source (TI/FTDC/pin#9+/10-), ready? \n");
  getchar();

  /* FP clock input */
  vldSetClockSource(islot, 1);

  printf("\n Does the Calibration pulse look OK? (hit 'enter'): \n");
  getchar();

  vldSetClockSource(islot, 0);

  /* Start the trigger */
  printf("\n Connect the FP_trigger input to a Trigger source (TI/FADC/pin#3+/4-), ready? \n");
  getchar();

  /* FP trigger enabled */
  vldSetTriggerSourceMask(islot, 0x10);
  printf("\n Does the Calibration pulse look OK? (hit 'enter'): \n");
  getchar();

  /* disable the trigger: */
  vldSetTriggerSourceMask(islot, 0);
  printf("\n Trigger disabled \n");

  if(dac_samples)
    free(dac_samples);

}
