/*
 * File:
 *    vldShmTest
 *
 * Description:
 *    Test VLD shared memory routines
 *
 *
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "jvme.h"
#include "vldLib.h"
#include "vldShm.h"

int32_t
main(int32_t argc, char *argv[])
{

  int32_t stat;
  uint32_t address=0;

  if (argc > 1)
    {
      address = (uint32_t) strtoll(argv[1],NULL,16)&0xffffffff;
    }

  printf("\n %s: address = 0x%08x\n", argv[0], address);
  printf("----------------------------\n");

  stat = vmeOpenDefaultWindows();
  if(stat != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();

  vldInit(address<<19, 0, 1, 0);
  vldGStatus(1);

  vldSetChannelMask(8, 0, 0xff00, 0x00ff);
  vldSetChannelMask(8, 2, 0x2200, 0x0022);
  vldSetChannelMask(8, 3, 0x30000, 0x00003);

  uint32_t data[256], retWords = 0;

  retWords = vldShmReadBlock(data, 256);

  int32_t idata = 0;
  for(idata = 0; idata < retWords; idata++)
    {
      printf("%4d: 0x%08x\n", idata, data[idata]);
    }


 CLOSE:

  vmeBusUnlock();

  vmeClearException(1);

  stat = vmeCloseDefaultWindows();
  if (stat != OK)
    {
      printf("vmeCloseDefaultWindows failed: code 0x%08x\n",stat);
      return -1;
    }

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k vldShmTest"
  End:
 */
