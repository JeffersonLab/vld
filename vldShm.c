/**
 * @copyright Copyright 2023, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *            Fax:   (757) 269-5800             Newport News, VA 23606
 *
 * @file      vldShm.c
 * @brief     Shared Memory routines for the VLD Library
 *
 */

#define _GNU_SOURCE

#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <errno.h>
#include <byteswap.h>
#include "jvme.h"
#include "vldShm.h"

static bool vldQuietFlag = true;
static char* shm_name_VLD = "/vld";
static pid_t processID;

static vld_shm_t *p_sync=NULL;
/* mmap'd address of shared memory mutex */
void *addr_shm = NULL;

extern int32_t nVLD;
extern int32_t vldID[MAX_VME_SLOTS+1];

static int32_t
vldShmMutexInit()
{
  if(!vldQuietFlag)
    printf("%s: Initializing vldShm mutex\n",__func__);

  p_sync->lockPID = 0;
  if(pthread_mutexattr_init(&p_sync->m_attr)<0)
    {
      perror("pthread_mutexattr_init");
      printf("%s: ERROR:  Unable to initialized mutex attribute\n",__func__);
      return ERROR;
    }
  if(pthread_mutexattr_setpshared(&p_sync->m_attr, PTHREAD_PROCESS_SHARED)<0)
    {
      perror("pthread_mutexattr_setpshared");
      printf("%s: ERROR:  Unable to set shared attribute\n",__func__);
      return ERROR;
    }
  if(pthread_mutexattr_setrobust_np(&p_sync->m_attr, PTHREAD_MUTEX_ROBUST_NP)<0)
    {
      perror("pthread_mutexattr_setrobust_np");
      printf("%s: ERROR:  Unable to set robust attribute\n",__func__);
      return ERROR;
    }
  if(pthread_mutex_init(&(p_sync->mutex), &p_sync->m_attr)<0)
    {
      perror("pthread_mutex_init");
      printf("%s: ERROR:  Unable to initialize shared mutex\n",__func__);
      return ERROR;
    }

  return OK;
}

/*!
  Routine to create (if needed) a shared mutex for VME Bus locking

  @return 0, if successful. -1, otherwise.
*/
int32_t
vldShmCreateLockShm()
{
  int32_t fd_shm;
  int32_t needMutexInit=0, stat=0;
  mode_t prev_mode;

  /* Save the process ID of the current process */
  processID = getpid();

  /* First check to see if the file already exists */
  fd_shm = shm_open(shm_name_VLD, O_RDWR,
		    S_IRUSR | S_IWUSR |
		    S_IRGRP | S_IWGRP |
		    S_IROTH | S_IWOTH );
  if(fd_shm<0)
    {
      /* Bad file handler.. */
      if(errno == ENOENT)
	{
	  needMutexInit=1;
	}
      else
	{
	  perror("shm_open");
	  printf(" %s: ERROR: Unable to open shared memory\n",__func__);
	  return ERROR;
	}
    }

  if(needMutexInit)
    {
      if(!vldQuietFlag)
	printf("%s: Creating vldShm shared memory file\n",__func__);

      prev_mode = umask(0); /* need to override the current umask, if necessary */

      /* Create and map 'mutex' shared memory */
      fd_shm = shm_open(shm_name_VLD, O_CREAT|O_RDWR,
			S_IRUSR | S_IWUSR |
			S_IRGRP | S_IWGRP |
			S_IROTH | S_IWOTH );
      umask(prev_mode);
      if(fd_shm<0)
	{
	  perror("shm_open");
	  printf(" %s: ERROR: Unable to open shared memory\n",__func__);
	  return ERROR;
	}
      ftruncate(fd_shm, sizeof(vld_shm_t));
    }

  addr_shm = mmap(0, sizeof(vld_shm_t), PROT_READ|PROT_WRITE, MAP_SHARED, fd_shm, 0);
  if(addr_shm<0)
    {
      perror("mmap");
      printf("%s: ERROR: Unable to mmap shared memory\n",__func__);
      return ERROR;
    }
  p_sync = addr_shm;

  if(needMutexInit)
    {
      stat = vldShmMutexInit();
      if(stat==ERROR)
	{
	  printf("%s: ERROR Initializing vldShm Mutex\n",
		 __func__);
	  return ERROR;
	}
    }

  if(!vldQuietFlag)
    printf("%s: vldShm shared memory mutex initialized\n",__func__);
  return OK;
}

/*!
  Routine to destroy the shared mutex created by vldShmCreateLockShm()

  @return 0, if successful. -1, otherwise.
*/
int32_t
vldShmKillLockShm(int32_t kflag)
{
  int32_t rval = OK;

  if(munmap(addr_shm, sizeof(vld_shm_t))<0)
    perror("munmap");

  if(kflag==1)
    {
      if(pthread_mutexattr_destroy(&p_sync->m_attr)<0)
	perror("pthread_mutexattr_destroy");

      if(pthread_mutex_destroy(&p_sync->mutex)<0)
	perror("pthread_mutex_destroy");

      if(shm_unlink(shm_name_VLD)<0)
	perror("shm_unlink");

      if(!vldQuietFlag)
	printf("%s: vldShm shared memory mutex destroyed\n",__func__);
    }
  return rval;
}

/*!
  Routine to lock the shared mutex created by vldShmCreateLockShm()

  @return 0, if successful. -1 or other error code otherwise.
*/
int32_t
vldShmLock()
{
  int32_t rval;

  if(p_sync!=NULL)
    {
      rval = pthread_mutex_lock(&(p_sync->mutex));
      if(rval<0)
	{
	  perror("pthread_mutex_lock");
	  printf("%s: ERROR locking vldShm\n",__func__);
	}
      else if (rval>0)
	{
	  printf("%s: ERROR: %s\n",__func__,
		 (rval==EINVAL)?"EINVAL":
		 (rval==EBUSY)?"EBUSY":
		 (rval==EAGAIN)?"EAGAIN":
		 (rval==EPERM)?"EPERM":
		 (rval==EOWNERDEAD)?"EOWNERDEAD":
		 (rval==ENOTRECOVERABLE)?"ENOTRECOVERABLE":
		 "Undefined");
	  if(rval==EOWNERDEAD)
	    {
	      printf("%s: WARN: Previous owner of vldShm (mutex) died unexpectedly\n",
		     __func__);
	      printf("  Attempting to recover..\n");
	      if(pthread_mutex_consistent_np(&(p_sync->mutex))<0)
		{
		  perror("pthread_mutex_consistent_np");
		}
	      else
		{
		  printf("  Successful!\n");
		  rval=OK;
		}
	    }
	  if(rval==ENOTRECOVERABLE)
	    {
	      printf("%s: ERROR: vldShm mutex in an unrecoverable state!\n",
		     __func__);
	    }
	}
      else
	{
	  p_sync->lockPID = processID;
	}
    }
  else
    {
      printf("%s: ERROR: vldShmLock not initialized.\n",__func__);
      return ERROR;
    }
  return rval;
}

/*!
  Routine to try to lock the shared mutex created by vldShmCreateLockShm()

  @return 0, if successful. -1 or other error code otherwise.
*/
int32_t
vldShmTryLock()
{
  int32_t rval=ERROR;

  if(p_sync!=NULL)
    {
      rval = pthread_mutex_trylock(&(p_sync->mutex));
      if(rval<0)
	{
	  perror("pthread_mutex_trylock");
	}
      else if(rval>0)
	{
	  printf("%s: ERROR: %s\n",__func__,
		 (rval==EINVAL)?"EINVAL":
		 (rval==EBUSY)?"EBUSY":
		 (rval==EAGAIN)?"EAGAIN":
		 (rval==EPERM)?"EPERM":
		 (rval==EOWNERDEAD)?"EOWNERDEAD":
		 (rval==ENOTRECOVERABLE)?"ENOTRECOVERABLE":
		 "Undefined");
	  if(rval==EBUSY)
	    {
	      printf("%s: Locked vldShm (mutex) owned by PID = %d\n",
		     __func__, p_sync->lockPID);
	    }
	  if(rval==EOWNERDEAD)
	    {
	      printf("%s: WARN: Previous owner of vldShm (mutex) died unexpectedly\n",
		     __func__);
	      printf("  Attempting to recover..\n");
	      if(pthread_mutex_consistent_np(&(p_sync->mutex))<0)
		{
		  perror("pthread_mutex_consistent_np");
		}
	      else
		{
		  printf("  Successful!\n");
		  rval=OK;
		}
	    }
	  if(rval==ENOTRECOVERABLE)
	    {
	      printf("%s: ERROR: vldShm mutex in an unrecoverable state!\n",
		     __func__);
	    }
	}
      else
	{
	  p_sync->lockPID = processID;
	}
    }
  else
    {
      printf("%s: ERROR: vldShm mutex not initialized\n",__func__);
      return ERROR;
    }

  return rval;

}

/*!
  Routine to lock the shared mutex created by vldShmCreateLockShm()

  @return 0, if successful. -1 or other error code otherwise.
*/

int32_t
vldShmTimedLock(int32_t time_seconds)
{
  int32_t rval=ERROR;
  struct timespec timeout;

  if(p_sync!=NULL)
    {
      clock_gettime(CLOCK_REALTIME, &timeout);
      timeout.tv_nsec = 0;
      timeout.tv_sec += time_seconds;

      rval = pthread_mutex_timedlock(&p_sync->mutex,&timeout);
      if(rval<0)
	{
	  perror("pthread_mutex_timedlock");
	}
      else if(rval>0)
	{
	  printf("%s: ERROR: %s\n",__func__,
		 (rval==EINVAL)?"EINVAL":
		 (rval==EBUSY)?"EBUSY":
		 (rval==EAGAIN)?"EAGAIN":
		 (rval==ETIMEDOUT)?"ETIMEDOUT":
		 (rval==EPERM)?"EPERM":
		 (rval==EOWNERDEAD)?"EOWNERDEAD":
		 (rval==ENOTRECOVERABLE)?"ENOTRECOVERABLE":
		 "Undefined");
	  if(rval==ETIMEDOUT)
	    {
	      printf("%s: Timeout: Locked vldShm (mutex) owned by PID = %d\n",
		     __func__, p_sync->lockPID);
	    }
	  if(rval==EOWNERDEAD)
	    {
	      printf("%s: WARN: Previous owner of vldShm (mutex) died unexpectedly\n",
		     __func__);
	      printf("  Attempting to recover..\n");
	      if(pthread_mutex_consistent_np(&(p_sync->mutex))<0)
		{
		  perror("pthread_mutex_consistent_np");
		}
	      else
		{
		  printf("  Successful!\n");
		  rval=OK;
		}
	    }
	}
      else
	{
	  p_sync->lockPID = processID;
	}
    }
  else
    {
      printf("%s: ERROR: vldShm mutex not initialized\n",__func__);
      return ERROR;
    }

  return rval;

}

/*!
  Routine to unlock the shared mutex created by vldShmCreateLockShm()

  @return 0, if successful. -1 or other error code otherwise.
*/
int32_t
vldShmUnlock()
{
  int32_t rval=0;
  if(p_sync!=NULL)
    {
      p_sync->lockPID = 0;
      rval = pthread_mutex_unlock(&p_sync->mutex);
      if(rval<0)
	{
	  perror("pthread_mutex_unlock");
	}
      else if(rval>0)
	{
	  printf("%s: ERROR: %s \n",__func__,
		   (rval==EINVAL)?"EINVAL":
		   (rval==EBUSY)?"EBUSY":
		   (rval==EAGAIN)?"EAGAIN":
		   (rval==EPERM)?"EPERM":
		   "Undefined");
	}
    }
  else
    {
      printf("%s: ERROR: vldShm mutex not initialized.\n",__func__);
      return ERROR;
    }
  return rval;
}

/*!
  Routine to check the "health" of the mutex created with vldShmCreateLockShm()

  If the mutex is found to be stale (Owner of the lock has died), it will
  be recovered.

  @param time_seconds     How many seconds to wait for mutex to unlock when testing

  @return 0, if successful. -1, otherwise.
*/
int32_t
vldShmCheckMutexHealth(int32_t time_seconds)
{
  int32_t rval=0, busy_rval=0;

  if(p_sync!=NULL)
    {
      if(!vldQuietFlag)
	printf("%s: Checking health of vldShm shared mutex...\n",
	       __func__);

      /* Try the Mutex to see if it's state (locked/unlocked) */
      printf(" * ");
      rval = vldShmTryLock();
      switch (rval)
	{
	case -1: /* Error */
	  printf("%s: rval = %d: Not sure what to do here\n",
		 __func__,rval);
	  break;
	case 0:  /* Success - Got the lock */
	  if(!vldQuietFlag)
	    printf(" * ");

	  rval = vldShmUnlock();
	  break;

	case EAGAIN: /* Bad mutex attribute initialization */
	case EINVAL: /* Bad mutex attribute initialization */
	  /* Re-Init here */
	  if(!vldQuietFlag)
	    printf(" * ");

	  rval = vldShmMutexInit();
	  break;

	case EBUSY: /* It's Locked */
	  {
	    /* Check to see if we can unlock it */
	    if(!vldQuietFlag)
	      printf(" * ");

	    busy_rval = vldShmUnlock();
	    switch(busy_rval)
	      {
	      case OK:     /* Got the unlock */
		rval=busy_rval;
		break;

	      case EAGAIN: /* Bad mutex attribute initialization */
	      case EINVAL: /* Bad mutex attribute initialization */
		/* Re-Init here */
		if(!vldQuietFlag)
		  printf(" * ");

		rval = vldShmMutexInit();
		break;

	      case EPERM: /* Mutex owned by another thread */
		{
		  /* Check to see if we can get the lock within 5 seconds */
		  if(!vldQuietFlag)
		    printf(" * ");

		  busy_rval = vldShmTimedLock(time_seconds);
		  switch(busy_rval)
		    {
		    case -1: /* Error */
		      printf("%s: rval = %d: Not sure what to do here\n",
			     __func__,busy_rval);
		      break;

		    case 0:  /* Success - Got the lock */
		      printf(" * ");
		      rval = vldShmUnlock();
		      break;

		    case EAGAIN: /* Bad mutex attribute initialization */
		    case EINVAL: /* Bad mutex attribute initialization */
		      /* Re-Init here */
		      if(!vldQuietFlag)
			printf(" * ");

		      rval = vldShmMutexInit();
		      break;

		    case ETIMEDOUT: /* Timeout getting the lock */
		      /* Re-Init here */
		      if(!vldQuietFlag)
			printf(" * ");

		      rval = vldShmMutexInit();
		      break;

		    default:
		      printf("%s: Undefined return from pthread_mutex_timedlock (%d)\n",
			     __func__,busy_rval);
		      rval=busy_rval;

		    }

		}
		break;

	      default:
		printf("%s: Undefined return from vldShmUnlock (%d)\n",
		       __func__,busy_rval);
		      rval=busy_rval;

	      }

	  }
	  break;

	default:
	  printf("%s: Undefined return from vldShmTryLock (%d)\n",
		 __func__,rval);

	}

      if(rval==OK)
	{
	  if(!vldQuietFlag)
	    printf("%s: Mutex Clean and Unlocked\n",__func__);
	}
      else
	{
	  printf("%s: Mutex is NOT usable\n",__func__);
	}

    }
  else
    {
      printf("%s: INFO: vldShm Mutex not initialized\n",
	     __func__);
      return ERROR;
    }

  return rval;
}

int32_t
vldShmSetChannelMask(int32_t id,
		     uint32_t connector, uint32_t lochanEnableMask, uint32_t hichanEnableMask)
{
  uint32_t rval = 0;

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

  if(vldShmLock() != OK)
    return ERROR;

  p_sync->slot_data[id].connector[connector].lo_channel_mask = lochanEnableMask;
  p_sync->slot_data[id].connector[connector].hi_channel_mask = hichanEnableMask;
  p_sync->write_count++;

  vldShmUnlock();

  return OK;

}

int32_t
vldShmReadBlock(volatile uint32_t *data, uint32_t nwords)
{

  int32_t rval = 0, ivld = 0, id = 0, iconn = 0;
  uint32_t block_header = 0, block_trailer = 0, slot_header = 0, connector_data = 0;

  if(vldShmLock() != OK)
    return ERROR;

  /* Block Header */
  block_header = (1 << 31) | (0 << 27) | (nVLD << 16) |
    ((p_sync->write_count & 0xFF) << 8) | (++p_sync->read_count & 0xFF);

  data[rval++] = bswap_32(block_header);


  for(ivld = 0; ivld < nVLD; ivld++)
    {
      id = vldID[ivld];

      /* Slot Header */
      slot_header = (1 << 31) | (2 << 27) | (id << 16) | 2 * 4;

      data[rval++] = bswap_32(slot_header);

      for(iconn = 0; iconn < 4; iconn++)
	{
	  /* Connector data: lo */
	  connector_data = (iconn << 28) | (0 << 24) |
	    p_sync->slot_data[id].connector[iconn].lo_channel_mask;
	  data[rval++] = bswap_32(connector_data);

	  /* Connector data: hi */
	  connector_data = (iconn << 28) | (1 << 24) |
	    p_sync->slot_data[id].connector[iconn].hi_channel_mask;
	  data[rval++] = bswap_32(connector_data);
	}
    }

  /* Block Trailer */
  block_trailer = (1 << 31) | (1 << 27) | (rval + 1);
  data[rval++] = bswap_32(block_trailer);

  vldShmUnlock();

  return rval;
}

int32_t
vldShmResetCounts(int32_t reset_read_count, int32_t reset_write_count)
{
  if(vldShmLock() != OK)
    return ERROR;

  if(reset_read_count)
    p_sync->read_count = 0;

  if(reset_write_count)
    p_sync->write_count = 0;

  vldShmUnlock();

  return OK;
}
