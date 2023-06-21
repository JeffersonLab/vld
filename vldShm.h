#pragma once
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
 * @file      vldShm.h
 * @brief     Header for Shared Memory routines for the VLD Library
 *
 */

#include <stdint.h>
#include <pthread.h>

#ifndef MAX_VME_SLOTS
#define MAX_VME_SLOTS 21
#endif

typedef struct
{
  uint32_t lo_channel_mask;
  uint32_t hi_channel_mask;
} vld_connector_t;

typedef struct
{
  vld_connector_t connector[6];
} vld_data_t;

typedef struct shared_memory_mutex
{
  pthread_mutex_t mutex;
  pthread_mutexattr_t m_attr;
  pid_t lockPID;
  uint32_t write_count;
  uint32_t read_count;
  vld_data_t slot_data[MAX_VME_SLOTS+1];
} vld_shm_t;

/* Function prototypes */
int32_t vldShmCreateLockShm();
int32_t vldShmKillLockShm(int32_t kflag);
int32_t vldShmLock();
int32_t vldShmTryLock();
int32_t vldShmTimedLock(int32_t time_seconds);
int32_t vldShmUnlock();
int32_t vldShmCheckMutexHealth(int32_t time_seconds);
int32_t vldShmSetChannelMask(int32_t id, uint32_t connector, uint32_t lochanEnableMask, uint32_t hichanEnableMask);
int32_t vldShmReadBlock(volatile uint32_t *data, uint32_t nwords);
