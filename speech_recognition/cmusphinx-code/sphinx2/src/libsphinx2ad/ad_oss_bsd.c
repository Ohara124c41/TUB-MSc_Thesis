/* ====================================================================
 * Copyright (c) 1999-2001 Carnegie Mellon University.  All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer. 
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * This work was supported in part by funding from the Defense Advanced 
 * Research Projects Agency and the National Science Foundation of the 
 * United States of America, and the CMU Sphinx Speech Consortium.
 *
 * THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY ``AS IS'' AND 
 * ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY
 * NOR ITS EMPLOYEES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ====================================================================
 *
 */
/* Sphinx II libad (FreeBSD)
 * ^^^^^^^^^^^^^^^^^^^^^^^
 * $Id: ad_oss_bsd.c 5469 2006-03-10 04:03:51Z dhdfu $
 *
 * John G. Dorsey (jd5q+@andrew.cmu.edu)
 * Engineering Design Research Center
 * Carnegie Mellon University
 * ********************************************************************
 * 
 * REVISION HISTORY
 *
 * 28-Jan-2000  Alan W Black (awb@cs.cmu.edu. added FreeBSD support (by
                simply cloning the linux support).
 * 09-Aug-1999  Kevin Lenzo (lenzo@cs.cmu.edu) at Cernegie Mellon University.
 *              Incorporated nickr@cs.cmu.edu's changes (marked below) and
 *              SPS_EPSILON to allow for sample rates that are "close enough".
 * 
 * 15-Jun-1999	M. K. Ravishankar (rkm@cs.cmu.edu) Consolidated all ad functions into
 *		this one file.  Added ad_open_sps().
 * 		Other cosmetic changes for consistency (e.g., use of err.h).
 * 
 * 18-May-1999	Kevin Lenzo (lenzo@cs.cmu.edu) added <errno.h>.
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <soundcard.h>
#include <errno.h>

#include "s2types.h"
#include "ad.h"

#define AUDIO_FORMAT AFMT_S16_LE	/* 16-bit signed, little endian */
#define INPUT_GAIN   (80)

#define SPS_EPSILON   200

#ifndef SNDCTL_DSP_SETDUPLEX
#define SNDCTL_DSP_SETDUPLEX -1
#endif

ad_rec_t *ad_open_sps (int32 sps) {
    ad_rec_t *handle;
    int32 dspFD, mixerFD;
    int32 nonBlocking=1, sourceMic=1, inputGain=INPUT_GAIN;
    int32 audioFormat=AUDIO_FORMAT;
    int32 dspCaps=0;
    int32 sampleRate;
    
    sampleRate = sps;
    
    /* Used to have O_NDELAY. */
    if((dspFD = open ("/dev/dsp", O_RDONLY))<0){
	if (errno == EBUSY)
	    fprintf(stderr, "Audio device busy\n");
	else
	    fprintf(stderr, "Failed to open audio device: %s\n", strerror(errno));
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_SYNC, 0) < 0){
	fprintf(stderr, "Audio ioctl(SYNC) failed: %s\n", strerror(errno));
	close (dspFD);
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_RESET, 0) < 0){
	fprintf(stderr, "Audio ioctl(RESET) failed: %s\n", strerror(errno));
	close (dspFD);
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_SETFMT, &audioFormat) < 0){
	fprintf(stderr, "Audio ioctl(SETFMT 0x%x) failed: %s\n", audioFormat, strerror(errno));
	close (dspFD);
	return NULL;
    }
    if (audioFormat != AUDIO_FORMAT) {
	fprintf(stderr, "Audio ioctl(SETFMT): 0x%x, expected: 0x%x\n", audioFormat, AUDIO_FORMAT);
	close (dspFD);
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_SPEED, &sampleRate) < 0) {
	fprintf(stderr, "Audio ioctl(SPEED %d) failed %s\n", sampleRate, strerror(errno));
	close (dspFD);
	return NULL;
    }
    if (sampleRate != sps) {
	fprintf(stderr, "Audio ioctl(SPEED): %d, expected: %d\n", sampleRate, sps);
	close (dspFD);
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_NONBLOCK, &nonBlocking) < 0) {
	fprintf(stderr, "ioctl(NONBLOCK) failed: %s\n", strerror(errno));
	close (dspFD);
	return NULL;
    }
    
    if (ioctl (dspFD, SNDCTL_DSP_GETCAPS, &dspCaps) < 0) {
	fprintf(stderr, "ioctl(GETCAPS) failed: %s\n", strerror(errno));
	close (dspFD);
	return NULL;
    }
    
#if 0
    printf("DSP Revision %d:\n", dspCaps&DSP_CAP_REVISION);
    printf("DSP %s duplex capability.\n", (dspCaps&DSP_CAP_DUPLEX)?"has":"does not have");
    printf("DSP %s real time capability.\n", (dspCaps&DSP_CAP_REALTIME)?"has":"does not have");
    printf("DSP %s batch capability.\n", (dspCaps&DSP_CAP_BATCH)?"has":"does not have");
    printf("DSP %s coprocessor capability.\n", (dspCaps&DSP_CAP_COPROC)?"has":"does not have");
    printf("DSP %s trigger capability.\n", (dspCaps&DSP_CAP_TRIGGER)?"has":"does not have");
    printf("DSP %s memory map capability.\n", (dspCaps&DSP_CAP_MMAP)?"has":"does not have");
#endif
    
    if ((dspCaps & DSP_CAP_DUPLEX) && (ioctl (dspFD, SNDCTL_DSP_SETDUPLEX, 0) < 0))
	fprintf(stderr, "ioctl(SETDUPLEX) failed: %s\n", strerror(errno));
    
  /* Patched by N. Roy (nickr@ri.cmu.edu), 99/7/23. 
     Previously, mixer was set through dspFD. This is incorrect. Should
     be set through mixerFD, /dev/mixer. 
     Also, only the left channel volume was being set.
  */

  if((mixerFD=open("/dev/mixer", O_RDONLY))<0){
    if(errno==EBUSY){
      fprintf(stderr, "%s %d: mixer device busy.\n", __FILE__, __LINE__);
      fprintf(stderr, "%s %d: Using current setting.\n", __FILE__, __LINE__);
    } else {
      fprintf(stderr, "%s %d: %s\n", __FILE__, __LINE__, strerror(errno));
      exit(1);
    }
  }

  if (mixerFD >= 0) {
    if(ioctl(mixerFD, SOUND_MIXER_WRITE_RECSRC, &sourceMic)<0) {
      if (errno == ENXIO) 
        fprintf(stderr, "%s %d: can't set mic source for this device.\n",
                __FILE__, __LINE__);
      else {
        fprintf(stderr, "%s %d: mixer set to mic: %s\n", __FILE__, __LINE__,
                strerror(errno));
        exit(1);
      }
    }

    /* Set the same gain for left and right channels. */

    inputGain = inputGain << 8 | inputGain;
    if(ioctl(mixerFD, SOUND_MIXER_WRITE_MIC, &inputGain)<0){
      fprintf(stderr, "%s %d: mixer input gain to %d: %s\n", __FILE__, __LINE__,
              strerror(errno));
      exit(1);
    }

    close(mixerFD);
  }

  if ((handle = (ad_rec_t *) calloc (1, sizeof(ad_rec_t))) == NULL) {
	fprintf(stderr, "calloc(%d) failed\n", sizeof(ad_rec_t));
	abort();
  }
    
  handle->dspFD = dspFD;
  handle->recording = 0;
  handle->sps = sps;
  handle->bps = sizeof(int16);

  return(handle);
}

ad_rec_t *ad_open ( void )
{
    return ad_open_sps (DEFAULT_SAMPLES_PER_SEC);
}

int32 ad_close (ad_rec_t *handle)
{
    if (handle->dspFD < 0)
	return AD_ERR_NOT_OPEN;
    
    if (handle->recording) {
	if (ad_stop_rec (handle) < 0)
	    return AD_ERR_GEN;
    }
    
    close (handle->dspFD);
    free(handle);
    
    return(0);
}

int32 ad_start_rec (ad_rec_t *handle)
{
    if (handle->dspFD < 0)
	return AD_ERR_NOT_OPEN;
    
    if (handle->recording)
	return AD_ERR_GEN;
    
    /* Sample rate, format, input mix settings, &c. are configured
     * with ioctl(2) calls under Linux. It makes more sense to handle
     * these at device open time and consider the state of the device
     * to be fixed until closed.
     */
    
    handle->recording = 1;

    /* rkm@cs: This doesn't actually do anything.  How do we turn recording on/off? */

    return(0);
}

int32 ad_stop_rec (ad_rec_t *handle)
{
    if (handle->dspFD < 0)
	return AD_ERR_NOT_OPEN;
    
    if (! handle->recording)
	return AD_ERR_GEN;
    
    if (ioctl (handle->dspFD, SNDCTL_DSP_SYNC, 0) < 0) {
	fprintf(stderr, "Audio ioctl(SYNC) failed: %s\n", strerror(errno));
	return AD_ERR_GEN;
    }
    
    handle->recording = 0;
    
    return (0);
}

int32 ad_read (ad_rec_t *handle, int16 *buf, int32 max)
{
    int32 length;
    
    length = max * handle->bps;		/* #samples -> #bytes */
    
    if ((length = read (handle->dspFD, buf, length)) > 0) {
#if 0
	if ((length % handle->bps) != 0)
	    fprintf(stderr, "Audio read returned non-integral #sample bytes (%d)\n", length);
#endif
	length /= handle->bps;
    }
    
    if (length < 0) {
	fprintf(stderr, "Audio read error\n");
	return AD_ERR_GEN;
    }
    
    if ((length == 0) && (! handle->recording))
	return AD_EOF;
    
    return length;
}
