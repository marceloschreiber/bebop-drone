/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
#ifndef _SDK_EXAMPLE_BD_H_
#define _SDK_EXAMPLE_BD_H_


#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "ihm.h"
#include "DecoderManager.h"
#include <libARCommands/ARCommands.h>

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>

#include <libARSAL/ARSAL.h>
#include <libARSAL/ARSAL_Print.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARStream/ARStream.h>

typedef struct
{
    int flag;
    int roll;
    int pitch;
    int yaw;
    int gaz;
}BD_PCMD_t;

typedef struct _ARDrone3CameraData_t_
{
    int tilt;
    int pan;
} BD_Cam_t;

typedef struct READER_THREAD_DATA_t READER_THREAD_DATA_t;

typedef struct RawFrame_t RawFrame_t;
typedef struct
{
    ARNETWORKAL_Manager_t *alManager;
    ARNETWORK_Manager_t *netManager;
    ARSTREAM_Reader_t *streamReader;
    ARSAL_Thread_t looperThread;
    ARSAL_Thread_t rxThread;
    ARSAL_Thread_t txThread;
    ARSAL_Thread_t videoTxThread;
    ARSAL_Thread_t videoRxThread;
    int d2cPort;
    int c2dPort;
    int arstreamFragSize;
    int arstreamFragNb;
    int arstreamAckDelay;
    uint8_t *videoFrame;
    uint32_t videoFrameSize;

    BD_PCMD_t dataPCMD;
    BD_Cam_t dataCam;

    ARCODECS_Manager_t *decoder;
    int decodingCanceled;
    ARSAL_Thread_t decodingThread;

    int hasReceivedFirstIFrame;
    RawFrame_t **freeRawFramePool;
    int rawFramePoolCapacity;
    int lastRawFrameFreeIdx;
    RawFrame_t **rawFrameFifo;
    int fifoReadIdx;
    int fifoWriteIdx;

    eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE flyingState;

    FILE *video_out;

    ARSAL_Mutex_t mutex;

    ARSAL_Thread_t *readerThreads;
    READER_THREAD_DATA_t *readerThreadsData;
    int run;

    IHM_t *ihm;
} BD_MANAGER_t;

struct READER_THREAD_DATA_t
{
    BD_MANAGER_t *deviceManager;
    int readerBufferId;
};


/** Old main function **/
int BebopDroneDecodeStreamMain (BD_MANAGER_t *deviceManager);

/** Connection part **/
int ardiscoveryConnect (BD_MANAGER_t *deviceManager);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_SendJsonCallback (uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ARDISCOVERY_Connection_ReceiveJsonCallback (uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);

/** Network part **/
int startNetwork (BD_MANAGER_t *deviceManager);
void stopNetwork (BD_MANAGER_t *deviceManager);
void onDisconnectNetwork (ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);

/** Video part **/
int startVideo (BD_MANAGER_t *deviceManager);
void stopVideo (BD_MANAGER_t *deviceManager);
uint8_t *frameCompleteCallback (eARSTREAM_READER_CAUSE cause, uint8_t *frame, uint32_t frameSize, int numberOfSkippedFrames, int isFlushFrame, uint32_t *newBufferCapacity, void *custom);

/** decoding part **/
int startDecoder (BD_MANAGER_t *deviceManager);
void stopDecoder (BD_MANAGER_t *deviceManager);
int getNextDataCallback(uint8_t **data, void *customData);
RawFrame_t *getFreeRawFrame(BD_MANAGER_t *deviceManager);
void addFreeRawFrameToFifo(BD_MANAGER_t *deviceManager, RawFrame_t *rawFrame);
void flushFifo(BD_MANAGER_t *deviceManager);
void putRawFrameBackToPool(BD_MANAGER_t *deviceManager, int fifoIdx);
RawFrame_t *getFrameFromData(BD_MANAGER_t *deviceManager, uint8_t *data);

/** Commands part **/
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
int sendPCMD(BD_MANAGER_t *deviceManager);
int sendCameraOrientation(BD_MANAGER_t *deviceManager);
int sendDate(BD_MANAGER_t *deviceManager);
int sendAllStates(BD_MANAGER_t *deviceManager);
int sendAllSettings(BD_MANAGER_t *deviceManager);
int sendTakeoff(BD_MANAGER_t *deviceManager);
int sendLanding(BD_MANAGER_t *deviceManager);
int sendEmergency(BD_MANAGER_t *deviceManager);
int sendBeginStream(BD_MANAGER_t *deviceManager);

/** Commands callback part **/
void registerARCommandsCallbacks (BD_MANAGER_t *deviceManager);
void unregisterARCommandsCallbacks(void);
void batteryStateChangedCallback (uint8_t percent, void *custom);
void flyingStateChangedCallback (eARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE state, void *custom);

/** IHM callbacks **/
void onInputEvent (eIHM_INPUT_EVENT event, void *customData);
int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va);

#endif /* _SDK_EXAMPLE_BD_H_ */
