// Written by Josh Aas and Eric Rescorla
// TODO(ekr@rtfm.com): Need license.

#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>

#include <prthread.h>

#include "gmp-general.h"
#include "gmp-video-host.h"
#include "gmp-video-encode.h"
#include "gmp-video-decode.h"
#include "gmp-video-frame-i420.h"
#include "gmp-video-frame-encoded.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"
#include "param_svc.h"

#if defined(_MSC_VER)
#define PUBLIC_FUNC __declspec(dllexport)
#else
#define PUBLIC_FUNC
#endif


#define GMPLOG(c, x) std::cerr << c << ":" << x << std::endl;
#define GL_ERROR "Error"
#define GL_INFO  "Info"

class OpenH264VideoEncoder : public GMPVideoEncoder
{
 public:
  OpenH264VideoEncoder(GMPVideoHost *hostAPI) :
      host_(hostAPI),
      encoder_(nullptr),
      max_payload_size_(0),
      callback_(nullptr) {}

  virtual ~OpenH264VideoEncoder() {
    // TODO(ekr@rtfm.com)
  }

  virtual GMPVideoErr InitEncode(const GMPVideoCodec& codecSettings,
                                 GMPEncoderCallback* callback,
                                 int32_t numberOfCores,
                                 uint32_t maxPayloadSize) override {
    thread_ = PR_CreateThread(PR_USER_THREAD,
                              &OpenH264VideoEncoder::ThreadMain,
                              this,
                              PR_PRIORITY_NORMAL,
                              PR_GLOBAL_THREAD,
                              PR_JOINABLE_THREAD,
                              0);

    GMPLOG(GL_INFO, "PID " << getpid());

    int rv = CreateSVCEncoder(&encoder_);
    if (rv) {
      return GMPVideoGenericErr;
    }

    SEncParamExt param;
    memset(&param, 0, sizeof(param));

    GMPLOG(GL_INFO, "Initializing encoder at "
            << codecSettings.mWidth
            << "x"
            << codecSettings.mHeight
            << "@"
            << static_cast<int>(codecSettings.mMaxFramerate));

    // Translate parameters.
    param.iPicWidth = codecSettings.mWidth;
    param.iPicHeight = codecSettings.mHeight;
    param.iTargetBitrate = codecSettings.mStartBitrate * 1000;
    param.iTemporalLayerNum = 1;
    param.iSpatialLayerNum = 1;

    // TODO(ekr@rtfm.com). Scary conversion from unsigned char to float below.
    param.fMaxFrameRate = codecSettings.mMaxFramerate;
    param.iInputCsp = videoFormatI420;

    // Set up layers. Currently we have one layer.
    auto layer = &param.sSpatialLayers[0];

    layer->iVideoWidth = codecSettings.mWidth;
    layer->iVideoHeight = codecSettings.mHeight;
    layer->iSpatialBitrate = param.iTargetBitrate;
    layer->fFrameRate = param.fMaxFrameRate;

    // Based on guidance from Cisco.
    layer->sSliceCfg.sSliceArgument.uiSliceMbNum[0] = 1000;
    layer->sSliceCfg.sSliceArgument.uiSliceNum = 1;
    layer->sSliceCfg.sSliceArgument.uiSliceSizeConstraint = 1000;

    rv = encoder_->InitializeExt(&param);
    if (rv) {
      GMPLOG(GL_ERROR, "Couldn't initialize encoder");
      return GMPVideoGenericErr;
    }

    max_payload_size_ = maxPayloadSize;
    callback_ = callback;

    GMPLOG(GL_INFO, "Initialized encoder");

    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Encode(GMPVideoi420Frame& aInputFrame,
                             const GMPCodecSpecificInfo& aCodecSpecificInfo,
                             const std::vector<GMPVideoFrameType>* aFrameTypes) override {
    printf("%s\n", __PRETTY_FUNCTION__);

    // Print out the contents of the input buffer just so
    // we can see that the memory was sent properly.
    // Should be full of 0x3.
    const uint8_t* inBuffer = aInputFrame.Buffer(kGMPYPlane);
    if (!inBuffer) {
      printf("No buffer for i420 frame!\n");
      return GMPVideoGenericErr;
    }
    for (uint32_t i = 0; i < 1000; i++) {
      printf("%i", inBuffer[i]);
    }
    printf("\n");

    // Create an output frame full of 0x4.
    GMPVideoEncodedFrame* f;
    host_->CreateEncodedFrame(&f);
    f->CreateEmptyFrame(1000);
    uint8_t* outBuffer = f->Buffer();
    if (!outBuffer) {
      printf("No buffer for encoded frame!\n");
      return GMPVideoGenericErr;
    }
    memset(outBuffer, 0x4, 1000);

    callback_->Encoded(*f, aCodecSpecificInfo);
    f->Destroy();
    return GMPVideoNoErr;
             }

  virtual GMPVideoErr SetChannelParameters(uint32_t aPacketLoss, uint32_t aRTT) override {
    printf("%s\n", __PRETTY_FUNCTION__);
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr SetRates(uint32_t aNewBitRate, uint32_t aFrameRate) override {
    printf("%s\n", __PRETTY_FUNCTION__);
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr SetPeriodicKeyFrames(bool aEnable) override {
    printf("%s\n", __PRETTY_FUNCTION__);
    return GMPVideoNoErr;
  }

  virtual void EncodingComplete() override {
    printf("%s\n", __PRETTY_FUNCTION__);
    delete this;
  }

private:
  static void ThreadMain(void *ctx) {
    GMPVideoEncoder* encoder = reinterpret_cast<GMPVideoEncoder*>(ctx);

    // ctx->ThreadMain();
  }

  PRThread* thread_;
  GMPVideoHost* host_;
  ISVCEncoder* encoder_;
  uint32_t max_payload_size_;
  GMPEncoderCallback* callback_;
};

class OpenH264VideoDecoder : public GMPVideoDecoder
{
public:
  OpenH264VideoDecoder(GMPVideoHost *hostAPI) {
    printf("%s\n", __PRETTY_FUNCTION__);
    mHostAPI = hostAPI;
    mCallback = nullptr;
  }

  virtual ~OpenH264VideoDecoder() {
    printf("%s\n", __PRETTY_FUNCTION__);
  }

  virtual GMPVideoErr InitDecode(const GMPVideoCodec& codecSettings,
                                 GMPDecoderCallback* callback,
                                 int32_t coreCount) override {
    printf("%s\n", __PRETTY_FUNCTION__);
    mCallback = callback;
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Decode(GMPVideoEncodedFrame& inputFrame,
                             bool missingFrames,
                             const GMPCodecSpecificInfo& codecSpecificInfo,
                             int64_t renderTimeMs = -1) override {
    printf("%s\n", __PRETTY_FUNCTION__);

    // Print out the contents of the input buffer just so
    // we can see that the memory was sent properly.
    // Should be full of 0x2.
    const uint8_t* inBuffer = inputFrame.Buffer();
    if (!inBuffer) {
      printf("No buffer for encoded frame!\n");
      return GMPVideoGenericErr;
    }
    for (uint32_t i = 0; i < 1000; i++) {
      printf("%i", inBuffer[i]);
    }
    printf("\n");

    // Create an output frame full of 0x1.
    GMPVideoFrame* f;
    mHostAPI->CreateFrame(kGMPI420VideoFrame, &f);
    auto vf = static_cast<GMPVideoi420Frame*>(f);
    vf->CreateEmptyFrame(100, 100, 100, 100, 100);
    uint8_t* outBuffer = vf->Buffer(kGMPYPlane);
    if (!outBuffer) {
      printf("No buffer for i420 frame!\n");
      return GMPVideoGenericErr;
    }
    memset(outBuffer, 0x1, 1000);

    mCallback->Decoded(*vf);
    f->Destroy();
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Reset() override {
    printf("%s\n", __PRETTY_FUNCTION__);
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Drain() override {
    printf("%s\n", __PRETTY_FUNCTION__);
    return GMPVideoNoErr;
  }
 
  virtual void DecodingComplete() override {
    printf("%s\n", __PRETTY_FUNCTION__);
    delete this;
  }

private:
  //XXXJOSH ownership of all this stuff?
  GMPVideoHost *mHostAPI;
  GMPDecoderCallback* mCallback;
};

extern "C" {

PUBLIC_FUNC GMPErr
GMPInit(void) {
  printf("GMP: Initialized.\n");
  return GMPNoErr;
}

PUBLIC_FUNC GMPErr
GMPGetAPI(const char* aApiName, void* aHostAPI, void** aPluginApi) {
  if (!strcmp(aApiName, "decode-video")) {
    *aPluginApi = new OpenH264VideoDecoder(static_cast<GMPVideoHost*>(aHostAPI));
    return GMPNoErr;
  } else if (!strcmp(aApiName, "encode-video")) {
    *aPluginApi = new OpenH264VideoEncoder(static_cast<GMPVideoHost*>(aHostAPI));
    return GMPNoErr;
  }
  return GMPGenericErr;
}

PUBLIC_FUNC void
GMPShutdown(void) {
  printf("GMP: Shutting down.\n");
}

} // extern "C"
