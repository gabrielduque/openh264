// Written by Josh Aas and Eric Rescorla
// TODO(ekr@rtfm.com): Need license.

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gmp-general.h"
#include "gmp-video-host.h"
#include "gmp-video-encode.h"
#include "gmp-video-decode.h"
#include "gmp-video-frame-i420.h"
#include "gmp-video-frame-encoded.h"

#if defined(_MSC_VER)
#define PUBLIC_FUNC __declspec(dllexport)
#else
#define PUBLIC_FUNC
#endif

class FakeVideoEncoder : public GMPVideoEncoder
{
public:
  FakeVideoEncoder(GMPVideoHost *hostAPI) {
    printf("%s\n", __PRETTY_FUNCTION__);
    mHostAPI = hostAPI;
    mCallback = nullptr;
  }

  virtual ~FakeVideoEncoder() {
    printf("%s\n", __PRETTY_FUNCTION__);
  }

  virtual GMPVideoErr InitEncode(const GMPVideoCodec& aCodecSettings,
                                 GMPEncoderCallback* aCallback,
                                 int32_t aNumberOfCores,
                                 uint32_t aMaxPayloadSize) override {
    printf("%s\n", __PRETTY_FUNCTION__);
    mCallback = aCallback;
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
    mHostAPI->CreateEncodedFrame(&f);
    f->CreateEmptyFrame(1000);
    uint8_t* outBuffer = f->Buffer();
    if (!outBuffer) {
      printf("No buffer for encoded frame!\n");
      return GMPVideoGenericErr;
    }
    memset(outBuffer, 0x4, 1000);

    mCallback->Encoded(*f, aCodecSpecificInfo);
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
  //XXXJOSH ownership of all this stuff?
  GMPVideoHost *mHostAPI;
  GMPEncoderCallback* mCallback;
};

class FakeVideoDecoder : public GMPVideoDecoder
{
public:
  FakeVideoDecoder(GMPVideoHost *hostAPI) {
    printf("%s\n", __PRETTY_FUNCTION__);
    mHostAPI = hostAPI;
    mCallback = nullptr;
  }

  virtual ~FakeVideoDecoder() {
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
    *aPluginApi = new FakeVideoDecoder(static_cast<GMPVideoHost*>(aHostAPI));
    return GMPNoErr;
  } else if (!strcmp(aApiName, "encode-video")) {
    *aPluginApi = new FakeVideoEncoder(static_cast<GMPVideoHost*>(aHostAPI));
    return GMPNoErr;
  }
  return GMPGenericErr;
}

PUBLIC_FUNC void
GMPShutdown(void) {
  printf("GMP: Shutting down.\n");
}

} // extern "C"
