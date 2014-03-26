// Written by Josh Aas and Eric Rescorla
// TODO(ekr@rtfm.com): Need license.

#include <stdint.h>
#include <time.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <sys/time.h>

#include "gmp-platform.h"
#include "gmp-video-host.h"
#include "gmp-video-encode.h"
#include "gmp-video-decode.h"
#include "gmp-video-frame-i420.h"
#include "gmp-video-frame-encoded.h"

#include "codec_def.h"
#include "codec_app_def.h"
#include "codec_api.h"
#include "param_svc.h"

#include "task_utils.h"

#if defined(_MSC_VER)
#define PUBLIC_FUNC __declspec(dllexport)
#else
#define PUBLIC_FUNC
#endif

static int g_log_level = 3;

#define GMPLOG(l, x) do { \
        if (l <= g_log_level) { \
        const char *log_string = "unknown"; \
        if ((l >= 0) && (l <= 3)) {               \
        log_string = kLogStrings[l];            \
        } \
        std::cerr << log_string << ": " << x << std::endl; \
        } \
    } while(0)

#define GL_CRIT 0
#define GL_ERROR 1
#define GL_INFO  2
#define GL_DEBUG 3

const char *kLogStrings[] = {
  "Critical",
  "Error",
  "Info",
  "Debug"
};


static GMPPlatformAPI* g_platform_api = nullptr;

class OpenH264VideoEncoder;

template <typename T> class SelfDestruct {
 public:
  SelfDestruct(T* t) : t_(t) {}
  ~SelfDestruct() {
    if (t_) {
      t_->Destroy();
    }
  }

  T* forget() {
    T* t = t_;
    t_ = nullptr;

    return t;
  }

 private:
  T* t_;
};

class FrameStats {
 public:
  FrameStats(const char *type) :
      frames_in_(0),
      frames_out_(0),
      start_time_(time(0)),
      last_time_(start_time_),
      type_(type) {}

  void FrameIn(const int64_t timestamp) {
    ++frames_in_;
    cur_frame_timestamp_ = timestamp;
    gettimeofday(&cur_time_, NULL);
    time_t now = cur_time_.tv_sec;//time(0);
    GMPLOG(GL_DEBUG, type_ << ": Current Frame (t=" << timestamp << ") Input Time:" << now << " . " << cur_time_.tv_usec/1000);

    if (now == last_time_)
      return;

    if (!(frames_in_ % 10)) {
      GMPLOG(GL_CRIT, type_ << ": " << now << " Frame count "
          << frames_in_
          << "(" << (frames_in_ / (now - start_time_)) << "/"
          << (30 / (now - last_time_)) << ")"
          << " -- " << frames_out_);
      last_time_ = now;
    }
  }

  void FrameOut(const int frame_type) {
    ++frames_out_;
    
    {
      struct timeval end_time_;
      gettimeofday(&end_time_, NULL);
      GMPLOG(GL_DEBUG, type_ << ": Current Frame (t=" << cur_frame_timestamp_ << "f=" << frame_type << ")"
             << "Elapse (ms):"
             << ((end_time_.tv_sec - cur_time_.tv_sec)*1000 + (end_time_.tv_usec - cur_time_.tv_usec)/1000)
             );
    }
  }

 private:
  uint64_t frames_in_;
  uint64_t frames_out_;
  time_t start_time_;
  time_t last_time_;
  const std::string type_;
  int64_t cur_frame_timestamp_;
  struct timeval cur_time_;
};

class OpenH264VideoEncoder : public GMPVideoEncoder
{
 public:
  OpenH264VideoEncoder(GMPVideoHost *hostAPI) :
      host_(hostAPI),
      worker_thread_(nullptr),
      encoder_(nullptr),
      max_payload_size_(0),
      callback_(nullptr),
      stats_("Encoder") {}

  virtual ~OpenH264VideoEncoder() {
    worker_thread_->Join();
    // TODO(ekr@rtfm.com)
  }

  virtual GMPVideoErr InitEncode(const GMPVideoCodec& codecSettings,
                                 GMPEncoderCallback* callback,
                                 int32_t numberOfCores,
                                 uint32_t maxPayloadSize) override {
    GMPErr err = g_platform_api->createthread(&worker_thread_);
    if (err != GMPNoErr) {
      GMPLOG(GL_ERROR, "Couldn't create new thread");
      return GMPVideoGenericErr;
    }

    int rv = CreateSVCEncoder(&encoder_);
    if (rv) {
      return GMPVideoGenericErr;
    }

    SEncParamBase param;
    memset(&param, 0, sizeof(param));
    //encoder_->GetDefaultParams(&param);

    GMPLOG(GL_INFO, "Initializing encoder at "
            << codecSettings.mWidth
            << "x"
            << codecSettings.mHeight
            << "@"
            << static_cast<int>(codecSettings.mMaxFramerate)
            << "max payload size="
           << maxPayloadSize);

    // Translate parameters.
    param.iUsageType = CAMERA_VIDEO_REAL_TIME;
    param.iPicWidth = codecSettings.mWidth;
    param.iPicHeight = codecSettings.mHeight;
    param.iTargetBitrate =
        (codecSettings.mStartBitrate + codecSettings.mMaxBitrate) * 500;
    
    param.iRCMode = RC_BITRATE_MODE;

    // TODO(ekr@rtfm.com). Scary conversion from unsigned char to float below.
    param.fMaxFrameRate = codecSettings.mMaxFramerate;
    param.iInputCsp = videoFormatI420;

    /*
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
    */

    rv = encoder_->Initialize(&param);
    if (rv) {
      GMPLOG(GL_ERROR, "Couldn't initialize encoder");
      return GMPVideoGenericErr;
    }

    max_payload_size_ = maxPayloadSize;
    callback_ = callback;

    GMPLOG(GL_INFO, "Initialized encoder");

    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Encode(GMPVideoi420Frame* inputImage,
                             const GMPCodecSpecificInfo& codecSpecificInfo,
                             const std::vector<GMPVideoFrameType>* frameTypes) override {
    GMPLOG(GL_DEBUG,
           __FUNCTION__
           << " size="
           << inputImage->Width() << "x" << inputImage->Height());

    stats_.FrameIn(inputImage->Timestamp());

#if 0
    // TODO(josh): this is empty.

    //    assert(!frameTypes->empty());
    if (frameTypes->empty()) {
      GMPLOG(GL_ERROR, "No frame types provided");
      return GMPVideoGenericErr;
    }
#endif

    worker_thread_->Post(WrapTask(
        this, &OpenH264VideoEncoder::Encode_w,
        inputImage,
#if 0
        (*frameTypes)[0])));
#else
        kGMPKeyFrame));
#endif

    return GMPVideoGenericErr;
  }

  void Encode_w(GMPVideoi420Frame* inputImage,
                GMPVideoFrameType frame_type) {
    SFrameBSInfo encoded;

    SSourcePicture src;

    src.iColorFormat = videoFormatI420;
    src.iStride[0] = inputImage->Stride(kGMPYPlane);
    src.pData[0] = reinterpret_cast<unsigned char*>(
        const_cast<uint8_t *>(inputImage->Buffer(kGMPYPlane)));
    src.iStride[1] = inputImage->Stride(kGMPUPlane);
    src.pData[1] = reinterpret_cast<unsigned char*>(
        const_cast<uint8_t *>(inputImage->Buffer(kGMPUPlane)));
    src.iStride[2] = inputImage->Stride(kGMPVPlane);
    src.pData[2] = reinterpret_cast<unsigned char*>(
        const_cast<uint8_t *>(inputImage->Buffer(kGMPVPlane)));
    src.iStride[3] = 0;
    src.pData[3] = nullptr;
    src.iPicWidth = inputImage->Width();
    src.iPicHeight = inputImage->Height();

    const SSourcePicture* pics = &src;

    int result = encoder_->EncodeFrame(pics, &encoded);
    if (result != cmResultSuccess) {
      GMPLOG(GL_ERROR, "Couldn't encode frame. Error = " << result);
    }


    // Translate int to enum
    GMPVideoFrameType encoded_type;
    bool has_frame = false;

    switch (encoded.eOutputFrameType) {
      case videoFrameTypeIDR:
        encoded_type = kGMPKeyFrame;
        has_frame = true;
        break;
      case videoFrameTypeI:
        encoded_type = kGMPKeyFrame;
        has_frame = true;
        break;
      case videoFrameTypeP:
        encoded_type = kGMPDeltaFrame;
        has_frame = true;
        break;
      case videoFrameTypeSkip:
        //can skip the call back since not actual bit stream will be generated
        break;
      case videoFrameTypeIPMixed://this type is currently not suppported
      case videoFrameTypeInvalid:
        GMPLOG(GL_ERROR, "Couldn't encode frame. Type = "
               << encoded.eOutputFrameType);
        break;
      default:
        // The API is defined as returning a type.
        assert(false);
        break;
    }

    if (!has_frame)
      return;

    // Synchronously send this back to the main thread for delivery.
    g_platform_api->syncrunonmainthread(WrapTask(
        this,
        &OpenH264VideoEncoder::Encode_m,
        inputImage,
        &encoded,
        encoded_type));
  }

  void Encode_m(GMPVideoi420Frame* frame, SFrameBSInfo* encoded,
                GMPVideoFrameType frame_type) {
    // Now return the encoded data back to the parent.
    GMPVideoFrame* ftmp;
    GMPVideoErr err = host_->CreateFrame(kGMPEncodedVideoFrame, &ftmp);
    if (err != GMPVideoNoErr) {
      GMPLOG(GL_ERROR, "Error creating encoded frame");
      return;
    }

    GMPVideoEncodedFrame* f = static_cast<GMPVideoEncodedFrame*>(ftmp);
    // Buffer up the data.
    uint32_t length = 0;
    std::vector<uint32_t> lengths;

    for (int i=0; i<encoded->iLayerNum; ++i) {
      lengths.push_back(0);
      for (int j=0; j<encoded->sLayerInfo[i].iNalCount; ++j) {
        lengths[i] += encoded->sLayerInfo[i].iNalLengthInByte[j];
        length += encoded->sLayerInfo[i].iNalLengthInByte[j];
      }
    }

    err = f->CreateEmptyFrame(length);
    if (err != GMPVideoNoErr) {
      GMPLOG(GL_ERROR, "Error allocating frame data");
      f->Destroy();
      return;
    }

    // Copy the data.
    uint8_t* tmp = f->Buffer();
    for (int i=0; i<encoded->iLayerNum; ++i) {
      // TODO(ekr@rtfm.com): This seems screwy, but I copied it from Cisco.
      memcpy(tmp, encoded->sLayerInfo[i].pBsBuf, lengths[i]);
      tmp += lengths[i];
    }

    f->SetEncodedWidth(frame->Width());
    f->SetEncodedHeight(frame->Height());
    f->SetTimeStamp(frame->Timestamp());
    f->SetFrameType(frame_type);
    f->SetCompleteFrame(true);

    GMPLOG(GL_DEBUG, "Encoding complete. type= "
           << f->FrameType()
           << " length="
           << f->Size()
           << " timestamp="
           << f->TimeStamp());

    // Destroy the frame.
    frame->Destroy();

    // Return the encoded frame.
    GMPCodecSpecificInfo info;
    memset(&info, 0, sizeof(info));
    callback_->Encoded(f, info);

    stats_.FrameOut(frame_type);
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
  GMPVideoHost* host_;
  GMPThread* worker_thread_;
  ISVCEncoder* encoder_;
  uint32_t max_payload_size_;
  GMPEncoderCallback* callback_;
  FrameStats stats_;
};

class OpenH264VideoDecoder : public GMPVideoDecoder {
public:
  OpenH264VideoDecoder(GMPVideoHost *hostAPI) :
      host_(hostAPI),
      worker_thread_(nullptr),
      callback_(nullptr),
      decoder_(nullptr),
      stats_("Decoder") {}

  virtual ~OpenH264VideoDecoder() {
  }

  virtual GMPVideoErr InitDecode(const GMPVideoCodec& codecSettings,
                                 GMPDecoderCallback* callback,
                                 int32_t coreCount) override {
    GMPLOG(GL_INFO, "InitDecode");

    GMPErr err = g_platform_api->createthread(&worker_thread_);
    if (err != GMPNoErr) {
      GMPLOG(GL_ERROR, "Couldn't create new thread");
      return GMPVideoGenericErr;
    }

    if (CreateDecoder(&decoder_)) {
      GMPLOG(GL_ERROR, "Couldn't create decoder");
      return GMPVideoGenericErr;
    }

    if (!decoder_) {
      GMPLOG(GL_ERROR, "Couldn't create decoder");
      return GMPVideoGenericErr;
    }

    SDecodingParam param;
    memset(&param, 0, sizeof(param));
    param.iOutputColorFormat = videoFormatI420;
    param.uiTargetDqLayer = UCHAR_MAX;  // TODO(ekr@rtfm.com): correct?
    param.uiEcActiveFlag = 1; // Error concealment on.
    param.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;

    if (decoder_->Initialize(&param)) {
      GMPLOG(GL_ERROR, "Couldn't initialize decoder");
      return GMPVideoGenericErr;
    }

    callback_ = callback;
    return GMPVideoNoErr;
  }

  virtual GMPVideoErr Decode(GMPVideoEncodedFrame* inputFrame,
                             bool missingFrames,
                             const GMPCodecSpecificInfo& codecSpecificInfo,
                             int64_t renderTimeMs = -1) override {
    GMPLOG(GL_DEBUG, __FUNCTION__
           << "Decoding frame size=" << inputFrame->Size()
           << " timestamp=" << inputFrame->TimeStamp());
    stats_.FrameIn(inputFrame->TimeStamp());

    worker_thread_->Post(WrapTask(
        this, &OpenH264VideoDecoder::Decode_w,
        inputFrame,
        missingFrames,
        renderTimeMs));

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
  void Decode_w(GMPVideoEncodedFrame* inputFrame,
                bool missingFrames,
                int64_t renderTimeMs = -1) {
    GMPLOG(GL_DEBUG, "Frame decode on worker thread length = "
           << inputFrame->Size());

    SBufferInfo decoded;
    bool valid = false;
    memset(&decoded, 0, sizeof(decoded));
    void *data[3] = {nullptr, nullptr, nullptr};

    int rv = decoder_->DecodeFrame2(inputFrame->Buffer(),
                                    inputFrame->Size(),
                                    data,
                                    &decoded);

    if (rv) {
      GMPLOG(GL_ERROR, "Decoding error rv=" << rv);
    } else {
      valid = true;
    }

    g_platform_api->syncrunonmainthread(WrapTask(
        this,
        &OpenH264VideoDecoder::Decode_m,
        inputFrame,
        &decoded,
        data,
        renderTimeMs,
        valid));
  }

  // Return the decoded data back to the parent.
  void Decode_m(GMPVideoEncodedFrame* inputFrame,
                SBufferInfo* decoded,
                void* data[3],
                int64_t renderTimeMs,
                bool valid) {
    // Attach a self-destructor so that this dies on return.
    SelfDestruct<GMPVideoEncodedFrame> ifd(inputFrame);

    // If we don't actually have data, just abort.
    if (!valid)
      return;

    // TODO(ekr@rtfm.com): still need to check for BUFFER_HOST?
    if (decoded->iBufferStatus != 1)
      return;

    int width = decoded->UsrData.sSystemBuffer.iWidth;
    int height = decoded->UsrData.sSystemBuffer.iHeight;
    int ystride = decoded->UsrData.sSystemBuffer.iStride[0];
    int uvstride = decoded->UsrData.sSystemBuffer.iStride[1];

    GMPLOG(GL_DEBUG, "Video frame ready for display "
           << width
           << "x"
           << height
           << " timestamp="
           << inputFrame->TimeStamp());

    GMPVideoFrame* ftmp = nullptr;

    // Translate the image.
    GMPVideoErr err = host_->CreateFrame(kGMPI420VideoFrame, &ftmp);
    if (err != GMPVideoNoErr) {
      GMPLOG(GL_ERROR, "Couldn't allocate empty I420 frame");
      return;
    }


    GMPVideoi420Frame* frame = static_cast<GMPVideoi420Frame*>(ftmp);
    err = frame->CreateFrame(
        ystride * height, static_cast<uint8_t *>(data[0]),
        uvstride * height/2, static_cast<uint8_t *>(data[1]),
        uvstride * height/2, static_cast<uint8_t *>(data[2]),
        width, height,
        ystride, uvstride, uvstride);
    if (err != GMPVideoNoErr) {
      GMPLOG(GL_ERROR, "Couldn't make decoded frame");
      return;
    }

    GMPLOG(GL_DEBUG, "Allocated size = "
           << frame->AllocatedSize(kGMPYPlane));
    frame->SetTimestamp(inputFrame->TimeStamp());
    frame->SetRenderTime_ms(renderTimeMs);
    callback_->Decoded(frame);

    stats_.FrameOut(inputFrame->FrameType());
  }

  GMPVideoHost* host_;
  GMPThread* worker_thread_;
  GMPDecoderCallback* callback_;
  ISVCDecoder* decoder_;
  FrameStats stats_;
};

extern "C" {

PUBLIC_FUNC GMPErr
GMPInit(GMPPlatformAPI* aPlatformAPI) {
  g_platform_api = aPlatformAPI;
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
  g_platform_api = nullptr;
}

} // extern "C"
