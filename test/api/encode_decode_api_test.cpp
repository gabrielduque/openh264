#include <gtest/gtest.h>
#include "codec_def.h"
#include "utils/BufferedData.h"
#include "utils/FileInputStream.h"
#include "BaseDecoderTest.h"
#include "BaseEncoderTest.h"
#include "wels_common_defs.h"
#include <string>
#include <vector>
using namespace WelsCommon;

typedef struct SLost_Sim {
  WelsCommon::EWelsNalUnitType eNalType;
  bool isLost;
} SLostSim;


struct EncodeDecodeFileParamBase {
  int numframes;
  int width;
  int height;
  float frameRate;
  int slicenum;
  bool bLostPara;
  const char* pLossSequence;
};

class EncodeDecodeTestBase : public ::testing::TestWithParam<EncodeDecodeFileParamBase>,
  public BaseEncoderTest, public BaseDecoderTest {
 public:
  virtual void SetUp() {
    BaseEncoderTest::SetUp();
    BaseDecoderTest::SetUp();
  }

  virtual void TearDown() {
    BaseEncoderTest::TearDown();
    BaseDecoderTest::TearDown();
  }

  virtual void prepareParam (int width, int height, float framerate) {
    memset (&param_, 0, sizeof (SEncParamExt));
    param_.iUsageType = CAMERA_VIDEO_REAL_TIME;
    param_.iPicWidth = width;
    param_.iPicHeight = height;
    param_.fMaxFrameRate = framerate;
    param_.iRCMode = RC_OFF_MODE; //rc off
    param_.iMultipleThreadIdc = 1; //single thread
    param_.sSpatialLayers[0].iVideoWidth = width;
    param_.sSpatialLayers[0].iVideoHeight = height;
    param_.sSpatialLayers[0].fFrameRate = framerate;
    param_.sSpatialLayers[0].sSliceCfg.uiSliceMode = SM_SINGLE_SLICE;
  }

  virtual void encToDecData (const SFrameBSInfo& info, int& len) {
    len = 0;
    for (int i = 0; i < info.iLayerNum; ++i) {
      const SLayerBSInfo& layerInfo = info.sLayerInfo[i];
      for (int j = 0; j < layerInfo.iNalCount; ++j) {
        len += layerInfo.pNalLengthInByte[j];
      }
    }
  }

 protected:
  SEncParamExt param_;
  BufferedData buf_;
  SBufferInfo dstBufInfo_;
  std::vector<SLostSim> m_SLostSim;
};

class EncodeDecodeTestAPI : public EncodeDecodeTestBase {
 public:
  void SetUp() {
    EncodeDecodeTestBase::SetUp();
  }

  void TearDown() {
    EncodeDecodeTestBase::TearDown();
  }

  void prepareParam (int width, int height, float framerate) {
    EncodeDecodeTestBase::prepareParam (width, height, framerate);
  }
};

static const EncodeDecodeFileParamBase kFileParamArray[] = {
  {300, 160, 96, 6.0f, 2, 1, "000000000000001010101010101010101010101001101010100000010101000011"},
  {300, 140, 96, 6.0f, 4, 1, "000000000000001010101010101010101010101001101010100000010101000011"},
};

TEST_P (EncodeDecodeTestAPI, DecoderVclNal) {
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);

  //init for encoder
  // I420: 1(Y) + 1/4(U) + 1/4(V)
  int frameSize = p.width * p.height * 3 / 2;

  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int iIdx = 0;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    ASSERT_TRUE (rv == cmResultSuccess);
    //decoding after each encoding frame
    int vclNal, len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    ASSERT_TRUE (rv == cmResultSuccess);
    rv = decoder_->GetOption (DECODER_OPTION_VCL_NAL, &vclNal);
    EXPECT_EQ (vclNal, FEEDBACK_UNKNOWN_NAL); //no reconstruction, unknown return
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    ASSERT_TRUE (rv == cmResultSuccess);
    rv = decoder_->GetOption (DECODER_OPTION_VCL_NAL, &vclNal);
    EXPECT_EQ (vclNal, FEEDBACK_VCL_NAL);
    iIdx++;
  } //while
  //ignore last frame
}

TEST_P (EncodeDecodeTestAPI, GetOptionFramenum) {
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);

  //init for encoder
  // I420: 1(Y) + 1/4(U) + 1/4(V)
  int frameSize = p.width * p.height * 3 / 2;
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);

  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iEncFrameNum = -1;
  int32_t iDecFrameNum;
  int iIdx = 0;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    ASSERT_TRUE (rv == cmResultSuccess);
    //decoding after each encoding frame
    int len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    ASSERT_TRUE (rv == cmResultSuccess);
    decoder_->GetOption (DECODER_OPTION_FRAME_NUM, &iDecFrameNum);
    EXPECT_EQ (iDecFrameNum, -1);
    iEncFrameNum++;
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    ASSERT_TRUE (rv == cmResultSuccess);
    decoder_->GetOption (DECODER_OPTION_FRAME_NUM, &iDecFrameNum);
    EXPECT_EQ (iEncFrameNum, iDecFrameNum);
    iIdx++;
  } //while
  //ignore last frame
}

TEST_P (EncodeDecodeTestAPI, GetOptionIDR) {
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);

  //init for encoder
  // I420: 1(Y) + 1/4(U) + 1/4(V)
  int frameSize = p.width * p.height * 3 / 2;
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);

  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iEncCurIdrPicId = 0;
  int32_t iDecCurIdrPicId;
  int32_t iIDRPeriod = 1;
  int32_t iSpsPpsIdAddition = 0;
  int iIdx = 0;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    iSpsPpsIdAddition = rand() % 2;
    iIDRPeriod = (rand() % 150) + 1;
    encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
    encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
    rv = encoder_->EncodeFrame (&pic, &info);
    ASSERT_TRUE (rv == cmResultSuccess);
    if (info.eFrameType == videoFrameTypeIDR) {
      iEncCurIdrPicId = (iSpsPpsIdAddition == 0) ? 0 : (iEncCurIdrPicId + 1);
    }
    //decoding after each encoding frame
    int len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    ASSERT_TRUE (rv == cmResultSuccess);
    decoder_->GetOption (DECODER_OPTION_IDR_PIC_ID, &iDecCurIdrPicId);
    EXPECT_EQ (iDecCurIdrPicId, iEncCurIdrPicId);
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    ASSERT_TRUE (rv == cmResultSuccess);
    decoder_->GetOption (DECODER_OPTION_IDR_PIC_ID, &iDecCurIdrPicId);
    EXPECT_EQ (iDecCurIdrPicId, iEncCurIdrPicId);
    iIdx++;
  } //while
  //ignore last frame
}


long IsKeyFrameLost (ISVCDecoder* pDecoder, SLTRRecoverRequest* p_LTR_Recover_Request, long hr) {
  long bLost = NO_RECOVERY_REQUSET;
  int tempInt = -1;
  int temple_id = -1;
  bool m_P2PmodeFlag = true;
  pDecoder->GetOption (DECODER_OPTION_TEMPORAL_ID, &temple_id);
  if (hr == dsErrorFree) {
    if (m_P2PmodeFlag && temple_id == 0) {
      pDecoder->GetOption (DECODER_OPTION_IDR_PIC_ID, &tempInt);
      // idr_pic_id change ,reset last correct position
      if (p_LTR_Recover_Request->uiIDRPicId != tempInt) {
        p_LTR_Recover_Request->uiIDRPicId = tempInt;
        p_LTR_Recover_Request->iLastCorrectFrameNum = -1;
      }
      pDecoder->GetOption (DECODER_OPTION_FRAME_NUM, &tempInt);
      if (tempInt >= 0) {
        p_LTR_Recover_Request->iLastCorrectFrameNum = tempInt;
      }
    }
    bLost = NO_RECOVERY_REQUSET;
  } else if (hr & dsNoParamSets) {
    bLost = IDR_RECOVERY_REQUEST;
  } else if (((hr & dsRefLost) && (1 == temple_id)) || ((dsErrorFree != hr) && (0 == temple_id)))	{
    bLost = LTR_RECOVERY_REQUEST;
  } else {
    bLost = NO_RECOVERY_REQUSET;
  }
  return bLost;
}

bool IsLTRMarking (ISVCDecoder* pDecoder) {
  int bLTR_marking_flag = 0;
  pDecoder->GetOption (DECODER_OPTION_LTR_MARKING_FLAG, &bLTR_marking_flag);
  return (bLTR_marking_flag) ? (true) : (false);
}

void LTRRecoveryRequest (ISVCDecoder* pDecoder, ISVCEncoder* pEncoder, SLTRRecoverRequest* p_LTR_Recover_Request,
                         long hr) {

  long bKLost = IsKeyFrameLost (pDecoder, p_LTR_Recover_Request, hr);
  bool m_P2PmodeFlag = true;
  if (m_P2PmodeFlag) {
    if (bKLost == IDR_RECOVERY_REQUEST) {
      pEncoder->ForceIntraFrame (true);
    } else if (bKLost == LTR_RECOVERY_REQUEST)	{
      p_LTR_Recover_Request->uiFeedbackType = LTR_RECOVERY_REQUEST;
      pDecoder->GetOption (DECODER_OPTION_FRAME_NUM, &p_LTR_Recover_Request->iCurrentFrameNum);
      pDecoder->GetOption (DECODER_OPTION_IDR_PIC_ID, &p_LTR_Recover_Request->uiIDRPicId);
      pEncoder->SetOption (ENCODER_LTR_RECOVERY_REQUEST, p_LTR_Recover_Request);
    }
  } else {
    if (bKLost == IDR_RECOVERY_REQUEST || bKLost == LTR_RECOVERY_REQUEST)	{
      p_LTR_Recover_Request->uiFeedbackType = IDR_RECOVERY_REQUEST;
      pDecoder->GetOption (DECODER_OPTION_FRAME_NUM, &p_LTR_Recover_Request->iCurrentFrameNum);
      pDecoder->GetOption (DECODER_OPTION_IDR_PIC_ID, &p_LTR_Recover_Request->uiIDRPicId);
      pEncoder->SetOption (ENCODER_LTR_RECOVERY_REQUEST, p_LTR_Recover_Request);
    }
  }
}

void LTRMarkFeedback (ISVCDecoder* pDecoder, ISVCEncoder* pEncoder, SLTRMarkingFeedback* p_LTR_Marking_Feedback,
                      long hr) {
  if (IsLTRMarking (pDecoder) == true) {
    p_LTR_Marking_Feedback->uiFeedbackType = (hr == dsErrorFree) ? (LTR_MARKING_SUCCESS) : (LTR_MARKING_FAILED);
    pDecoder->GetOption (DECODER_OPTION_IDR_PIC_ID, &p_LTR_Marking_Feedback->uiIDRPicId);
    pDecoder->GetOption (DECODER_OPTION_LTR_MARKED_FRAME_NUM, &p_LTR_Marking_Feedback->iLTRFrameNum);
    pEncoder->SetOption (ENCODER_LTR_MARKING_FEEDBACK, p_LTR_Marking_Feedback);
  }
}


int SimulateNALLoss (const unsigned char* pSrc,  int& iSrcLen, std::vector<SLostSim>* p_SLostSim,
                     const char* pLossChars, bool bLossPara, int& iLossIdx, bool& bVCLLoss) {
  unsigned char* pDst = new unsigned char[iSrcLen];
  int iLossCharLen = strlen (pLossChars);
  int iSkipedBytes = 0;
  int iDstLen = 0;
  int iBufPos = 0;
  int ilastprefixlen = 0;
  int i = 0;
  bool bLost;
  bVCLLoss = false;
  SLostSim tmpSLostSim;
  p_SLostSim->clear();
  for (i = 0; i < iSrcLen;) {
    if (pSrc[i] == 0 && pSrc[i + 1] == 0 && pSrc[i + 2] == 0 && pSrc[i + 3] == 1) {
      if (i - iBufPos) {
        tmpSLostSim.eNalType = (EWelsNalUnitType) ((* (pSrc + iBufPos + ilastprefixlen)) & 0x1f);	// eNalUnitType
        bLost = iLossIdx < iLossCharLen ? (pLossChars[iLossIdx] == '1') : (rand() % 2 == 1);
        bLost = (!bLossPara) && (IS_PARAM_SETS_NALS (tmpSLostSim.eNalType)) ? false : bLost;
        iLossIdx++;
        tmpSLostSim.isLost = bLost;
        p_SLostSim->push_back (tmpSLostSim);
        if (!bLost) {
          memcpy (pDst + iDstLen, pSrc + iBufPos, i - iBufPos);
          iDstLen += (i - iBufPos);
        } else {
          bVCLLoss = (IS_VCL_NAL (tmpSLostSim.eNalType, 1)) ? true : bVCLLoss;
          iSkipedBytes += (i - iBufPos);
        }
      }
      ilastprefixlen = 4;
      iBufPos = i;
      i = i + 4;
    } else if (pSrc[i] == 0 && pSrc[i + 1] == 0 && pSrc[i + 2] == 1) {
      if (i - iBufPos) {
        tmpSLostSim.eNalType = (EWelsNalUnitType) ((* (pSrc + iBufPos + ilastprefixlen)) & 0x1f);	// eNalUnitType
        bLost = iLossIdx < iLossCharLen ? (pLossChars[iLossIdx] == '1') : (rand() % 2 == 1);
        bLost = (!bLossPara) && (IS_PARAM_SETS_NALS (tmpSLostSim.eNalType)) ? false : bLost;
        iLossIdx++;
        tmpSLostSim.isLost = bLost;
        p_SLostSim->push_back (tmpSLostSim);
        if (!bLost) {
          memcpy (pDst + iDstLen, pSrc + iBufPos, i - iBufPos);
          iDstLen += (i - iBufPos);
        } else {
          bVCLLoss = (IS_VCL_NAL (tmpSLostSim.eNalType, 1)) ? true : bVCLLoss;
          iSkipedBytes += (i - iBufPos);
        }
      }
      ilastprefixlen = 3;
      iBufPos = i;
      i = i + 3;
    } else {
      i++;
    }
  }
  if (i - iBufPos) {
    tmpSLostSim.eNalType = (EWelsNalUnitType) ((* (pSrc + iBufPos + ilastprefixlen)) & 0x1f);	// eNalUnitType
    bLost = iLossIdx < iLossCharLen ? (pLossChars[iLossIdx] == '1') : (rand() % 2 == 1);
    bLost = (!bLossPara) && (IS_PARAM_SETS_NALS (tmpSLostSim.eNalType)) ? false : bLost;
    iLossIdx++;
    tmpSLostSim.isLost = bLost;
    p_SLostSim->push_back (tmpSLostSim);
    if (!bLost) {
      memcpy (pDst + iDstLen, pSrc + iBufPos, i - iBufPos);
      iDstLen += (i - iBufPos);
    } else {
      bVCLLoss = (IS_VCL_NAL (tmpSLostSim.eNalType, 1)) ? true : bVCLLoss;
      iSkipedBytes += (i - iBufPos);
    }
  }
  memset ((void*)pSrc, 0, iSrcLen);
  memcpy ((void*)pSrc, pDst, iDstLen);
  iSrcLen = iDstLen;
  delete pDst;
  return iSkipedBytes;
}

TEST_P (EncodeDecodeTestAPI, GetOptionLTR_ALLIDR) {
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);
  int frameSize = p.width * p.height * 3 / 2;
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);

  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));
  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  int32_t iSpsPpsIdAddition = 1;
  encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
  int32_t iIDRPeriod = 60;
  encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
  SLTRConfig sLtrConfigVal;
  sLtrConfigVal.bEnableLongTermReference = 1;
  sLtrConfigVal.iLTRRefNum = 1;
  encoder_->SetOption (ENCODER_OPTION_LTR, &sLtrConfigVal);
  int32_t iLtrPeriod = 2;
  encoder_->SetOption (ENCODER_LTR_MARKING_PERIOD, &iLtrPeriod);
  int iIdx = 0;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    ASSERT_EQ (rv, cmResultSuccess);
    ASSERT_TRUE (info.eFrameType == videoFrameTypeIDR);
    encoder_->ForceIntraFrame (true);
    iIdx++;
  }
}

TEST_P (EncodeDecodeTestAPI, GetOptionLTR_ALLLTR) {
  SLTRMarkingFeedback m_LTR_Marking_Feedback;
  SLTRRecoverRequest m_LTR_Recover_Request;
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);
  m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
  int frameSize = p.width * p.height * 3 / 2;
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);
  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  int32_t iSpsPpsIdAddition = 1;
  encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
  int32_t iIDRPeriod = 60;
  encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
  SLTRConfig sLtrConfigVal;
  sLtrConfigVal.bEnableLongTermReference = 1;
  sLtrConfigVal.iLTRRefNum = 1;
  encoder_->SetOption (ENCODER_OPTION_LTR, &sLtrConfigVal);
  int32_t iLtrPeriod = 2;
  encoder_->SetOption (ENCODER_LTR_MARKING_PERIOD, &iLtrPeriod);
  int iIdx = 0;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    if (m_LTR_Recover_Request.uiFeedbackType == IDR_RECOVERY_REQUEST) {
      ASSERT_TRUE (info.eFrameType == videoFrameTypeIDR);
    }
    ASSERT_TRUE (rv == cmResultSuccess || rv == cmUnkonwReason);
    m_LTR_Recover_Request.uiFeedbackType = LTR_RECOVERY_REQUEST;
    m_LTR_Recover_Request.iCurrentFrameNum = rand() % 2 == 1 ? -rand() % 10000 : rand() % 10000;
    m_LTR_Recover_Request.uiIDRPicId = rand() % 2 == 1 ? -rand() % 10000 : rand() % 10000;
    encoder_->SetOption (ENCODER_LTR_RECOVERY_REQUEST, &m_LTR_Recover_Request);
    m_LTR_Marking_Feedback.uiFeedbackType = rand() % 2 == 1 ? LTR_MARKING_SUCCESS : LTR_MARKING_FAILED;
    m_LTR_Marking_Feedback.uiIDRPicId = rand() % 2 == 1 ? -rand() % 10000 : rand() % 10000;
    m_LTR_Marking_Feedback.iLTRFrameNum = rand() % 2 == 1 ? -rand() % 10000 : rand() % 10000;
    encoder_->SetOption (ENCODER_LTR_MARKING_FEEDBACK, &m_LTR_Marking_Feedback);
    iIdx++;
  }
}

TEST_P (EncodeDecodeTestAPI, GetOptionLTR_Engine) {
  SLTRMarkingFeedback m_LTR_Marking_Feedback;
  SLTRRecoverRequest m_LTR_Recover_Request;
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  param_.sSpatialLayers[0].sSliceCfg.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
  param_.sSpatialLayers[0].sSliceCfg.sSliceArgument.uiSliceNum = p.slicenum;
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);
  m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
  int frameSize = p.width * p.height * 3 / 2;
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);
  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  int32_t iSpsPpsIdAddition = 1;
  encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
  int32_t iIDRPeriod = 60;
  encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
  SLTRConfig sLtrConfigVal;
  sLtrConfigVal.bEnableLongTermReference = 1;
  sLtrConfigVal.iLTRRefNum = 1;
  encoder_->SetOption (ENCODER_OPTION_LTR, &sLtrConfigVal);
  int32_t iLtrPeriod = 2;
  encoder_->SetOption (ENCODER_LTR_MARKING_PERIOD, &iLtrPeriod);
  int iIdx = 0;
  int iLossIdx = 0;
  bool bVCLLoss = false;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    if (m_LTR_Recover_Request.uiFeedbackType == IDR_RECOVERY_REQUEST) {
      ASSERT_TRUE (info.eFrameType == videoFrameTypeIDR);
    }
    ASSERT_TRUE (rv == cmResultSuccess || rv == cmUnkonwReason);
    //decoding after each encoding frame
    int len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    SimulateNALLoss (info.sLayerInfo[0].pBsBuf, len, &m_SLostSim, p.pLossSequence, p.bLostPara, iLossIdx, bVCLLoss);
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    LTRMarkFeedback (decoder_, encoder_, &m_LTR_Marking_Feedback, rv);
    iIdx++;
  }
}

TEST_P (EncodeDecodeTestAPI, SetOptionECFlag_ERROR_CON_DISABLE) {
  SLTRMarkingFeedback m_LTR_Marking_Feedback;
  SLTRRecoverRequest m_LTR_Recover_Request;
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  param_.sSpatialLayers[0].sSliceCfg.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
  param_.sSpatialLayers[0].sSliceCfg.sSliceArgument.uiSliceNum = p.slicenum;
  param_.bEnableLongTermReference = true;
  param_.iLTRRefNum = 1;
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);
  if (decoder_ != NULL) {
    decoder_->Uninitialize();
  }
  SDecodingParam decParam;
  memset (&decParam, 0, sizeof (SDecodingParam));
  decParam.eOutputColorFormat  = videoFormatI420;
  decParam.uiTargetDqLayer = UCHAR_MAX;
  decParam.eEcActiveIdc = ERROR_CON_DISABLE;
  decParam.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;
  rv = decoder_->Initialize (&decParam);
  ASSERT_EQ (0, rv);
  m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
  int frameSize = p.width * p.height * 3 / 2;
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);
  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  int32_t iSpsPpsIdAddition = 1;
  encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
  int32_t iIDRPeriod = 60;
  encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
  SLTRConfig sLtrConfigVal;
  sLtrConfigVal.bEnableLongTermReference = 1;
  sLtrConfigVal.iLTRRefNum = 1;
  encoder_->SetOption (ENCODER_OPTION_LTR, &sLtrConfigVal);
  int32_t iLtrPeriod = 2;
  encoder_->SetOption (ENCODER_LTR_MARKING_PERIOD, &iLtrPeriod);
  int iIdx = 0;
  int iLossIdx = 0;
  int iSkipedBytes;
  bool bVCLLoss = false;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    if (m_LTR_Recover_Request.uiFeedbackType == IDR_RECOVERY_REQUEST) {
      ASSERT_TRUE (info.eFrameType == videoFrameTypeIDR);
    }
    ASSERT_TRUE (rv == cmResultSuccess || rv == cmUnkonwReason);
    //decoding after each encoding frame
    int len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    iSkipedBytes = SimulateNALLoss (info.sLayerInfo[0].pBsBuf, len, &m_SLostSim, p.pLossSequence, p.bLostPara, iLossIdx,
                                    bVCLLoss);
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    LTRMarkFeedback (decoder_, encoder_, &m_LTR_Marking_Feedback, rv);
    if (iSkipedBytes && bVCLLoss) {
      ASSERT_TRUE (dstBufInfo_.iBufferStatus == 0);
    }
    iIdx++;
  }
}

TEST_P (EncodeDecodeTestAPI, SetOptionECFlag_ERROR_CON_SLICE_COPY) {
  SLTRMarkingFeedback m_LTR_Marking_Feedback;
  SLTRRecoverRequest m_LTR_Recover_Request;
  EncodeDecodeFileParamBase p = GetParam();
  prepareParam (p.width, p.height, p.frameRate);
  param_.sSpatialLayers[0].sSliceCfg.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
  param_.sSpatialLayers[0].sSliceCfg.sSliceArgument.uiSliceNum = p.slicenum;
  encoder_->Uninitialize();
  int rv = encoder_->InitializeExt (&param_);
  ASSERT_TRUE (rv == cmResultSuccess);
  m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
  int frameSize = p.width * p.height * 3 / 2;
  buf_.SetLength (frameSize);
  ASSERT_TRUE (buf_.Length() == (size_t)frameSize);
  SFrameBSInfo info;
  memset (&info, 0, sizeof (SFrameBSInfo));

  SSourcePicture pic;
  memset (&pic, 0, sizeof (SSourcePicture));
  pic.iPicWidth = p.width;
  pic.iPicHeight = p.height;
  pic.iColorFormat = videoFormatI420;
  pic.iStride[0] = pic.iPicWidth;
  pic.iStride[1] = pic.iStride[2] = pic.iPicWidth >> 1;
  pic.pData[0] = buf_.data();
  pic.pData[1] = pic.pData[0] + p.width * p.height;
  pic.pData[2] = pic.pData[1] + (p.width * p.height >> 2);
  int32_t iTraceLevel = WELS_LOG_QUIET;
  encoder_->SetOption (ENCODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  decoder_->SetOption (DECODER_OPTION_TRACE_LEVEL, &iTraceLevel);
  int32_t iSpsPpsIdAddition = 1;
  encoder_->SetOption (ENCODER_OPTION_ENABLE_SPS_PPS_ID_ADDITION, &iSpsPpsIdAddition);
  int32_t iIDRPeriod = 60;
  encoder_->SetOption (ENCODER_OPTION_IDR_INTERVAL, &iIDRPeriod);
  SLTRConfig sLtrConfigVal;
  sLtrConfigVal.bEnableLongTermReference = 1;
  sLtrConfigVal.iLTRRefNum = 1;
  encoder_->SetOption (ENCODER_OPTION_LTR, &sLtrConfigVal);
  int32_t iLtrPeriod = 2;
  encoder_->SetOption (ENCODER_LTR_MARKING_PERIOD, &iLtrPeriod);
  int iIdx = 0;
  int iLossIdx = 0;
  int iSkipedBytes;
  bool bVCLLoss = false;
  while (iIdx <= p.numframes) {
    memset (buf_.data(), rand() % 256, frameSize);
    rv = encoder_->EncodeFrame (&pic, &info);
    if (m_LTR_Recover_Request.uiFeedbackType == IDR_RECOVERY_REQUEST) {
      ASSERT_TRUE (info.eFrameType == videoFrameTypeIDR);
    }
    ASSERT_TRUE (rv == cmResultSuccess || rv == cmUnkonwReason);
    //decoding after each encoding frame
    int len = 0;
    encToDecData (info, len);
    unsigned char* pData[3] = { NULL };
    memset (&dstBufInfo_, 0, sizeof (SBufferInfo));
    iSkipedBytes = SimulateNALLoss (info.sLayerInfo[0].pBsBuf, len, &m_SLostSim, p.pLossSequence, p.bLostPara, iLossIdx,
                                    bVCLLoss);
    uint32_t uiEcIdc = rand() % 2 == 1 ? ERROR_CON_DISABLE : ERROR_CON_SLICE_COPY;
    decoder_->SetOption (DECODER_OPTION_ERROR_CON_IDC, &uiEcIdc);
    rv = decoder_->DecodeFrame2 (info.sLayerInfo[0].pBsBuf, len, pData, &dstBufInfo_);
    m_LTR_Recover_Request.uiFeedbackType = NO_RECOVERY_REQUSET;
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    rv = decoder_->DecodeFrame2 (NULL, 0, pData, &dstBufInfo_); //reconstruction
    LTRRecoveryRequest (decoder_, encoder_, &m_LTR_Recover_Request, rv);
    LTRMarkFeedback (decoder_, encoder_, &m_LTR_Marking_Feedback, rv);
    iIdx++;
  }
}

INSTANTIATE_TEST_CASE_P (EncodeDecodeTestBase, EncodeDecodeTestAPI,
                         ::testing::ValuesIn (kFileParamArray));
