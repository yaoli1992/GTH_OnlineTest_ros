#ifndef TOFSDK_H
#define TOFSDK_H

#include "Tof_status.h"
#include <opencv2/core.hpp>
#include "Tof_frame.h"
//#include "Tof_filters.h"

namespace GTH
{

class ITofSdk
{
public:
    /// 创建ITofSdk接口
    /// \param [in] name 名称，以'\0'结束，用于扩展。
    /// \return 非0 成功
    /// \return 0 失败
    static ITofSdk* create(const char* name = 0);

    /// 销毁对象
    virtual void destroy() = 0;

protected:
    /// 析构函数
    virtual ~ITofSdk() { }

public:

    virtual TOF_Status medianFilter(cv::Mat &depth,int ksize) = 0;

    virtual TOF_Status sobelFilter(cv::Mat &depth,int ksize,int max_val,int min_val) = 0;

    virtual TOF_Status setbilateralFilter(cv::Mat &depth,int radius,int sigmaColor,int sigmaSpace) = 0;

    virtual TOF_Status open(TOF_Config& config) = 0;

    virtual TOF_Status close() = 0;

    virtual TOF_Status getVersion(uint32_t &verMaj, uint32_t &verMin, uint16_t &supportedDeviceTypes) = 0;

    virtual TOF_Status setFrameMode(const TOF_FrameMode &frameMode) = 0;

    virtual TOF_Status getFrameMode(TOF_FrameMode &frameMode) = 0;

    virtual TOF_Status getFrame(TOF_Frame &frame) = 0;

    virtual TOF_Status freeFrame(TOF_Frame &frame) = 0;

    virtual TOF_Status getChannelData(TOF_Frame &frame, TOF_ChannelId &id, void *data) = 0;

    virtual TOF_Status getDistances(TOF_Frame &frame, void* &distBuffer) = 0;

    virtual TOF_Status getAmplitudes(TOF_Frame &frame, void* &ampBuffer) = 0;

    virtual TOF_Status getXYZcoordinates(TOF_Frame &frame, void* &xBuffer, void* &yBuffer, void* &zBuffer) = 0;

    virtual TOF_Status getPhases(TOF_Frame &frame, void* &phase1, void* &phase2, void* &phase3, void* &phase4) = 0;

    virtual TOF_Status get8Phases(TOF_Frame &frame, void* &phase1, void* &phase2, void* &phase3, void* &phase4,
                                  void* &phase5, void* &phase6, void* &phase7, void* &phase8) = 0;

    virtual TOF_Status setIntegrationTime(const uint32_t &integrationTime) = 0;

    virtual TOF_Status getIntegrationTime(uint32_t &integrationTime) = 0;

    virtual TOF_Status setFrameRate(const float &frameRate) = 0;

    virtual TOF_Status getFrameRate(float &frameRate) = 0;

    virtual TOF_Status setModulationFrequency(const uint32_t &modulationFrequency) = 0;

    virtual TOF_Status getModulationFrequency(uint32_t &modulationFrequency) = 0;

    virtual TOF_Status setDistanceOffset(const int &globalOffset) = 0;

    virtual TOF_Status getDistanceOffset(int &globalOffset) = 0;

    virtual TOF_Status setGlobalOffset(int32_t globalOffset) = 0;

    virtual TOF_Status setAmplitudeThreshold(const int &amplitudeThreshold) = 0;

    virtual TOF_Status getAmplitudeThreshold(int &amplitudeThreshold) = 0;

    virtual TOF_Status setBinningStatus(const bool &binning) = 0;

    virtual TOF_Status getBinningStatus(bool &binning) = 0;

    virtual TOF_Status setTofParameters(const TOF_Parameters &para) = 0;

    virtual TOF_Status getTofParameters(TOF_Parameters &para) = 0;

    virtual TOF_Status readRegister(uint16_t *address, uint16_t *data, uint32_t registerCount) = 0;

    virtual TOF_Status writeRegister(uint16_t *address, uint16_t *data, uint32_t registerCount) = 0;

    virtual TOF_Status getCalibrationFile(const char* filename) = 0;

    virtual TOF_Status setCalibrationFile(const char* filename) = 0;

    virtual TOF_Status sendReset() = 0;

    virtual TOF_Status sendClearErrors() = 0;

    virtual TOF_Status sendTrigger() = 0;

    virtual TOF_Status getInternalParameters(TOF_InternalParameters& intPara) = 0;

    virtual TOF_Status calculate(int16_t* pixels1, int16_t* pixels2,
                                 int16_t* pixels3, int16_t* pixels4,
                                 int16_t* depth_out, int16_t* confidence_out) = 0;

    virtual TOF_Status calculate8Phases(int16_t* pixels1, int16_t* pixels2,
                                        int16_t* pixels3, int16_t* pixels4,
                                        int16_t* pixels5, int16_t* pixels6,
                                        int16_t* pixels7, int16_t* pixels8,
                                        int16_t* depth_out, int16_t* confidence_out) = 0;

    virtual TOF_Status calculateLongShortExposure(int16_t* pixels1_f, int16_t* pixels2_f,
                                                  int16_t* pixels3_f, int16_t* pixels4_f,
                                                  int16_t* pixels1_b, int16_t* pixels2_b,
                                                  int16_t* pixels3_b, int16_t* pixels4_b,
                                                  float* depth_out, float* confidence_out) = 0;

    virtual TOF_Status calculateLongShortExposure_L(int16_t* pixels1, int16_t* pixels2,
                                          int16_t* pixels3, int16_t* pixels4,
                                          float* depth_out, float* confidence_out) = 0;

    virtual TOF_Status calculateLongShortExposure_S(int16_t* pixels1, int16_t* pixels2,
                                          int16_t* pixels3, int16_t* pixels4,
                                          float* depth_out, float* confidence_out) = 0;

    virtual TOF_Status calculateLongShortExposure_four_mode(int16_t* pixels1, int16_t* pixels2,
                                          int16_t* pixels3, int16_t* pixels4,
                                          float* depth_out, float* confidence_out) = 0;

    virtual TOF_Status isFilter(bool filter) = 0;

    virtual  TOF_Status setLongShortExposurePara(const uint32_t &fmod_l,
                                                  const uint32_t &integrationTime_l,
                                                  const uint32_t &gain_l,
                                                  const uint32_t &fmod_s,
                                                  const uint32_t &integrationTime_s,
                                                  const uint32_t &gain_s,
                                                  const float &fps) = 0;

};

#endif // TOFSDK_H
}
