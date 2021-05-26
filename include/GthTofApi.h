/*****************************************************************//**
 *       @file  GthTofApi.h
 *      @brief  Gth's camera device API
 *
 *  Detail Decsription starts here
 *
 *   @internal
 *     Project  $Project$
 *     Created  7/13/2020
 *    Revision  $Id$
 *     Company  Gth, Hefei
 *   Copyright  (C) 2020 Gth
 *    
 *    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *    PARTICULAR PURPOSE.
 *
 * *******************************************************************/

#ifndef GTH_TOF_API_H
#define GTH_TOF_API_H
#ifdef __cplusplus
extern "C"
{
#endif
#ifdef _MSC_VER
#define __API __declspec(dllexport)

#if _MSC_VER >= 1800
#include <stdbool.h>
#else
#define false   0
#define true    1

#define bool uint8_t
#endif

#else
#define __API
#include <stdbool.h>
#endif
#include <stdint.h>

#include "GthTofTypes.h"

#define GTH_TOF_NAME "GTH_TOF"
#define GTH_TOF_VERSION_MAJOR 1
#define GTH_TOF_VERSION_MINOR 1
#define GTH_TOF_VERSION_REV   0
#define GTH_TOF_VERSION_STR "v1.1.0"

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5)
#define API_DEPRECATED_FOR(f) \
          __attribute__((deprecated("Use " #f " instead")))
#define API_DEPRECATED \
          __attribute__((deprecated("This API is deprecated and will be remove in the furture release")))
#else
#define API_DEPRECATED_FOR(f)
#define API_DEPRECATED
#endif /* __GNUC__ */

/**
 * Set the logging configuration for gth tof camera.
 *
 * @param log_level [in] specified Gth_Log_Level, the
 *                      console log whose log level bellow this
 *                      value will be suppressed.
 * @param path [in] specified log path name of gth tof camera.
 *                      fthe default log path is "/log/".
 * @return int [out] return 0 for success.
 */
__API int Gth_LogConfig(Gth_Log_Level log_level, const char* path = "./log/");

/**
* Open gth tof camera.
*
* @param dev [in] specified information of gth tof camera.
*                 If the callback function(Gth_FrameArrived) is not null,
*                 only use the callback function to get image data.
* @param handle [out] specified camera handle.
* @return int [out] return 0 for success.
*/
__API int Gth_OpenDevice(const Gth_Dev_Info *dev, Gth_Dev_Handle *handle);

/**
* Close gth tof camera.
*
* @param handle [in] specified camera handle.
* @return int [out] return 0 for success.
*/
__API int Gth_CloseDevice(const Gth_Dev_Handle *handle);

/**
* Start capturing the image stream.
*
* @param handle [in] specified camera handle.
* @return int [out] return 0 for success.
*/
__API int Gth_StartStream(const Gth_Dev_Handle *handle);

/**
* Stop capturing the image stream.
*
* @param handle [in] specified camera handle.
* @return int [out] return 0 for success.
*/
__API int Gth_StopStream(const Gth_Dev_Handle *handle);

/**
* Capture a image frame from the camera
*
* @param handle [in] specified camera handle.
* @param frame [out] pointer to a buffer in which to store the returned image data,
*                    only used when the callback function is empty.
* @return int [out] return 0 for success.
*/
__API int Gth_GetFrame(const Gth_Dev_Handle *handle, Gth_Frame *frame);

/**
* Free a image frame from the camera
*
* @param handle [in] specified camera handle.
* @param frame [in] pointer to a buffer in which to store image data,
*                   only used when get a frame success.
* @return int [out] return 0 for success.
*/
__API int Gth_FreeFrame(const Gth_Dev_Handle *handle, Gth_Frame *frame);

/**
* Get depth data and amplitude data from the a frame.
*
* @param handle [in] specified camera handle.
* @param frame [in] pointer to a buffer in which to store image data.
* @param depth [in] pointer to a buffer in which to store depth data in int_16(unit: millimeter),
*                   has to be pre-allocated.
* @param amplitude [in] pointer to a buffer in which to store amplitude data in int_16,
*                       has to be pre-allocated.
* @return int [out] return 0 for success.
*/
__API int Gth_GetDepthI16andAmplitudeData(const Gth_Dev_Handle *handle, const Gth_Frame* frame, int16_t* depth, int16_t* amplitude);

/**
* Get depth data and amplitude data from the a frame.
*
* @param handle [in] specified camera handle.
* @param frame [in] pointer to a buffer in which to store image data.
* @param depth [in] pointer to a buffer in which to store depth data in float(unit: meter),
*                   has to be pre-allocated.
* @param amplitude [in] pointer to a buffer in which to store amplitude data in int_16,
*                       has to be pre-allocated.
* @return int [out] return 0 for success.
*/
__API int Gth_GetDepthF32andAmplitudeData(const Gth_Dev_Handle *handle, const Gth_Frame* frame, float* depth, int16_t* amplitude);

/**
* Get point cloud data from distance data. The distance data is
* usually calcuated using Gth_GetDepthI16andAmplitudeData.
*
* @param handle [in] specified camera handle.
* @param depth [in] pointer to a buffer in which to store depth data in int_16(unit: millimeter),
* @param pcl [out] point clound buffer. each 3 element consists a (x,y,z) point.
*                  (x,y,z) is coordinate in meter unit. point in value (0,0,0) is invalid.
* @param pointCount [out] point cloud float element count.
* @return int [out] return 0 for success.
*/
__API int Gth_GetXYZDataF32(const Gth_Dev_Handle *handle, const int16_t* depth, float* pcl, int32_t pointCount);

/**
* Get point cloud data from distance data. The distance data is
* usually calcuated using Gth_GetDepthF32andAmplitudeData.
*
* @param handle [in] specified camera handle.
* @param depth [in] pointer to a buffer in which to store depth data in float(unit: meter).
* @param pcl [out] point clound buffer. each 3 element consists a (x,y,z) point.
*                  (x,y,z) is coordinate in meter unit. point in value (0,0,0) is invalid.
* @param pointCount [out] point cloud float element count.
* @return int [out] return 0 for success.
*/
__API int Gth_GetXYZDataF32_f(const Gth_Dev_Handle *handle, const float* depth, float* pcl, int32_t pointCount);

/**
* Get point cloud data from distance data. The distance data is
* usually calcuated using Gth_GetDepthI16andAmplitudeData.
*
* @param handle [in] specified camera handle.
* @param depth [in] pointer to a buffer in which to store depth data in int_16(unit: millimeter),
* @param pcl [out] point clound buffer. each 3 element consists a (x,y,z) point.
*                  (x,y,z) is coordinate in meter unit. point in value (0,0,0) is invalid.
* @param pointCount [out] point cloud float element count.
* @return int [out] return 0 for success.
*/
__API int Gth_GetXYZDataI16(const Gth_Dev_Handle *handle, const int16_t* depth, int16_t* pcl, int32_t pointCount);

/**
 * convert dist_i16 image (pixel in millimeter) to pesudo-RGB points
 * with specified pixel format
 *
 * @param handle [in] specified camera handle..
 * @param dst [out] pseudo-RGB point buffer.
 * @param dst_len [in] point buffer size in bytes.
 * @param src [in] int_16 points buffer(unit: millimeter).
 * @param src_len [in] count of int_16 points.
 * @param outfmt [in] pixel format of the pseudo-RGB.
 * @param range_min_m [in] minimum range of source point(unit: millimeter).
 * @param range_max_m [in] max range of source point(unit: millimeter).
 *
 * @return int [out] return 0 for success.
 */
__API int Gth_DepthI16ToRGB(const Gth_Dev_Handle *handle, uint8_t *dst, int dst_len, const int16_t *src, int src_len, int16_t range_min_m, int16_t range_max_m);

/**
 * convert dist_f32 image (pixel in meter) to pesudo-RGB points
 * with specified pixel format
 *
 * @param handle [in] specified camera handle.
 * @param dst [out] pseudo-RGB point buffer.
 * @param dst_len [in] point buffer size in bytes.
 * @param src [in] float points buffer(unit: meter).
 * @param src_len [in] count of float points.
 * @param outfmt [in] pixel format of the pseudo-RGB.
 * @param range_min_m [in] minimum range of source point(unit: meter).
 * @param range_max_m [in] max range of source point(unit: meter).
 *
 * @return int [out] return 0 for success.
 */
__API int Gth_DepthF32ToRGB(const Gth_Dev_Handle *handle, uint8_t *dst, int dst_len, const float *src, int src_len, float range_min_m, float range_max_m);

/**
 * convert amplitude_i16 data to IR image whose pixel is in [0~255].
 *
 * @param handle [in] specified camera handle.
 * @param dst [out] IR image buffer.
 * @param dst_len [in] IR image buffer size in bytes.
 * @param src [in] amplitude_i16 data.
 * @param src_len [in] count of u16 points in amplitude_i16 data.
 * @param balance [darkest, brightest].
 *
 * @return int [out] return 0 for success.
 */
__API int Gth_AmplitudeToIR(const Gth_Dev_Handle *handle, uint8_t *dst, int dst_len, const int16_t *src, int src_len, int balance);

/**
 * Get raw 4 phases data from a frame, only use in Mode_RawPhases frame mode.
 *
 * @param handle [in] specified camera handle.
 * @param frame [int] IR image buffer.
 * @param phase1 [out] pointer to a pointer in which to store 0° raw phase.
 * @param phase2 [out] pointer to a pointer in which to store 90° raw phase.
 * @param phase3 [out] pointer to a pointer in which to store 180° raw phase.
 * @param phase4 [out] pointer to a pointer in which to store 270° raw phase.
 *
 * @return int [out] return 0 for success.
 */
__API int Gth_GetRawPhasesData(const Gth_Dev_Handle *handle, const Gth_Frame *frame, int16_t **phase1, int16_t **phase2, int16_t **phase3, int16_t **phase4);

/**
 * Get raw 8 phases data from a frame, only use in Mode_RawPhases frame mode and Mode_Range_WDR range mode.
 *
 * @param handle [in] specified camera handle.
 * @param frame [int] IR image buffer.
 * @param phase1_1 [out] pointer to a pointer in which to store 0° raw phase of short exposure.
 * @param phase2_1 [out] pointer to a pointer in which to store 90° raw phase of short exposure.
 * @param phase3_1 [out] pointer to a pointer in which to store 180° raw phase of short exposure.
 * @param phase4_1 [out] pointer to a pointer in which to store 270° raw phase of short exposure.
 * @param phase1_2 [out] pointer to a pointer in which to store 0° raw phase of long exposure.
 * @param phase2_2 [out] pointer to a pointer in which to store 90° raw phase of long exposure.
 * @param phase3_2 [out] pointer to a pointer in which to store 180° raw phase of long exposure.
 * @param phase4_2 [out] pointer to a pointer in which to store 270° raw phase of long exposure.
 * @return int [out] return 0 for success.
 */
__API int Gth_GetRawPhasesDataWDR(const Gth_Dev_Handle *handle, const Gth_Frame *frame, int16_t **phase1_1, int16_t **phase2_1, int16_t **phase3_1, int16_t **phase4_1, int16_t **phase1_2, int16_t **phase2_2, int16_t **phase3_2, int16_t **phase4_2);

/**
* Get depth_i16 data from the a frame, onl use in Mode_DistAmp frame mode.
*
* @param handle [in] specified camera handle.
* @param frame [in] pointer to a buffer in which to store image data.
* @param depth [out] pointer to a buffer in which to store depth data in int_16(unit: millimeter),
* @return int [out] return 0 for success.
*/
__API int Gth_GetDepthData(const Gth_Dev_Handle *handle, const Gth_Frame* frame, int16_t** depth);

/**
* Get amplitude_i16 data from the a frame, onl use in Mode_DistAmp frame mode.
*
* @param handle [in] specified camera handle.
* @param frame [in] pointer to a buffer in which to store image data.
* @param amplitude [out] pointer to a buffer in which to store amplitude data in int_16.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GeAmplitudeData(const Gth_Dev_Handle *handle, const Gth_Frame* frame, int16_t** Amplitude);

/**
* Get the support lists of range mode.
*
* @param handle [in] specified camera handle.
* @param rangeModeList [out] pointer to store range mode list.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetRangeModeList(const Gth_Dev_Handle *handle, Gth_Range_Mode_List* rangeModeList);

/**
* Set frame mode for camera.
*
* @param handle [in] specified camera handle.
* @param frameMode [in] the frame mode. See Gth_Frame_Mode for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetFrameMode(const Gth_Dev_Handle *handle, const Gth_Frame_Mode frameMode);

/**
* Get frame mode from camera.
*
* @param handle [in] specified camera handle.
* @param frameMode [out] the frame mode. See Gth_Frame_Mode for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetFrameMode(const Gth_Dev_Handle *handle, Gth_Frame_Mode *frameMode);

/**
* Set range mode for camera.
*
* @param handle [in] specified camera handle.
* @param frameMode [in] the range mode. See Gth_Range_Mode for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetRangeMode(const Gth_Dev_Handle *handle, const Gth_Range_Mode rangeMode);

/**
* Get range mode from camera.
*
* @param handle [in] specified camera handle.
* @param rangeMode [out] the range mode. See Gth_Range_Mode for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetRangeMode(const Gth_Dev_Handle *handle, Gth_Range_Mode *rangeMode);

/**
* Set integrationTime for camera.
*
* @param handle [in] specified camera handle.
* @param integrationTime [in] the integrationTime.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetIntegrationTime(const Gth_Dev_Handle *handle, const uint32_t integrationTime);

/**
* Get integrationTime from camera.
*
* @param handle [in] specified camera handle.
* @param integrationTime [out] the integrationTime.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetIntegrationTime(const Gth_Dev_Handle *handle, uint32_t* integrationTime);

/**
* Set gain for camera.
*
* @param handle [in] specified camera handle.
* @param gain [in] the gain.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetGain(const Gth_Dev_Handle *handle, const uint32_t gain);

/**
* Get gain from camera.
*
* @param handle [in] specified camera handle.
* @param gain [out] the gain, only supports 1/2/4.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetGain(const Gth_Dev_Handle *handle, uint32_t* gain);

/**
* Set frame rate for camera.
*
* @param handle [in] specified camera handle.
* @param frameRate [in] the frame rate.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetFrameRate(const Gth_Dev_Handle *handle, const uint32_t frameRate);

/**
* Get frame rate from camera.
*
* @param handle [in] specified camera handle.
* @param gain [out] the frame rate.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetFrameRate(const Gth_Dev_Handle *handle, uint32_t* frameRate);

/**
* Set modulation frequency rate for camera.
*
* @param handle [in] specified camera handle.
* @param modulationFrequency [in] the modulation frequency.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetModulationFrequency(const Gth_Dev_Handle *handle, const uint32_t modulationFrequency);

/**
* Get modulation frequency from camera.
*
* @param handle [in] specified camera handle.
* @param modulationFrequency [out] the modulation frequency.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetModulationFrequency(const Gth_Dev_Handle *handle, uint32_t* modulationFrequency);

/**
* Set amplitude threshold rate for camera.
*
* @param handle [in] specified camera handle.
* @param modulationFrequency [in] the amplitude threshold.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetAmplitudeThreshold(const Gth_Dev_Handle *handle, const uint32_t amplitudeThreshold);

/**
* Get amplitude threshold from camera.
*
* @param handle [in] specified camera handle.
* @param amplitudeThreshold [out] the amplitude threshold.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetAmplitudeThreshold(const Gth_Dev_Handle *handle, uint32_t* amplitudeThreshold);

/**
* Set distance offset rate for camera.
*
* @param handle [in] specified camera handle.
* @param distanceOffset [in] the distance offset.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetDistanceOffset(const Gth_Dev_Handle *handle, const int32_t distanceOffset);

/**
* Get distance offset from camera.
*
* @param handle [in] specified camera handle.
* @param distanceOffset [out] the distance offset.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetDistanceOffset(const Gth_Dev_Handle *handle, int32_t* distanceOffset);

/**
* Set depth range for camera.
*
* @param handle [in] specified camera handle.
* @param minDepthRange [in] minimum range of source point.
* @param maxDepthRange [in] max range of source point.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetDepthRange(const Gth_Dev_Handle *handle, const uint32_t minDepthRange, const uint32_t maxDepthRange);

/**
* Get depth range for camera.
*
* @param handle [in] specified camera handle.
* @param minDepthRange [out] minimum range of source point.
* @param maxDepthRange [out] max range of source point.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetDepthRange(const Gth_Dev_Handle *handle, uint32_t* minDepthRange, uint32_t* maxDepthRange);

/**
* Set all parameters for camera.
*
* @param handle [in] specified camera handle.
* @param para [in] the all parameters. See Gth_Parameters for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetAllParameters(const Gth_Dev_Handle *handle, const Gth_Parameters* para);

/**
* Get all parameters from camera.
*
* @param handle [in] specified camera handle.
* @param para [out] the all parameters. See Gth_Parameters for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetAllParameters(const Gth_Dev_Handle *handle, Gth_Parameters* para);

/**
* Download file from camera.
*
* @param handle [in] specified camera handle.
* @param filename [out] the file name.
*
* @return int [out] return 0 for success.
*/
__API int Gth_DownloadFile(const Gth_Dev_Handle *handle, const char* filename);

/**
* Upload file to camera.
*
* @param handle [in] specified camera handle.
* @param filename [out] the file name.
*
* @return int [out] return 0 for success.
*/
__API int Gth_UploadFile(const Gth_Dev_Handle *handle, const char *filename);

/**
* Read resisters data from camera.
*
* @param handle [in] specified camera handle.
* @param address [in] resister address.
* @param data [out] the resister data which read from camera.
* @param registerCount [int] the count of resisters.
*
* @return int [out] return 0 for success.
*/
__API int Gth_ReadRegister(const Gth_Dev_Handle *handle, const uint16_t *address, uint16_t *data, const uint32_t registerCount);

/**
* Write resister datas to camera.
*
* @param handle [in] specified camera handle.
* @param address [in] resister address.
* @param data [in] the resister data which write to camera.
* @param registerCount [int] the count of resisters.
*
* @return int [out] return 0 for success.
*/
__API int Gth_WriteRegister(const Gth_Dev_Handle *handle, const uint16_t *address, const uint16_t *data, const uint32_t registerCount);

/**
* Get camera extrinsic parameters.
*
* @param handle [in] specified camera handle.
* @param extrinsicPara [out] resister address.
*
* @return int [out] return 0 for success.
*/
__API int Gth_GetCameraExtrinsicParameters(const Gth_Dev_Handle *handle, Gth_Internal_Parameters *extrinsicPara);

/**
* Set the FlipMirror feature.
*
* @param handle [in] specified camera handle.
* @param flipMirror [in] the FlipMirror feature. See Gth_Flip_Mirror for more information.
*
* @return int [out] return 0 for success.
*/
__API int Gth_SetFlipMirror(const Gth_Dev_Handle *handle, const Gth_Flip_Mirror flipMirror);

#ifdef __cplusplus
}
#endif
#endif //GTH_TOF_API_H
