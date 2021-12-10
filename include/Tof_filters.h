#ifndef TOF_FILTERS_H_INCLUDED
#define TOF_FILTERS_H_INCLUDED
#include <stdint.h>
#include "Tof_frame.h"
typedef void* TOF_FltConfig;
typedef void* TOF_FltHandle;


/// @brief  This enum defines which filter is to be instantiated
typedef enum TOF_FltType {
    TOF_FltTypeIntrinsicUndistort,      ///< Intrinsic lens parameters are used to undistort the data
    TOF_FltTypeMath,                    ///< Basic mathematical operations
    TOF_FltTypeAvgSequences,            ///< Averaging of multiple sequences into one frame
#   ifndef TOF_EXCLUDE_FILTERS
    TOF_FltTypePixelIntrpl,             ///< A pixel can be replaced by the average value of a set of pixels
    TOF_FltTypeMotionDetector,          ///< A sliding average is used to calculate the satic background. Motion is then filtered
    TOF_FltTypeMorphology,              ///< Basic morphology functions (dilation, erosion)
    TOF_FltTypeOrientation,             ///< Filter for rotating and flipping data
    TOF_FltTypeAvgPixels,				///< Average all pixels of input frame seperately for each channel
    TOF_FltTypeDct,                     ///< DCT computation for sharpness factor
    TOF_FltTypeCropChessboard,          ///< If a chessboard is recognized, all is cropped but the chessboard
    TOF_FltTypeLaplace,                 ///< Apply a laplace operator to a channel
    TOF_FltTypeFocusPreproc,            ///< Propretary preprocessing for focus calibration
    TOF_FltTypeCrop,                    ///< Crop to ROI
    TOF_FltTypeCombine,                 ///< Combine channels patching them together
    TOF_FltTypeGaussianBlur,            ///< Apply a gaussian filter
    TOF_FltTypeFindChessboard,          ///< The defined chessboard is found, the corners are drawn on the image data if so specified and the corner data is added as metadata
    TOF_FltTypeRmsFrames,				///< Apply a root mean sqare filter
#   endif
} TOF_FltType;



typedef struct TOF_FltIntrinsicUndistortConfig {
    //TOF_ChannelId channelToProcess;       //TBA
    //TOF_DataFormat dataFormatToProcess;   //TBA
    uint16_t xRes;          ///< this filter is meant for input data of this width (0: all sizes)
    uint16_t yRes;          ///< this filter is meant for input data of this height (0: all sizes)
    float cameraMatrix[9];  ///< 3x3 Matrix: camera matrix
    float distCoeffs[5];    ///< 5x1 Vector: distortion coefficients
} TOF_FltIntrinsicUndistortConfig;



typedef enum TOF_FltMathType {
    TOF_FltMathTypeMultFM1,
    TOF_FltMathTypeAbs,
    TOF_FltMathTypeMax,
} TOF_FltMathType;

typedef struct TOF_FltMathConfig {
    TOF_FltMathType mathType;
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
    float *dataFM1;                      ///< Float Matrix #1
    uint16_t xResFM1;                    ///< Number of columns of FM1
    uint16_t yResFM1;                    ///< Number of rows of FM1
} TOF_FltMathConfig;



typedef struct TOF_FltAvgSequencesConfig {
    //TOF_ChannelId channel(s)ToProcess ///< The filter currently averages only Distance, X, Y, Z and ORs Flags
    uint16_t averageWindowLength;       ///< The filter expects this many sequences to be averaged
    //values not to average             ///< The filter currently ignores 0 pixels. if the pixel is invalid in half the sequences the whole average is set to 0
} TOF_FltAvgSequencesConfig;


#ifndef TOF_EXCLUDE_FILTERS


typedef struct TOF_FltAvgPixelsConfig {
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
} TOF_FltAvgPixelsConfig;


typedef struct TOF_FltPixelIntrplConfig {
    uint32_t **pxIndicesIn;
    uint16_t *pxIndicesInLens;
    uint32_t *pxIndicesOut;
    uint16_t pxCount;
} TOF_FltPixelIntrplConfig;



typedef struct TOF_FltMotionDetectorConfig {
    uint16_t slafWindowLength;          ///< The background adaption is done by a sliding average
    uint8_t slafStride;                 ///< The sliding average can be configured to average every nth frame
    uint16_t threshold;                 ///< If the current pixel exceeds the background by this amount: activity
} TOF_FltMotionDetectorConfig;



typedef enum TOF_FltMorphologyType {
    TOF_FltMorphologyTypeDilation,
    TOF_FltMorphologyTypeErosion,
} TOF_FltMorphologyType;

typedef struct TOF_FltMorphologyConfig {
    TOF_ChannelId channelToProcess;
    TOF_FltMorphologyType morphologyType;
    uint8_t *mask;
    uint8_t xRes;
    uint8_t yRes;
} TOF_FltMorphologyConfig;



typedef enum TOF_FltOrientationType {
    TOF_FltOrientationTypeFlipHor,
    TOF_FltOrientationTypeFlipVer,
    TOF_FltOrientationTypeRotate,
} TOF_FltOrientationType;

typedef struct TOF_FltOrientationConfig {
    TOF_FltOrientationType orientationType;
    uint16_t degrees;
} TOF_FltOrientationConfig;



typedef struct TOF_FltBilateralConfig {
    TOF_ChannelId channelToProcess;
} TOF_FltBilateralConfig;



typedef struct TOF_FltDctConfig {
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
} TOF_FltDctConfig;



typedef struct TOF_FltCropChessboardConfig {
    TOF_ChannelId channelToProcess;
    float scaleFactor;                  ///< upscales or downscales the input before calling findChessboardCorners (should be scaled for functionality and performance)
    uint8_t edgeCountHor;               ///< The number of edges to be fount horizontally
    uint8_t edgeCountVert;              ///< The number of edges to be fount vertically
    float border;                       ///< 0 for no border, <0 for cropping >0 for border
    TOF_ChannelId channelIdResult;
} TOF_FltCropChessboardConfig;



typedef struct TOF_FltLaplaceConfig {
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
} TOF_FltLaplaceConfig;



typedef struct TOF_FltFocusPreprocConfig {
    TOF_ChannelId channelToProcess;
    float scaleFactorForChessboard;
    uint8_t edgeCountHorForChessboard;
    uint8_t edgeCountVertForChessboard;
    TOF_ChannelId channelIdResult;
} TOF_FltFocusPreprocConfig;


typedef struct TOF_FltCropConfig {
    TOF_ChannelId channelToProcess;
    float roiStartX;                ///< x coordinate for ROI to preserve including this coordinate. If relative (0 >= x < 1) -> multiplied by xRes.
    float roiStartY;                ///< x coordinate for ROI to preserve including this coordinate. If relative (0 >= x < 1) -> multiplied by xRes.
    float roiEndX;                  ///< x coordinate for ROI to preserve including this coordinate. If relative (0 >= x < 1) -> multiplied by xRes.
    float roiEndY;                  ///< x coordinate for ROI to preserve including this coordinate. If relative (0 >= x < 1) -> multiplied by xRes.
    TOF_ChannelId channelIdResult;
} TOF_FltCropConfig;


typedef struct TOF_FltCombineConfig {
    TOF_ChannelId channelToProcess;
    uint8_t removeOriginals;
} TOF_FltCombineConfig;


typedef struct TOF_FltGaussianBlurConfig {
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
} TOF_FltGaussianBlurConfig;


typedef struct TOFFltFindChessboardConfig {
    TOF_ChannelId channelToProcess;
    float scaleFactor;                  ///< upscales or downscales the input before calling findChessboardCorners (should be scaled for functionality and performance)
    uint8_t edgeCountHor;               ///< The number of edges to be fount horizontally
    uint8_t edgeCountVert;              ///< The number of edges to be fount vertically
} TOF_FltFindChessboardConfig;

typedef struct TOFFltRmsFramesConfig {
    TOF_ChannelId channelToProcess;
    TOF_ChannelId channelIdResult;
	uint16_t windowLength;				///< The number frames averaged
} TOF_FltRmsFramesConfig;
#endif
#endif
