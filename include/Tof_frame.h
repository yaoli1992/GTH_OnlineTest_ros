
#ifndef TOF_FRAME_H_INCLUDED
#define TOF_FRAME_H_INCLUDED

#include "Tof_status.h"
//#include  "Tof_filters.h"
#include <stdint.h>

#if (defined(WIN32) || defined(WIN64)) && !defined(PLAT_WINDOWS) && !defined(PLAT_LINUX)
#   define PLAT_WINDOWS
#endif

//#if defined(linux) && !defined(PLAT_WINDOWS) && !defined(PLAT_LINUX)
//#   define PLAT_LINUX
//#endif
#define PLAT_LINUX

///     @brief  The TOF_Config shall be 8-byte aligned
#define TOF_CONFIG_STRUCT_STRIDE 8
#ifdef PLAT_WINDOWS
#define TOF_PRAGMA_ALIGN __declspec(align(TOF_CONFIG_STRUCT_STRIDE))
#elif defined PLAT_LINUX
#define TOF_PRAGMA_ALIGN __attribute__((aligned(TOF_CONFIG_STRUCT_STRIDE)))
#endif

typedef enum TOF_DeviceType {
    TOF_DeviceTypeGenericEth = 0x0001,
    TOF_DeviceTypeGenericUsb = 0x0002,
    TOF_DeviceTypeGenericUart = 0x0003,
    TOF_DeviceTypeBltstream = 0x000f,
} TOF_DeviceType;

///     @brief Enumerator with valid frame modes to be passed with TOFsetFrameMode
typedef enum TOF_FrameMode {
    TOF_FrameModeDistAmp = 0,
    TOF_FrameModeRawPhases,
    TOF_FrameModeRaw8Phases,
    TOF_FrameModeLongShortExposure,
    TOF_FrameModeNULL,
} TOF_FrameMode;

typedef struct TOF_Frame{
    uint16_t index;			/**< 帧序号 */

    TOF_FrameMode mode;  /**< 帧模式 */

    uint16_t width;		/**< 帧宽 */
    uint16_t height;    /**< 帧高 */

    uint8_t *data;      /**< 帧数据 */
    uint32_t size;      /**< 帧所占用空间大小 */

    float vesalTemp;
    float main_Temp;
    float genericTemp;

    uint8_t paraIndex;
} TOF_Frame;

typedef void (*TOF_FrameArrived)(TOF_Frame &frame);


///     @brief  Configuration structure to be passed with TOFopen
typedef struct TOF_Config {
    TOF_PRAGMA_ALIGN TOF_DeviceType deviceType;             ///< The device type, when not left 0 implies the type of connection to use (Ethernet, USB (P100), UART, Bltstream, ...)

    TOF_PRAGMA_ALIGN char *udpDataIpAddr;                ///< The IP address for the UDP data interface (The address the device is configured to stream to)
    TOF_PRAGMA_ALIGN uint16_t udpDataPort;                  ///< The port for the UDP data interface (The port the device is configured to stream to)

    TOF_PRAGMA_ALIGN char *tcpDeviceIpAddr;              ///< The IP address for the TCP data and control interface (The device's IP address)
    TOF_PRAGMA_ALIGN uint16_t tcpDataPort;                  ///< The port for the TCP data interface (The port the device sends data to) (not supported yet)

    TOF_PRAGMA_ALIGN uint8_t *uartPortName;                 ///< The port name of the UART to use (ASCII coded)
    TOF_PRAGMA_ALIGN uint32_t uartBaudRate;                 ///< The UART baud rate
    TOF_PRAGMA_ALIGN uint8_t uartDataBits;                  ///< The number of UART data bits used
    TOF_PRAGMA_ALIGN uint8_t uartStopBits;                  ///< 0: None, 1: One, 2: Two, 3: 1.5 stop bits
    TOF_PRAGMA_ALIGN uint8_t uartParity;                    ///< 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space Parity
    TOF_PRAGMA_ALIGN uint8_t uartTransmitterAddress;        ///< The source address for UART communications
    TOF_PRAGMA_ALIGN uint8_t uartReceiverAddress;           ///< The target address for UART communications

    TOF_PRAGMA_ALIGN TOF_FrameArrived frameArrived;      ///< Callback function pointer to the function to be called when a frame is ready (optional)

    TOF_PRAGMA_ALIGN uint8_t *pon;                          ///< Product Order Number of device to be opened (0 == not specified) (ASCII coded)
    TOF_PRAGMA_ALIGN uint32_t serialNumber;                 ///< Serial number of device to be opened (0 == not specified)
} TOF_Config;




///     @brief Enumerator with channel IDs. They allow the identification of the various channels in a TOF_Frame
typedef enum TOF_ChannelId {
    TOF_ChannelIdUnknown =        0x0,
    TOF_ChannelIdDistance =       0x1,
    TOF_ChannelIdAmplitude =      0x2,
    TOF_ChannelIdX =              0x4,
    TOF_ChannelIdY =              0x8,
    TOF_ChannelIdZ =             0x10,
    TOF_ChannelIdConfidence =    0x20,
    TOF_ChannelIdPhase0 =        0x80,
    TOF_ChannelIdPhase90 =      0x100,
    TOF_ChannelIdPhase180 =     0x200,
    TOF_ChannelIdPhase270 =     0x400,
    TOF_ChannelIdRawPhase =      0x81,
} TOF_ChannelId;

typedef enum TOF_MetadataId {
    TOF_MetadataIdChessboardCorners     = 0xab8471f9,
    TOF_MetadataIdMlxMeta1              = 0xa720b906,
    TOF_MetadataIdMlxMeta2              = 0xa720b907,
    TOF_MetadataIdMlxTest               = 0xa720b908,
    TOF_MetadataIdMlxAdcData            = 0xa720b909,
} TOF_MetadataId;


///     @brief TOF_Channel holds a two-dimensional array of data  (A part of TOF_Frame)
typedef struct TOF_Metadata {
    TOF_MetadataId id;              ///< Type of metadata. Needs to be specified outside. The TOF is not aware of its meaning
    void *data;                     ///< The data
    uint32_t dataLen;               ///< The length of data in bytes
} TOF_Metadata;


///     @brief TOF_Channel holds a two-dimensional array of data  (A part of TOF_Frame)
typedef struct TOF_Channel {
    TOF_ChannelId id;                 ///< Type of data in this channel
    uint16_t xRes;                    ///< Number of columns
    uint16_t yRes;                    ///< Number of rows
/*    TOF_DataFormat dataFormat;        ///< The bytestream in data needs to be casted to this format
    TOF_Unit unit;  */                  ///< Informative, for easier interpretation of the data
    uint32_t integrationTime;         ///< Integration time at which the frame was captured in [us]
    uint32_t modulationFrequency;     ///< Modulation frequency at which the frame was captured in [Hz]
    uint8_t *data;                    ///< Pixels starting with upper left pixel. For nofBytesPerPixel bigger than 1 the first byte is the lowbyte
    uint32_t dataLen;                 ///< Length of the channel data in bytes (= xRes*yRes*bytesPerPixel)
    TOF_Metadata **metadata;          ///< List of pointers to additional generic data
    uint32_t metadataLen;             ///< The number of TOF_Metadata pointers stored in metadata
} TOF_Channel;


///     @brief TOF_Frame holds all the data gathered from one frame (one or more channels)
//typedef struct TOF_Frame {
//    uint8_t firmwareVersionMajor;       ///< Firmware version major
//    uint8_t firmwareVersionMinor;       ///< Firmware version minor
//    uint8_t firmwareVersionNonFunc;     ///< Firmware version non functional
//    float mainTemp;                     ///< Main-board/processor temperature sensor in degree Celcius
//    float ledTemp;                      ///< Led-board temperature sensor in degree Celcius
//    float genericTemp;                  ///< Additional Generic temperature sensor in degree Celcius
//    uint32_t frameCounter;              ///< Consecutive numbering of frames
//    uint32_t timeStamp;                 ///< Time-stamp at which the frame was captured (in microseconds) (max 1h 11m 34s 967ms 295 us)
//    TOF_Channel **channels;             ///< Data containing channelsLen Channel structs
//    uint8_t channelsLen;                ///< The number of TOF_Channel pointers stored in channels
//    uint8_t sequenceCounter;            ///< If multiple sequences were captured, they can be seperated by the sequence counter
//} TOF_Frame;

typedef struct TOF_Parameters{
    uint32_t integrationTime;
    uint32_t integrationTime_short;
    float frameRate;
    uint32_t modulationFrequency;
    uint32_t modulationFrequency_short;
    uint32_t inputFrequency;
    int32_t distanceOffset;
    uint32_t amplitudeThreshold;
    TOF_FrameMode frameMode;
    uint32_t minDepthRange;
    uint32_t maxDepthRange;
    uint32_t illuminationPower;
    bool continuousMode;
    bool binning;
    uint32_t gain;
    uint32_t gain_short;
//    TOF_FltType filterType;
//    TOF_FltOrientationConfig flipType;
}TOF_Parameters;

typedef struct TOF_InternalParameters
{
    float f;     //unit:mm /** focus **/
    float pixel_size; // unit:mm  /** pixel size **/
    int u0;  //unit:pixel  /**the center of image**/
    int v0;  //unit:pixel
}TOF_InternalParameters;

#endif
