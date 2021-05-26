#ifndef GTH_TOF_TYPES_H
#define GTH_TOF_TYPES_H

#include <stdint.h>
#include "GthTofEnums.h"



struct Gth_Dev_Info_Usb {
    uint8_t usb_bus_num;
    uint8_t usb_port_num;
    uint8_t usb_dev_addr;
    uint8_t usb_speed;
};

struct Gth_Dev_Info_Eth {
    char* addr;
    uint16_t port;
};

struct Gth_Dev_Version{
    uint64_t dev_id;  /* device uniq id */
    /*  device info  */
    char platform_info[32];
    char software_info[32];
    char firmware_info[32];
};

struct Gth_Dev_Handle {
    void *handler;
    Gth_Dev_Version dev_version;
};

struct Gth_Range_Mode_List
{
    uint8_t count;
    uint8_t list[32];
};

struct Gth_Frame{
    uint16_t index;			/**< 帧序号 */
    Gth_Frame_Mode frame_Mode;  /**< 帧模式 */
    uint16_t width;		/**< 帧宽 */
    uint16_t height;    /**< 帧高 */
    uint8_t *data;      /**< 帧数据 */
    uint32_t size;      /**< 帧所占用空间大小 */
    float vcselTemp;
    float main_Temp;
    float sensor_Temp;
};

struct Gth_Parameters{
    uint32_t modulationFrequency;
    uint32_t amplitudeThreshold;
    Gth_Frame_Mode frameMode;
    Gth_Range_Mode rangeMode;
    uint32_t integrationTime;
    int32_t distanceOffset;
    uint32_t minDepthRange;
    uint32_t maxDepthRange;
    uint32_t frameRate;
    uint32_t gain;
};

struct Gth_Internal_Parameters
{
    float f;     //unit:mm /** focus **/
    float pixel_size; // unit:mm  /** pixel size **/
    int u0;  //unit:pixel  /**the center of image**/
    int v0;  //unit:pixel
};

typedef void (*Gth_FrameArrived)(Gth_Frame* frame);

typedef struct {
    Gth_Dev_Type type; // interface type
    Gth_FrameArrived frameReady;
    union {
        struct Gth_Dev_Info_Usb usb;
        struct Gth_Dev_Info_Eth eth;
    } Info;
}Gth_Dev_Info;

#endif //GTH_TOF_TYPES_H
