#ifndef GTH_TOF_ENUMS_H
#define GTH_TOF_ENUMS_H

typedef enum {
    Dev_Usb = 0,
    Dev_Eth,
    Dev_None,
}Gth_Dev_Type;

typedef enum {
    Mode_DistAmp = 0,
    Mode_RawPhases,
    Mode_None,
} Gth_Frame_Mode;

typedef enum {
    Mode_Range_None = 0,
    Mode_Range_S,
    Mode_Range_M,
    Mode_Range_L,
    Mode_Range_XL,
    Mode_Range_Custom,
    Mode_Range_WDR,
} Gth_Range_Mode;

typedef enum {
    GTH_LOG_LEVEL_TRACE = 0,
    GTH_LOG_LEVEL_DEBUG,
    GTH_LOG_LEVEL_INFO,
    GTH_LOG_LEVEL_WARN,
    GTH_LOG_LEVEL_ERROR,
    GTH_LOG_LEVEL_ALARM,
    GTH_LOG_LEVEL_FATAL,
} Gth_Log_Level;

/*** Mirror the image along its horizontal and/or vertical central axis ***/
typedef enum {
    Default = 0,
    Flip, // (along horizontal axis)
    Mirror, //(along vertical axis)
    Flip_Mirror,
} Gth_Flip_Mirror;

#endif
