#ifndef TOF_STATUS_H_INCLUDED
#define TOF_STATUS_H_INCLUDED

#include <stdint.h>

///   @brief Error code for error handling
typedef enum TOF_Status {
    TOF_StatusOk = 0,                       ///< Everything went ok

    // Errors
    TOF_StatusInvalidParameter = -32768,    ///< At least one parameter passed is invalid or out of valid range
                                            ///< The register address provided is outside the valid range
                                            ///< The combination of parameters is contradictory or incomplete
                                            ///< The provided frame does not contain the channel(s) expected
    TOF_StatusIllegalOperation,             ///< The data requested by the user cannot be read / written because it does not exist or is not accessible in the current configuration / state
                                            ///< The modulation frequency to be set or currently configured is not supported
                                            ///< TOFclose was already called
    TOF_StatusTimeOut,                      ///< Within the waiting period a necessary condition was not met, so the operation had to be aborted
                                            ///< After trying repeatedly the operation did not succeed
    TOF_StatusDeviceUnreachable,            ///< The connection to the device could not be established
                                            ///< An error occurred during communication
                                            ///< The device with the specified attributes could not be found
    TOF_StatusNotConnected,                 ///< The operation cannot be executed because the connection is down
    TOF_StatusInvalidVersion,               ///<
    TOF_StatusRuntimeError,                 ///< A system resource (mutex, semaphore, thread, file) could not be created / initialized / read / written
                                            ///< The ToF device did not react as expected
    TOF_StatusOutOfMemory,                  ///< A malloc, realloc or calloc failed to reserve the needed memory
                                            ///< The buffer provided by the caller is not large enough
                                            ///< The end of the file was reached
    TOF_StatusNotSupported,                 ///< The function is not supported by this device/firmware
                                            ///<
    TOF_StatusCrcError,                     ///< The cyclic redundancy check revealed that the data in question must be corrupt
    TOF_StatusUnknown,

    // These stati are used only in infoEvent callbacks, it is merely a placeholder rather than a state
    TOF_StatusInformation = 1,              ///< The infoEvent message contains the actual information
    TOF_StatusWarning,                      ///< The infoEvent message describes the cause of the warning
    TOF_StatusConfigFailed,
} TOF_Status;


/// @brief  Deprecated TOF_EventId now only represents a TOF_Status
typedef TOF_Status TOF_EventId;

#endif
