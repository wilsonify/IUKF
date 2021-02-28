#include <stdio.h>
#include <Windows.h>
#include "yei_threespace_basic_utils.h"


// For a full list of streamable commands refer to "Wired Streaming Mode" section in the
// 3-Space Manual of your sensor
#define TSS_TARE_CURRENT_ORIENTATION 0x60
#define TSS_SET_AXIS_DIRECTION 0x74
#define TSS_GET_TARED_ORIENTATION_AS_QUAT 0x00
#define TSS_GET_TARED_ORIENTATION_AS_PYR 0x01
#define TSS_GET_NORTH_AND_GRAVITY 0x0A
#define TSS_GET_FORWARD_AND_DOWN 0x0B
#define TSS_GET_UNTARED_2_VECTOR_IN_SENSOR_FRAME 0x0C

#define TSS_UPDATE_CURRENT_TIMESTAMP 0x5F

#define TSS_GET_TARED_ORIENTATION_AS_AXI 0x03
#define TSS_READ_TEMP_FAH 0x2C
#define TSS_SET_PEDESTRIAN_TRACKING 0x34
#define TSS_SET_PEDESTRIAN_TRACKING_ON_OFF 0x00
#define TSS_GET_PEDESTRIAN_TRACKING 0x35 
#define TSS_GET_CONFIDENCE_VALUE 0x15
#define TSS_GET_NORMALIZED_UNIT_VEC_ACCELEROMETER 0x22
#define TSS_GET_CORRECTED_LINEAR_ACC_AND_GRAVITY 0x27
#define TSS_GET_CORRECTED_LINEAR_ACC_ONLY 0x29
#define TSS_GET_RAD_PER_SEC_GYROSCOPE 0x26
#define TSS_GET_NORMALIZED_COMPASS 0x23
#define TSS_GET_CORRECTED_COMPASS 0x28
#define TSS_GET_RAW_COMPASS_DATA 0x43
#define TSS_GET_GYRO_CALIBRATE_COEFFS 0xA4


#define TSS_NULL 0xff // No command use to fill the empty slots in "set stream slots"
// For a full list of commands refer to the 3-Space Manual of the sensor
#define TSS_SET_STREAMING_SLOTS 0x50
#define TSS_SET_STREAMING_TIMING 0x52
#define TSS_START_STREAMING 0x55
#define TSS_STOP_STREAMING 0x56
#define THRSHLD 0.75 //Stillness Threshold
#define N 50 // Number of samples used to determine Gyro Bias
  
// Stream data stuctures must be packed else they will not properly work
#pragma pack(push,1)
typedef struct Batch_Data{
    float Data[14];
} Batch_Data;
#pragma pack(pop)

typedef struct vec3f
{
    float x;
    float y;
    float z;
}
vec3f;

typedef struct vec4f
{
    float x;
    float y;
    float z;
	float w;
}
vec4f;

// Streaming mode require the streaming slots and streaming timing being setup prior to start streaming
int setupStreaming(HANDLE com_handle)
{
    DWORD bytes_written;
    unsigned char write_slot_bytes[11];   // start byte, command, data(8), checksum
    unsigned char write_timing_bytes[15]; // start byte, command, data(12), checksum
	unsigned char write_pedestrian_bytes[5];
    unsigned int interval;
    unsigned int duration;
    unsigned int delay;
 
    printf(">> SETTING STREAMING SLOTS VIA >> \n");
	// SET STREAMING SLOTS //
    // There are 8 streaming slots available for use, and each one can hold one of the streamable commands.
    // Unused slots should be filled with 0xff so that they will output nothing.
    write_slot_bytes[0]= TSS_START_BYTE;
    write_slot_bytes[1]= TSS_SET_STREAMING_SLOTS;
    write_slot_bytes[2]= TSS_READ_TEMP_FAH; // stream slot0
    write_slot_bytes[3]= TSS_GET_RAD_PER_SEC_GYROSCOPE; // stream slot1
    write_slot_bytes[4]= TSS_GET_CORRECTED_LINEAR_ACC_AND_GRAVITY; // stream slot2
    write_slot_bytes[5]= TSS_GET_TARED_ORIENTATION_AS_QUAT; // stream slot3
    write_slot_bytes[6]= TSS_GET_NORMALIZED_COMPASS; // stream slot4
    write_slot_bytes[7]= TSS_NULL; // stream slot5
    write_slot_bytes[8]= TSS_NULL; // stream slot6
    write_slot_bytes[9]= TSS_NULL; // stream slot7
    write_slot_bytes[10]= createChecksum(&write_slot_bytes[1], 8+1);

    // Write the bytes to the serial
    if(!WriteFile(com_handle, write_slot_bytes, sizeof(write_slot_bytes), &bytes_written, 0)){
        printf("Error writing to port\n");
        return 1;
    }
 
    // SET STREAMING TIMING //
    // Interval determines how often the streaming session will output data from the requested commands
    // An interval of 0 will output data at the max filter rate
    interval= 100000; // microseconds
    // Duration determines how long the streaming session will run for
    // A duration of 0xffffffff will have the streaming session run till the stop stream command is called
    //duration= STREAM_DURATION*1000000; // microseconds
	duration= 0xffffffff; // microseconds
    // Delay determines how long the sensor should wait after a start command is issued to actually begin
    // streaming
    delay= 100000; //microseconds

    //The data must be flipped to big endian before sending to sensor
    endian_swap_32((unsigned int *)&interval);
    endian_swap_32((unsigned int *)&duration);
    endian_swap_32((unsigned int *)&delay);
 
    write_timing_bytes[0]= TSS_START_BYTE;
    write_timing_bytes[1]= TSS_SET_STREAMING_TIMING;
    memcpy(&write_timing_bytes[2], &interval, sizeof(interval));
    memcpy(&write_timing_bytes[6], &duration, sizeof(duration));
    memcpy(&write_timing_bytes[10], &delay, sizeof(delay));
    write_timing_bytes[sizeof(write_timing_bytes)-1]= createChecksum(&write_timing_bytes[1], 12+1);
    // Write the bytes to the serial
    if(!WriteFile(com_handle, write_timing_bytes, sizeof(write_timing_bytes), &bytes_written, 0)){
        printf("Error writing to port\n");
        return 2;
    }
    return 0;
}