/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_BEECAM_UWB_I2C_UWB_I2C_PROTO_PB_H_INCLUDED
#define PB_BEECAM_UWB_I2C_UWB_I2C_PROTO_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum beecam_uwb_i2c_downlink_error_error_code {
    /* Unspecified error: something went wrong that doesn't fit into existing categories. */
    BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_UNKNOWN = 0,
    /* Request timed out.

 For ranging requests, this most likely means that the receiving node is unavailable
 or out of range. */
    BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_TIMEOUT = 1,
    /* Data encoding or decoding failed. */
    BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_SERDE = 2
} beecam_uwb_i2c_downlink_error_error_code_t;

/* Struct definitions */
/* Unit. */
typedef struct beecam_uwb_i2c_unit {
    char dummy_field;
} beecam_uwb_i2c_unit_t;

typedef PB_BYTES_ARRAY_T(8) beecam_uwb_i2c_peer_address_address_t;
/* 802.15.4 address. */
typedef struct beecam_uwb_i2c_peer_address {
    /* Encoded as u16. */
    uint16_t pan_id;
    /* 2 or 8 bytes (short / extended). */
    beecam_uwb_i2c_peer_address_address_t address;
} beecam_uwb_i2c_peer_address_t;

typedef struct beecam_uwb_i2c_device_info_uwb_info {
    /* Current UWB peer address. */
    bool has_address;
    beecam_uwb_i2c_peer_address_t address;
} beecam_uwb_i2c_device_info_uwb_info_t;

typedef struct beecam_uwb_i2c_device_info_bluetooth_info {
    /* MAC address. */
    pb_byte_t mac[6];
} beecam_uwb_i2c_device_info_bluetooth_info_t;

typedef struct beecam_uwb_i2c_device_info_firmware_build_info {
    /* Git hash firmware was built at. */
    char git_revision[41];
    /* Build timestamp as epoch seconds. */
    char build_timestamp[41];
    /* Compiler version string. */
    char compiler_version[33];
} beecam_uwb_i2c_device_info_firmware_build_info_t;

typedef struct beecam_uwb_i2c_device_info {
    bool has_uwb;
    beecam_uwb_i2c_device_info_uwb_info_t uwb;
    /* Information about the bluetooth stack. May be left unpopulated if the bluetooth stack
 is not running. */
    bool has_bluetooth;
    beecam_uwb_i2c_device_info_bluetooth_info_t bluetooth;
    bool has_build_info;
    beecam_uwb_i2c_device_info_firmware_build_info_t build_info;
} beecam_uwb_i2c_device_info_t;

typedef struct beecam_uwb_i2c_uplink_multi_ptp {
    /* Peer to send request to. */
    bool has_peer;
    beecam_uwb_i2c_peer_address_t peer;
    /* Numer of requests to send. */
    uint32_t n;
    /* Send all individual ranging results rather than a single distribution. */
    bool full_result;
} beecam_uwb_i2c_uplink_multi_ptp_t;

typedef PB_BYTES_ARRAY_T(64) beecam_uwb_i2c_uplink_correlation_data_t;
/* Messages from BeeCam to DWM3001C (master -> slave). */
typedef struct beecam_uwb_i2c_uplink {
    /* Raw bytes supplied by sender to correlate a response to this request.
 Returned verbatim in the responding downlink message. */
    beecam_uwb_i2c_uplink_correlation_data_t correlation_data;
    pb_size_t which_command;
    union {
        /* Request status of the UWB node. */
        beecam_uwb_i2c_unit_t request_status;
        /* Reset MCU. */
        beecam_uwb_i2c_unit_t reset;
        /* Initiate TWR point-to-point with the node at the specified address. */
        beecam_uwb_i2c_peer_address_t twr_ptp;
        /* Get device info. */
        beecam_uwb_i2c_unit_t get_device_info;
        /* Run repeated point-to-point TWRs. */
        beecam_uwb_i2c_uplink_multi_ptp_t multi_ptp;
        /* Set twr delay parameter (in picoseconds). */
        uint64_t set_delay_ps;
    } command;
} beecam_uwb_i2c_uplink_t;

typedef struct beecam_uwb_i2c_downlink_error {
    /* Notes on the circumstances of the error. */
    char context[64];
    beecam_uwb_i2c_downlink_error_error_code_t code;
} beecam_uwb_i2c_downlink_error_t;

/* Status of the DWM3001C system. */
typedef struct beecam_uwb_i2c_downlink_status {
    bool ready;
} beecam_uwb_i2c_downlink_status_t;

/* Point-to-point TWR result. */
typedef struct beecam_uwb_i2c_downlink_ptp_result {
    /* Address of the peer we ranged to. */
    bool has_peer_address;
    beecam_uwb_i2c_peer_address_t peer_address;
    /* Raw tx timestamp as recorded by UWB chip. */
    uint64_t raw_tx_timestamp;
    /* Raw rx timestamp as recorded by UWB chip. The clock may have rolled over since the tx
 packet was transmitted, and the size of the timestamp values is 40 bits. */
    uint64_t raw_rx_timestamp;
    /* Assumed delay. */
    uint64_t delay;
    /* Calculated range in millimeters. */
    int32_t range_mm;
} beecam_uwb_i2c_downlink_ptp_result_t;

typedef struct beecam_uwb_i2c_downlink_normal_distribution {
    float mean;
    float stddev;
    /* Number of samples used to produce this distribution. */
    uint32_t n_samples;
    /* Assumed delay to produce this distribution. */
    uint64_t delay;
} beecam_uwb_i2c_downlink_normal_distribution_t;

/* Multi-PTP result. */
typedef struct beecam_uwb_i2c_downlink_full_multi_ptp_result {
    /* All sampled ranges in mm. */
    pb_size_t ranges_mm_count;
    int32_t ranges_mm[128];
    /* Address of the peer we ranged to. */
    bool has_peer_address;
    beecam_uwb_i2c_peer_address_t peer_address;
    /* Normal distribution derived from the distance range samples. */
    bool has_normal_result;
    beecam_uwb_i2c_downlink_normal_distribution_t normal_result;
    /* All sampled ranges dt samples. */
    pb_size_t ranges_dt_count;
    uint64_t ranges_dt[128];
} beecam_uwb_i2c_downlink_full_multi_ptp_result_t;

typedef PB_BYTES_ARRAY_T(64) beecam_uwb_i2c_downlink_correlation_data_t;
/* Messages from DWM3001C to BeeCam (slave -> master). */
typedef struct beecam_uwb_i2c_downlink {
    /* Raw bytes supplied in the initiating request. */
    beecam_uwb_i2c_downlink_correlation_data_t correlation_data;
    pb_size_t which_response;
    union {
        /* Requested action failed. */
        beecam_uwb_i2c_downlink_error_t error;
        /* System status. */
        beecam_uwb_i2c_downlink_status_t status;
        /* Successful point-to-point TWR result. */
        beecam_uwb_i2c_downlink_ptp_result_t twr_ptp_result;
        /* Information about hardware and firmware. */
        beecam_uwb_i2c_device_info_t info;
        /* Normal distribution of the ranging attempts (mm units). */
        beecam_uwb_i2c_downlink_normal_distribution_t multi_ptp_normal;
        /* Normal distribution of the ranging attempts (mm units). */
        beecam_uwb_i2c_downlink_full_multi_ptp_result_t multi_ptp_full;
        /* Generic success message, currently associated with set_delay_ps. */
        beecam_uwb_i2c_unit_t success;
    } response;
    /* Sample from clock counter. */
    uint32_t clock_sample;
} beecam_uwb_i2c_downlink_t;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_MIN BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_UNKNOWN
#define _BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_MAX BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_SERDE
#define _BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_ARRAYSIZE ((beecam_uwb_i2c_downlink_error_error_code_t)(BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_SERDE+1))










#define beecam_uwb_i2c_downlink_error_t_code_ENUMTYPE beecam_uwb_i2c_downlink_error_error_code_t






/* Initializer values for message structs */
#define BEECAM_UWB_I2C_UNIT_INIT_DEFAULT         {0}
#define BEECAM_UWB_I2C_PEER_ADDRESS_INIT_DEFAULT {0, {0, {0}}}
#define BEECAM_UWB_I2C_DEVICE_INFO_INIT_DEFAULT  {false, BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_INIT_DEFAULT, false, BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_INIT_DEFAULT, false, BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_INIT_DEFAULT}
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_INIT_DEFAULT {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_DEFAULT}
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_INIT_DEFAULT {{0}}
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_INIT_DEFAULT {"", "", ""}
#define BEECAM_UWB_I2C_UPLINK_INIT_DEFAULT       {{0, {0}}, 0, {BEECAM_UWB_I2C_UNIT_INIT_DEFAULT}}
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_INIT_DEFAULT {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_DEFAULT, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_INIT_DEFAULT     {{0, {0}}, 0, {BEECAM_UWB_I2C_DOWNLINK_ERROR_INIT_DEFAULT}, 0}
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_INIT_DEFAULT {"", _BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_MIN}
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_INIT_DEFAULT {0}
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_INIT_DEFAULT {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_DEFAULT, 0, 0, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_INIT_DEFAULT {0, 0, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_INIT_DEFAULT {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_DEFAULT, false, BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_INIT_DEFAULT, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
#define BEECAM_UWB_I2C_UNIT_INIT_ZERO            {0}
#define BEECAM_UWB_I2C_PEER_ADDRESS_INIT_ZERO    {0, {0, {0}}}
#define BEECAM_UWB_I2C_DEVICE_INFO_INIT_ZERO     {false, BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_INIT_ZERO, false, BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_INIT_ZERO, false, BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_INIT_ZERO}
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_INIT_ZERO {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_ZERO}
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_INIT_ZERO {{0}}
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_INIT_ZERO {"", "", ""}
#define BEECAM_UWB_I2C_UPLINK_INIT_ZERO          {{0, {0}}, 0, {BEECAM_UWB_I2C_UNIT_INIT_ZERO}}
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_INIT_ZERO {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_ZERO, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_INIT_ZERO        {{0, {0}}, 0, {BEECAM_UWB_I2C_DOWNLINK_ERROR_INIT_ZERO}, 0}
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_INIT_ZERO  {"", _BEECAM_UWB_I2C_DOWNLINK_ERROR_ERROR_CODE_MIN}
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_INIT_ZERO {0}
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_INIT_ZERO {false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_ZERO, 0, 0, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_INIT_ZERO {0, 0, 0, 0}
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_INIT_ZERO {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, BEECAM_UWB_I2C_PEER_ADDRESS_INIT_ZERO, false, BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_INIT_ZERO, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}

/* Field tags (for use in manual encoding/decoding) */
#define BEECAM_UWB_I2C_PEER_ADDRESS_PAN_ID_TAG   1
#define BEECAM_UWB_I2C_PEER_ADDRESS_ADDRESS_TAG  2
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_ADDRESS_TAG 1
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_MAC_TAG 1
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_GIT_REVISION_TAG 1
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_BUILD_TIMESTAMP_TAG 2
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_COMPILER_VERSION_TAG 3
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_TAG       1
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_TAG 2
#define BEECAM_UWB_I2C_DEVICE_INFO_BUILD_INFO_TAG 3
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_PEER_TAG 1
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_N_TAG    2
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_FULL_RESULT_TAG 3
#define BEECAM_UWB_I2C_UPLINK_CORRELATION_DATA_TAG 1
#define BEECAM_UWB_I2C_UPLINK_REQUEST_STATUS_TAG 2
#define BEECAM_UWB_I2C_UPLINK_RESET_TAG          3
#define BEECAM_UWB_I2C_UPLINK_TWR_PTP_TAG        4
#define BEECAM_UWB_I2C_UPLINK_GET_DEVICE_INFO_TAG 5
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_TAG      6
#define BEECAM_UWB_I2C_UPLINK_SET_DELAY_PS_TAG   7
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_CONTEXT_TAG 1
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_CODE_TAG   2
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_READY_TAG 1
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_PEER_ADDRESS_TAG 2
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_RAW_TX_TIMESTAMP_TAG 3
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_RAW_RX_TIMESTAMP_TAG 4
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_DELAY_TAG 6
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_RANGE_MM_TAG 7
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_MEAN_TAG 1
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_STDDEV_TAG 2
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_N_SAMPLES_TAG 3
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_DELAY_TAG 4
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_RANGES_MM_TAG 1
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_PEER_ADDRESS_TAG 2
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_NORMAL_RESULT_TAG 3
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_RANGES_DT_TAG 4
#define BEECAM_UWB_I2C_DOWNLINK_CORRELATION_DATA_TAG 1
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_TAG        2
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_TAG       3
#define BEECAM_UWB_I2C_DOWNLINK_TWR_PTP_RESULT_TAG 4
#define BEECAM_UWB_I2C_DOWNLINK_INFO_TAG         5
#define BEECAM_UWB_I2C_DOWNLINK_MULTI_PTP_NORMAL_TAG 6
#define BEECAM_UWB_I2C_DOWNLINK_MULTI_PTP_FULL_TAG 7
#define BEECAM_UWB_I2C_DOWNLINK_SUCCESS_TAG      8
#define BEECAM_UWB_I2C_DOWNLINK_CLOCK_SAMPLE_TAG 9

/* Struct field encoding specification for nanopb */
#define BEECAM_UWB_I2C_UNIT_FIELDLIST(X, a) \

#define BEECAM_UWB_I2C_UNIT_CALLBACK NULL
#define BEECAM_UWB_I2C_UNIT_DEFAULT NULL

#define BEECAM_UWB_I2C_PEER_ADDRESS_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pan_id,            1) \
X(a, STATIC,   SINGULAR, BYTES,    address,           2)
#define BEECAM_UWB_I2C_PEER_ADDRESS_CALLBACK NULL
#define BEECAM_UWB_I2C_PEER_ADDRESS_DEFAULT NULL

#define BEECAM_UWB_I2C_DEVICE_INFO_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  uwb,               1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  bluetooth,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  build_info,        3)
#define BEECAM_UWB_I2C_DEVICE_INFO_CALLBACK NULL
#define BEECAM_UWB_I2C_DEVICE_INFO_DEFAULT NULL
#define beecam_uwb_i2c_device_info_t_uwb_MSGTYPE beecam_uwb_i2c_device_info_uwb_info_t
#define beecam_uwb_i2c_device_info_t_bluetooth_MSGTYPE beecam_uwb_i2c_device_info_bluetooth_info_t
#define beecam_uwb_i2c_device_info_t_build_info_MSGTYPE beecam_uwb_i2c_device_info_firmware_build_info_t

#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  address,           1)
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_CALLBACK NULL
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_DEFAULT NULL
#define beecam_uwb_i2c_device_info_uwb_info_t_address_MSGTYPE beecam_uwb_i2c_peer_address_t

#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FIXED_LENGTH_BYTES, mac,               1)
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_CALLBACK NULL
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_DEFAULT NULL

#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   git_revision,      1) \
X(a, STATIC,   SINGULAR, STRING,   build_timestamp,   2) \
X(a, STATIC,   SINGULAR, STRING,   compiler_version,   3)
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_CALLBACK NULL
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_DEFAULT NULL

#define BEECAM_UWB_I2C_UPLINK_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BYTES,    correlation_data,   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,request_status,command.request_status),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,reset,command.reset),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,twr_ptp,command.twr_ptp),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,get_device_info,command.get_device_info),   5) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,multi_ptp,command.multi_ptp),   6) \
X(a, STATIC,   ONEOF,    UINT64,   (command,set_delay_ps,command.set_delay_ps),   7)
#define BEECAM_UWB_I2C_UPLINK_CALLBACK NULL
#define BEECAM_UWB_I2C_UPLINK_DEFAULT NULL
#define beecam_uwb_i2c_uplink_t_command_request_status_MSGTYPE beecam_uwb_i2c_unit_t
#define beecam_uwb_i2c_uplink_t_command_reset_MSGTYPE beecam_uwb_i2c_unit_t
#define beecam_uwb_i2c_uplink_t_command_twr_ptp_MSGTYPE beecam_uwb_i2c_peer_address_t
#define beecam_uwb_i2c_uplink_t_command_get_device_info_MSGTYPE beecam_uwb_i2c_unit_t
#define beecam_uwb_i2c_uplink_t_command_multi_ptp_MSGTYPE beecam_uwb_i2c_uplink_multi_ptp_t

#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  peer,              1) \
X(a, STATIC,   SINGULAR, UINT32,   n,                 2) \
X(a, STATIC,   SINGULAR, BOOL,     full_result,       3)
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_CALLBACK NULL
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_DEFAULT NULL
#define beecam_uwb_i2c_uplink_multi_ptp_t_peer_MSGTYPE beecam_uwb_i2c_peer_address_t

#define BEECAM_UWB_I2C_DOWNLINK_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BYTES,    correlation_data,   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,error,response.error),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,status,response.status),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,twr_ptp_result,response.twr_ptp_result),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,info,response.info),   5) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,multi_ptp_normal,response.multi_ptp_normal),   6) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,multi_ptp_full,response.multi_ptp_full),   7) \
X(a, STATIC,   ONEOF,    MESSAGE,  (response,success,response.success),   8) \
X(a, STATIC,   SINGULAR, UINT32,   clock_sample,      9)
#define BEECAM_UWB_I2C_DOWNLINK_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_DEFAULT NULL
#define beecam_uwb_i2c_downlink_t_response_error_MSGTYPE beecam_uwb_i2c_downlink_error_t
#define beecam_uwb_i2c_downlink_t_response_status_MSGTYPE beecam_uwb_i2c_downlink_status_t
#define beecam_uwb_i2c_downlink_t_response_twr_ptp_result_MSGTYPE beecam_uwb_i2c_downlink_ptp_result_t
#define beecam_uwb_i2c_downlink_t_response_info_MSGTYPE beecam_uwb_i2c_device_info_t
#define beecam_uwb_i2c_downlink_t_response_multi_ptp_normal_MSGTYPE beecam_uwb_i2c_downlink_normal_distribution_t
#define beecam_uwb_i2c_downlink_t_response_multi_ptp_full_MSGTYPE beecam_uwb_i2c_downlink_full_multi_ptp_result_t
#define beecam_uwb_i2c_downlink_t_response_success_MSGTYPE beecam_uwb_i2c_unit_t

#define BEECAM_UWB_I2C_DOWNLINK_ERROR_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   context,           1) \
X(a, STATIC,   SINGULAR, UENUM,    code,              2)
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_DEFAULT NULL

#define BEECAM_UWB_I2C_DOWNLINK_STATUS_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     ready,             1)
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_DEFAULT NULL

#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  peer_address,      2) \
X(a, STATIC,   SINGULAR, UINT64,   raw_tx_timestamp,   3) \
X(a, STATIC,   SINGULAR, UINT64,   raw_rx_timestamp,   4) \
X(a, STATIC,   SINGULAR, UINT64,   delay,             6) \
X(a, STATIC,   SINGULAR, INT32,    range_mm,          7)
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_DEFAULT NULL
#define beecam_uwb_i2c_downlink_ptp_result_t_peer_address_MSGTYPE beecam_uwb_i2c_peer_address_t

#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    mean,              1) \
X(a, STATIC,   SINGULAR, FLOAT,    stddev,            2) \
X(a, STATIC,   SINGULAR, UINT32,   n_samples,         3) \
X(a, STATIC,   SINGULAR, UINT64,   delay,             4)
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_DEFAULT NULL

#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_FIELDLIST(X, a) \
X(a, STATIC,   REPEATED, INT32,    ranges_mm,         1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  peer_address,      2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  normal_result,     3) \
X(a, STATIC,   REPEATED, UINT64,   ranges_dt,         4)
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_CALLBACK NULL
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_DEFAULT NULL
#define beecam_uwb_i2c_downlink_full_multi_ptp_result_t_peer_address_MSGTYPE beecam_uwb_i2c_peer_address_t
#define beecam_uwb_i2c_downlink_full_multi_ptp_result_t_normal_result_MSGTYPE beecam_uwb_i2c_downlink_normal_distribution_t

extern const pb_msgdesc_t beecam_uwb_i2c_unit_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_peer_address_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_device_info_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_device_info_uwb_info_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_device_info_bluetooth_info_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_device_info_firmware_build_info_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_uplink_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_uplink_multi_ptp_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_error_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_status_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_ptp_result_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_normal_distribution_t_msg;
extern const pb_msgdesc_t beecam_uwb_i2c_downlink_full_multi_ptp_result_t_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define BEECAM_UWB_I2C_UNIT_FIELDS &beecam_uwb_i2c_unit_t_msg
#define BEECAM_UWB_I2C_PEER_ADDRESS_FIELDS &beecam_uwb_i2c_peer_address_t_msg
#define BEECAM_UWB_I2C_DEVICE_INFO_FIELDS &beecam_uwb_i2c_device_info_t_msg
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_FIELDS &beecam_uwb_i2c_device_info_uwb_info_t_msg
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_FIELDS &beecam_uwb_i2c_device_info_bluetooth_info_t_msg
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_FIELDS &beecam_uwb_i2c_device_info_firmware_build_info_t_msg
#define BEECAM_UWB_I2C_UPLINK_FIELDS &beecam_uwb_i2c_uplink_t_msg
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_FIELDS &beecam_uwb_i2c_uplink_multi_ptp_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_FIELDS &beecam_uwb_i2c_downlink_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_FIELDS &beecam_uwb_i2c_downlink_error_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_FIELDS &beecam_uwb_i2c_downlink_status_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_FIELDS &beecam_uwb_i2c_downlink_ptp_result_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_FIELDS &beecam_uwb_i2c_downlink_normal_distribution_t_msg
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_FIELDS &beecam_uwb_i2c_downlink_full_multi_ptp_result_t_msg

/* Maximum encoded size of messages (where known) */
#define BEECAM_UWB_I2C_DEVICE_INFO_BLUETOOTH_INFO_SIZE 8
#define BEECAM_UWB_I2C_DEVICE_INFO_FIRMWARE_BUILD_INFO_SIZE 118
#define BEECAM_UWB_I2C_DEVICE_INFO_SIZE          148
#define BEECAM_UWB_I2C_DEVICE_INFO_UWB_INFO_SIZE 16
#define BEECAM_UWB_I2C_DOWNLINK_ERROR_SIZE       67
#define BEECAM_UWB_I2C_DOWNLINK_FULL_MULTI_PTP_RESULT_SIZE 2861
#define BEECAM_UWB_I2C_DOWNLINK_NORMAL_DISTRIBUTION_SIZE 27
#define BEECAM_UWB_I2C_DOWNLINK_PTP_RESULT_SIZE  60
#define BEECAM_UWB_I2C_DOWNLINK_SIZE             2936
#define BEECAM_UWB_I2C_DOWNLINK_STATUS_SIZE      2
#define BEECAM_UWB_I2C_PEER_ADDRESS_SIZE         14
#define BEECAM_UWB_I2C_UNIT_SIZE                 0
#define BEECAM_UWB_I2C_UPLINK_MULTI_PTP_SIZE     24
#define BEECAM_UWB_I2C_UPLINK_SIZE               92

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
