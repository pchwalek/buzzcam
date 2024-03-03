/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_MESSAGE_PB_H_INCLUDED
#define PB_MESSAGE_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* * defined by Bosch */
typedef enum signal_identifier {
    SIGNAL_IDENTIFIER_UNDEFINED = 0,
    SIGNAL_IDENTIFIER_IAQ = 1,
    SIGNAL_IDENTIFIER_STATIC_IAQ = 2,
    SIGNAL_IDENTIFIER_CO2_EQ = 3,
    SIGNAL_IDENTIFIER_BREATH_VOC_EQ = 4,
    SIGNAL_IDENTIFIER_RAW_TEMPERATURE = 6,
    SIGNAL_IDENTIFIER_RAW_PRESSURE = 7,
    SIGNAL_IDENTIFIER_RAW_HUMIDITY = 8,
    SIGNAL_IDENTIFIER_RAW_GAS = 9,
    SIGNAL_IDENTIFIER_STABILIZATION_STATUS = 12,
    SIGNAL_IDENTIFIER_RUN_IN_STATUS = 13,
    SIGNAL_IDENTIFIER_SENSOR_HEAT_COMPEN_TEMP = 14,
    SIGNAL_IDENTIFIER_HEAT_COMPEN_HUMID = 15,
    SIGNAL_IDENTIFIER_GAS_PERCENTAGE = 21
} signal_identifier_t;

/* * defined by Bosch */
typedef enum sensor_accuracy {
    SENSOR_ACCURACY_UNRELIABLE = 0,
    SENSOR_ACCURACY_LOW_ACCURACY = 1,
    SENSOR_ACCURACY_MEDIUM_ACCURACY = 2,
    SENSOR_ACCURACY_HIGH_ACCURACY = 3
} sensor_accuracy_t;

typedef enum mic_sample_freq {
    MIC_SAMPLE_FREQ_SAMPLE_RATE_8000 = 0,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_11025 = 1,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_16000 = 2,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_22500 = 3,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_24000 = 4,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_32000 = 5,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_44100 = 6,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_48000 = 7,
    MIC_SAMPLE_FREQ_SAMPLE_RATE_96000 = 8
} mic_sample_freq_t;

typedef enum mic_gain {
    MIC_GAIN_GAIN_60_DB = 0,
    MIC_GAIN_GAIN_57_DB = 1,
    MIC_GAIN_GAIN_54_DB = 2,
    MIC_GAIN_GAIN_51_DB = 3,
    MIC_GAIN_GAIN_48_DB = 4,
    MIC_GAIN_GAIN_45_DB = 5,
    MIC_GAIN_GAIN_42_DB = 6,
    MIC_GAIN_GAIN_39_DB = 7,
    MIC_GAIN_GAIN_36_DB = 8,
    MIC_GAIN_GAIN_33_DB = 9,
    MIC_GAIN_GAIN_30_DB = 10,
    MIC_GAIN_GAIN_27_DB = 11,
    MIC_GAIN_GAIN_24_DB = 12,
    MIC_GAIN_GAIN_21_DB = 13,
    MIC_GAIN_GAIN_18_DB = 14,
    MIC_GAIN_GAIN_15_DB = 15,
    MIC_GAIN_GAIN_12_DB = 16,
    MIC_GAIN_GAIN_9_DB = 17,
    MIC_GAIN_GAIN_6_DB = 18,
    MIC_GAIN_GAIN_3_DB = 19,
    MIC_GAIN_GAIN_0_DB = 20,
    MIC_GAIN_GAIN_NEG_3_DB = 21,
    MIC_GAIN_GAIN_NEG_6_DB = 22,
    MIC_GAIN_GAIN_NEG_9_DB = 23,
    MIC_GAIN_GAIN_NEG_12_DB = 24,
    MIC_GAIN_GAIN_NEG_15_DB = 25
} mic_gain_t;

typedef enum mic_bit_resolution {
    MIC_BIT_RESOLUTION_BIT_RES_8 = 0,
    MIC_BIT_RESOLUTION_BIT_RES_16 = 1,
    MIC_BIT_RESOLUTION_BIT_RES_24 = 2
} mic_bit_resolution_t;

typedef enum compression_type {
    COMPRESSION_TYPE_OPUS = 0,
    COMPRESSION_TYPE_FLAC = 1
} compression_type_t;

/* Struct definitions */
typedef struct packet_header {
    uint32_t system_uid;
    uint32_t ms_from_start;
    uint64_t epoch;
} packet_header_t;

typedef struct simple_sensor_reading {
    uint32_t index;
    uint32_t timestamp_unix;
    float temperature;
    float humidity;
    float co2;
    float light_level;
} simple_sensor_reading_t;

typedef struct sensor_reading {
    uint32_t packet_index;
    uint32_t sample_period;
    pb_callback_t payload;
} sensor_reading_t;

typedef struct sensor_reading_payload {
    uint64_t timestamp_sensor;
    uint64_t timestamp_unix;
    uint32_t timestamp_ms_from_start;
    float signal;
    uint32_t signal_dimensions;
    signal_identifier_t sensor_id;
    sensor_accuracy_t accuracy;
} sensor_reading_payload_t;

typedef struct sensor_config {
    bool enable_temperature;
    bool enable_humidity;
    bool enable_gas;
} sensor_config_t;

typedef struct sd_card_state {
    bool detected;
    uint64_t space_remaining;
    uint64_t estimated_remaining_recording_time;
} sd_card_state_t;

typedef struct mark_state {
    /* bool beep_enabled = 1; */
    uint32_t mark_number;
    uint64_t timestamp_unix;
} mark_state_t;

typedef struct mark_packet {
    /* optional string annotation = 1 [(nanopb).max_length = 50];// throttle max character count (e.g., 50) */
    bool has_annotation;
    char annotation[50]; /* throttle max character count (e.g., 50) */
    bool beep_enabled;
} mark_packet_t;

typedef struct battery_state {
    bool charging;
    float voltage;
    bool has_percentage;
    float percentage;
} battery_state_t;

typedef struct device {
    uint32_t uid;
    float range;
} device_t;

typedef struct system_info_packet {
    bool has_simple_sensor_reading;
    simple_sensor_reading_t simple_sensor_reading;
    bool device_recording;
    bool has_sdcard_state;
    sd_card_state_t sdcard_state;
    bool has_mark_state;
    mark_state_t mark_state;
    bool has_battery_state;
    battery_state_t battery_state;
    pb_size_t discovered_devices_count;
    device_t discovered_devices[20];
} system_info_packet_t;

typedef struct audio_compression {
    bool enabled;
    compression_type_t compression_type;
    uint32_t compression_factor;
} audio_compression_t;

typedef struct audio_config {
    bool channel_1;
    bool channel_2;
    mic_sample_freq_t sample_freq;
    mic_gain_t mic_gain;
    mic_bit_resolution_t bit_resolution;
    bool has_audio_compression;
    audio_compression_t audio_compression;
    float estimated_record_time;
    bool free_run_mode;
    bool chirp_enable;
} audio_config_t;

typedef struct schedule_config {
    bool sunday;
    bool monday;
    bool tuesday;
    bool wednesday;
    bool thursday;
    bool friday;
    bool saturday;
    uint32_t start_hour;
    uint32_t start_minute;
    uint32_t stop_hour;
    uint32_t stop_minute;
} schedule_config_t;

typedef struct low_power_config {
    bool low_power_mode;
} low_power_config_t;

typedef struct camera_control {
    bool pair_with_nearby_cameras;
    bool wakeup_cameras;
    bool capture;
} camera_control_t;

typedef struct device_uid {
    char addr[10];
} device_uid_t;

typedef struct network_state {
    uint32_t number_of_discovered_devices;
    pb_size_t discovered_device_uid_count;
    device_uid_t discovered_device_uid[10];
    uint32_t channel;
    uint32_t pan_id;
    bool slave_sync;
    bool master_node;
} network_state_t;

typedef struct config_packet {
    bool has_audio_config;
    audio_config_t audio_config;
    pb_size_t schedule_config_count;
    schedule_config_t schedule_config[10];
    bool has_sensor_config;
    sensor_config_t sensor_config;
    bool has_low_power_config;
    low_power_config_t low_power_config;
    bool has_network_state;
    network_state_t network_state;
    bool enable_recording;
} config_packet_t;

typedef PB_BYTES_ARRAY_T(8) peer_address_address_t;
typedef struct peer_address {
    /* Encoded as u16. */
    uint16_t pan_id;
    /* 2 or 8 bytes (short / extended). */
    peer_address_address_t address;
} peer_address_t;

typedef struct uwb_range {
    bool has_openthread_uid;
    device_uid_t openthread_uid;
    uint32_t system_uid;
    bool has_uwb_addr;
    peer_address_t uwb_addr;
    uint32_t range;
    float std_dev;
} uwb_range_t;

typedef struct uwb_info {
    bool has_openthread_uid;
    device_uid_t openthread_uid;
    uint32_t system_uid;
    bool has_uwb_addr;
    peer_address_t uwb_addr;
} uwb_info_t;

typedef struct uwb_packet {
    bool start_ranging;
    bool turn_on_uwb;
    pb_size_t ranges_count;
    uwb_range_t ranges[20];
} uwb_packet_t;

typedef struct special_function {
    pb_size_t which_payload;
    union {
        bool format_sdcard;
        camera_control_t camera_control;
        uwb_packet_t uwb_packet;
        bool openthread_sync_time;
        bool mag_calibration;
        bool slave_req_config;
        uint32_t timestamp;
        bool dfu_mode;
        uwb_info_t uwb_info;
    } payload;
} special_function_t;

typedef struct packet {
    bool has_header;
    packet_header_t header;
    pb_size_t which_payload;
    union {
        system_info_packet_t system_info_packet;
        mark_packet_t mark_packet;
        config_packet_t config_packet;
        special_function_t special_function;
    } payload;
} packet_t;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _SIGNAL_IDENTIFIER_MIN SIGNAL_IDENTIFIER_UNDEFINED
#define _SIGNAL_IDENTIFIER_MAX SIGNAL_IDENTIFIER_GAS_PERCENTAGE
#define _SIGNAL_IDENTIFIER_ARRAYSIZE ((signal_identifier_t)(SIGNAL_IDENTIFIER_GAS_PERCENTAGE+1))

#define _SENSOR_ACCURACY_MIN SENSOR_ACCURACY_UNRELIABLE
#define _SENSOR_ACCURACY_MAX SENSOR_ACCURACY_HIGH_ACCURACY
#define _SENSOR_ACCURACY_ARRAYSIZE ((sensor_accuracy_t)(SENSOR_ACCURACY_HIGH_ACCURACY+1))

#define _MIC_SAMPLE_FREQ_MIN MIC_SAMPLE_FREQ_SAMPLE_RATE_8000
#define _MIC_SAMPLE_FREQ_MAX MIC_SAMPLE_FREQ_SAMPLE_RATE_96000
#define _MIC_SAMPLE_FREQ_ARRAYSIZE ((mic_sample_freq_t)(MIC_SAMPLE_FREQ_SAMPLE_RATE_96000+1))

#define _MIC_GAIN_MIN MIC_GAIN_GAIN_60_DB
#define _MIC_GAIN_MAX MIC_GAIN_GAIN_NEG_15_DB
#define _MIC_GAIN_ARRAYSIZE ((mic_gain_t)(MIC_GAIN_GAIN_NEG_15_DB+1))

#define _MIC_BIT_RESOLUTION_MIN MIC_BIT_RESOLUTION_BIT_RES_8
#define _MIC_BIT_RESOLUTION_MAX MIC_BIT_RESOLUTION_BIT_RES_24
#define _MIC_BIT_RESOLUTION_ARRAYSIZE ((mic_bit_resolution_t)(MIC_BIT_RESOLUTION_BIT_RES_24+1))

#define _COMPRESSION_TYPE_MIN COMPRESSION_TYPE_OPUS
#define _COMPRESSION_TYPE_MAX COMPRESSION_TYPE_FLAC
#define _COMPRESSION_TYPE_ARRAYSIZE ((compression_type_t)(COMPRESSION_TYPE_FLAC+1))




#define sensor_reading_payload_t_sensor_id_ENUMTYPE signal_identifier_t
#define sensor_reading_payload_t_accuracy_ENUMTYPE sensor_accuracy_t








#define audio_compression_t_compression_type_ENUMTYPE compression_type_t

#define audio_config_t_sample_freq_ENUMTYPE mic_sample_freq_t
#define audio_config_t_mic_gain_ENUMTYPE mic_gain_t
#define audio_config_t_bit_resolution_ENUMTYPE mic_bit_resolution_t














/* Initializer values for message structs */
#define PACKET_HEADER_INIT_DEFAULT               {0, 0, 0}
#define SIMPLE_SENSOR_READING_INIT_DEFAULT       {0, 0, 0, 0, 0, 0}
#define SENSOR_READING_INIT_DEFAULT              {0, 0, {{NULL}, NULL}}
#define SENSOR_READING_PAYLOAD_INIT_DEFAULT      {0, 0, 0, 0, 0, _SIGNAL_IDENTIFIER_MIN, _SENSOR_ACCURACY_MIN}
#define SENSOR_CONFIG_INIT_DEFAULT               {0, 0, 0}
#define SD_CARD_STATE_INIT_DEFAULT               {0, 0, 0}
#define MARK_STATE_INIT_DEFAULT                  {0, 0}
#define MARK_PACKET_INIT_DEFAULT                 {false, "", 0}
#define BATTERY_STATE_INIT_DEFAULT               {0, 0, false, 0}
#define DEVICE_INIT_DEFAULT                      {0, 0}
#define SYSTEM_INFO_PACKET_INIT_DEFAULT          {false, SIMPLE_SENSOR_READING_INIT_DEFAULT, 0, false, SD_CARD_STATE_INIT_DEFAULT, false, MARK_STATE_INIT_DEFAULT, false, BATTERY_STATE_INIT_DEFAULT, 0, {DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT, DEVICE_INIT_DEFAULT}}
#define AUDIO_COMPRESSION_INIT_DEFAULT           {0, _COMPRESSION_TYPE_MIN, 0}
#define AUDIO_CONFIG_INIT_DEFAULT                {0, 0, _MIC_SAMPLE_FREQ_MIN, _MIC_GAIN_MIN, _MIC_BIT_RESOLUTION_MIN, false, AUDIO_COMPRESSION_INIT_DEFAULT, 0, 0, 0}
#define SCHEDULE_CONFIG_INIT_DEFAULT             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define LOW_POWER_CONFIG_INIT_DEFAULT            {0}
#define CAMERA_CONTROL_INIT_DEFAULT              {0, 0, 0}
#define DEVICE_UID_INIT_DEFAULT                  {""}
#define NETWORK_STATE_INIT_DEFAULT               {0, 0, {DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT, DEVICE_UID_INIT_DEFAULT}, 0, 0, 0, 0}
#define CONFIG_PACKET_INIT_DEFAULT               {false, AUDIO_CONFIG_INIT_DEFAULT, 0, {SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT, SCHEDULE_CONFIG_INIT_DEFAULT}, false, SENSOR_CONFIG_INIT_DEFAULT, false, LOW_POWER_CONFIG_INIT_DEFAULT, false, NETWORK_STATE_INIT_DEFAULT, 0}
#define PEER_ADDRESS_INIT_DEFAULT                {0, {0, {0}}}
#define UWB_RANGE_INIT_DEFAULT                   {false, DEVICE_UID_INIT_DEFAULT, 0, false, PEER_ADDRESS_INIT_DEFAULT, 0, 0}
#define UWB_INFO_INIT_DEFAULT                    {false, DEVICE_UID_INIT_DEFAULT, 0, false, PEER_ADDRESS_INIT_DEFAULT}
#define UWB_PACKET_INIT_DEFAULT                  {0, 0, 0, {UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT, UWB_RANGE_INIT_DEFAULT}}
#define SPECIAL_FUNCTION_INIT_DEFAULT            {0, {0}}
#define PACKET_INIT_DEFAULT                      {false, PACKET_HEADER_INIT_DEFAULT, 0, {SYSTEM_INFO_PACKET_INIT_DEFAULT}}
#define PACKET_HEADER_INIT_ZERO                  {0, 0, 0}
#define SIMPLE_SENSOR_READING_INIT_ZERO          {0, 0, 0, 0, 0, 0}
#define SENSOR_READING_INIT_ZERO                 {0, 0, {{NULL}, NULL}}
#define SENSOR_READING_PAYLOAD_INIT_ZERO         {0, 0, 0, 0, 0, _SIGNAL_IDENTIFIER_MIN, _SENSOR_ACCURACY_MIN}
#define SENSOR_CONFIG_INIT_ZERO                  {0, 0, 0}
#define SD_CARD_STATE_INIT_ZERO                  {0, 0, 0}
#define MARK_STATE_INIT_ZERO                     {0, 0}
#define MARK_PACKET_INIT_ZERO                    {false, "", 0}
#define BATTERY_STATE_INIT_ZERO                  {0, 0, false, 0}
#define DEVICE_INIT_ZERO                         {0, 0}
#define SYSTEM_INFO_PACKET_INIT_ZERO             {false, SIMPLE_SENSOR_READING_INIT_ZERO, 0, false, SD_CARD_STATE_INIT_ZERO, false, MARK_STATE_INIT_ZERO, false, BATTERY_STATE_INIT_ZERO, 0, {DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO, DEVICE_INIT_ZERO}}
#define AUDIO_COMPRESSION_INIT_ZERO              {0, _COMPRESSION_TYPE_MIN, 0}
#define AUDIO_CONFIG_INIT_ZERO                   {0, 0, _MIC_SAMPLE_FREQ_MIN, _MIC_GAIN_MIN, _MIC_BIT_RESOLUTION_MIN, false, AUDIO_COMPRESSION_INIT_ZERO, 0, 0, 0}
#define SCHEDULE_CONFIG_INIT_ZERO                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define LOW_POWER_CONFIG_INIT_ZERO               {0}
#define CAMERA_CONTROL_INIT_ZERO                 {0, 0, 0}
#define DEVICE_UID_INIT_ZERO                     {""}
#define NETWORK_STATE_INIT_ZERO                  {0, 0, {DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO, DEVICE_UID_INIT_ZERO}, 0, 0, 0, 0}
#define CONFIG_PACKET_INIT_ZERO                  {false, AUDIO_CONFIG_INIT_ZERO, 0, {SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO, SCHEDULE_CONFIG_INIT_ZERO}, false, SENSOR_CONFIG_INIT_ZERO, false, LOW_POWER_CONFIG_INIT_ZERO, false, NETWORK_STATE_INIT_ZERO, 0}
#define PEER_ADDRESS_INIT_ZERO                   {0, {0, {0}}}
#define UWB_RANGE_INIT_ZERO                      {false, DEVICE_UID_INIT_ZERO, 0, false, PEER_ADDRESS_INIT_ZERO, 0, 0}
#define UWB_INFO_INIT_ZERO                       {false, DEVICE_UID_INIT_ZERO, 0, false, PEER_ADDRESS_INIT_ZERO}
#define UWB_PACKET_INIT_ZERO                     {0, 0, 0, {UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO, UWB_RANGE_INIT_ZERO}}
#define SPECIAL_FUNCTION_INIT_ZERO               {0, {0}}
#define PACKET_INIT_ZERO                         {false, PACKET_HEADER_INIT_ZERO, 0, {SYSTEM_INFO_PACKET_INIT_ZERO}}

/* Field tags (for use in manual encoding/decoding) */
#define PACKET_HEADER_SYSTEM_UID_TAG             1
#define PACKET_HEADER_MS_FROM_START_TAG          2
#define PACKET_HEADER_EPOCH_TAG                  3
#define SIMPLE_SENSOR_READING_INDEX_TAG          1
#define SIMPLE_SENSOR_READING_TIMESTAMP_UNIX_TAG 2
#define SIMPLE_SENSOR_READING_TEMPERATURE_TAG    3
#define SIMPLE_SENSOR_READING_HUMIDITY_TAG       4
#define SIMPLE_SENSOR_READING_CO2_TAG            5
#define SIMPLE_SENSOR_READING_LIGHT_LEVEL_TAG    6
#define SENSOR_READING_PACKET_INDEX_TAG          1
#define SENSOR_READING_SAMPLE_PERIOD_TAG         2
#define SENSOR_READING_PAYLOAD_TAG               3
#define SENSOR_READING_PAYLOAD_TIMESTAMP_SENSOR_TAG 1
#define SENSOR_READING_PAYLOAD_TIMESTAMP_UNIX_TAG 2
#define SENSOR_READING_PAYLOAD_TIMESTAMP_MS_FROM_START_TAG 3
#define SENSOR_READING_PAYLOAD_SIGNAL_TAG        4
#define SENSOR_READING_PAYLOAD_SIGNAL_DIMENSIONS_TAG 5
#define SENSOR_READING_PAYLOAD_SENSOR_ID_TAG     6
#define SENSOR_READING_PAYLOAD_ACCURACY_TAG      7
#define SENSOR_CONFIG_ENABLE_TEMPERATURE_TAG     1
#define SENSOR_CONFIG_ENABLE_HUMIDITY_TAG        2
#define SENSOR_CONFIG_ENABLE_GAS_TAG             3
#define SD_CARD_STATE_DETECTED_TAG               1
#define SD_CARD_STATE_SPACE_REMAINING_TAG        2
#define SD_CARD_STATE_ESTIMATED_REMAINING_RECORDING_TIME_TAG 3
#define MARK_STATE_MARK_NUMBER_TAG               1
#define MARK_STATE_TIMESTAMP_UNIX_TAG            2
#define MARK_PACKET_ANNOTATION_TAG               1
#define MARK_PACKET_BEEP_ENABLED_TAG             2
#define BATTERY_STATE_CHARGING_TAG               1
#define BATTERY_STATE_VOLTAGE_TAG                2
#define BATTERY_STATE_PERCENTAGE_TAG             3
#define DEVICE_UID_TAG                           1
#define DEVICE_RANGE_TAG                         2
#define SYSTEM_INFO_PACKET_SIMPLE_SENSOR_READING_TAG 1
#define SYSTEM_INFO_PACKET_DEVICE_RECORDING_TAG  2
#define SYSTEM_INFO_PACKET_SDCARD_STATE_TAG      3
#define SYSTEM_INFO_PACKET_MARK_STATE_TAG        4
#define SYSTEM_INFO_PACKET_BATTERY_STATE_TAG     5
#define SYSTEM_INFO_PACKET_DISCOVERED_DEVICES_TAG 6
#define AUDIO_COMPRESSION_ENABLED_TAG            1
#define AUDIO_COMPRESSION_COMPRESSION_TYPE_TAG   2
#define AUDIO_COMPRESSION_COMPRESSION_FACTOR_TAG 3
#define AUDIO_CONFIG_CHANNEL_1_TAG               1
#define AUDIO_CONFIG_CHANNEL_2_TAG               2
#define AUDIO_CONFIG_SAMPLE_FREQ_TAG             3
#define AUDIO_CONFIG_MIC_GAIN_TAG                4
#define AUDIO_CONFIG_BIT_RESOLUTION_TAG          5
#define AUDIO_CONFIG_AUDIO_COMPRESSION_TAG       6
#define AUDIO_CONFIG_ESTIMATED_RECORD_TIME_TAG   7
#define AUDIO_CONFIG_FREE_RUN_MODE_TAG           8
#define AUDIO_CONFIG_CHIRP_ENABLE_TAG            9
#define SCHEDULE_CONFIG_SUNDAY_TAG               1
#define SCHEDULE_CONFIG_MONDAY_TAG               2
#define SCHEDULE_CONFIG_TUESDAY_TAG              3
#define SCHEDULE_CONFIG_WEDNESDAY_TAG            4
#define SCHEDULE_CONFIG_THURSDAY_TAG             5
#define SCHEDULE_CONFIG_FRIDAY_TAG               6
#define SCHEDULE_CONFIG_SATURDAY_TAG             7
#define SCHEDULE_CONFIG_START_HOUR_TAG           8
#define SCHEDULE_CONFIG_START_MINUTE_TAG         9
#define SCHEDULE_CONFIG_STOP_HOUR_TAG            10
#define SCHEDULE_CONFIG_STOP_MINUTE_TAG          11
#define LOW_POWER_CONFIG_LOW_POWER_MODE_TAG      1
#define CAMERA_CONTROL_PAIR_WITH_NEARBY_CAMERAS_TAG 1
#define CAMERA_CONTROL_WAKEUP_CAMERAS_TAG        2
#define CAMERA_CONTROL_CAPTURE_TAG               3
#define DEVICE_UID_ADDR_TAG                      1
#define NETWORK_STATE_NUMBER_OF_DISCOVERED_DEVICES_TAG 1
#define NETWORK_STATE_DISCOVERED_DEVICE_UID_TAG  2
#define NETWORK_STATE_CHANNEL_TAG                3
#define NETWORK_STATE_PAN_ID_TAG                 4
#define NETWORK_STATE_SLAVE_SYNC_TAG             5
#define NETWORK_STATE_MASTER_NODE_TAG            6
#define CONFIG_PACKET_AUDIO_CONFIG_TAG           1
#define CONFIG_PACKET_SCHEDULE_CONFIG_TAG        2
#define CONFIG_PACKET_SENSOR_CONFIG_TAG          3
#define CONFIG_PACKET_LOW_POWER_CONFIG_TAG       4
#define CONFIG_PACKET_NETWORK_STATE_TAG          5
#define CONFIG_PACKET_ENABLE_RECORDING_TAG       6
#define PEER_ADDRESS_PAN_ID_TAG                  1
#define PEER_ADDRESS_ADDRESS_TAG                 2
#define UWB_RANGE_OPENTHREAD_UID_TAG             1
#define UWB_RANGE_SYSTEM_UID_TAG                 2
#define UWB_RANGE_UWB_ADDR_TAG                   3
#define UWB_RANGE_RANGE_TAG                      4
#define UWB_RANGE_STD_DEV_TAG                    5
#define UWB_INFO_OPENTHREAD_UID_TAG              1
#define UWB_INFO_SYSTEM_UID_TAG                  2
#define UWB_INFO_UWB_ADDR_TAG                    3
#define UWB_PACKET_START_RANGING_TAG             1
#define UWB_PACKET_TURN_ON_UWB_TAG               2
#define UWB_PACKET_RANGES_TAG                    3
#define SPECIAL_FUNCTION_FORMAT_SDCARD_TAG       1
#define SPECIAL_FUNCTION_CAMERA_CONTROL_TAG      2
#define SPECIAL_FUNCTION_UWB_PACKET_TAG          3
#define SPECIAL_FUNCTION_OPENTHREAD_SYNC_TIME_TAG 4
#define SPECIAL_FUNCTION_MAG_CALIBRATION_TAG     5
#define SPECIAL_FUNCTION_SLAVE_REQ_CONFIG_TAG    6
#define SPECIAL_FUNCTION_TIMESTAMP_TAG           7
#define SPECIAL_FUNCTION_DFU_MODE_TAG            8
#define SPECIAL_FUNCTION_UWB_INFO_TAG            9
#define PACKET_HEADER_TAG                        1
#define PACKET_SYSTEM_INFO_PACKET_TAG            2
#define PACKET_MARK_PACKET_TAG                   3
#define PACKET_CONFIG_PACKET_TAG                 4
#define PACKET_SPECIAL_FUNCTION_TAG              5

/* Struct field encoding specification for nanopb */
#define PACKET_HEADER_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   system_uid,        1) \
X(a, STATIC,   SINGULAR, UINT32,   ms_from_start,     2) \
X(a, STATIC,   SINGULAR, UINT64,   epoch,             3)
#define PACKET_HEADER_CALLBACK NULL
#define PACKET_HEADER_DEFAULT NULL

#define SIMPLE_SENSOR_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   index,             1) \
X(a, STATIC,   SINGULAR, UINT32,   timestamp_unix,    2) \
X(a, STATIC,   SINGULAR, FLOAT,    temperature,       3) \
X(a, STATIC,   SINGULAR, FLOAT,    humidity,          4) \
X(a, STATIC,   SINGULAR, FLOAT,    co2,               5) \
X(a, STATIC,   SINGULAR, FLOAT,    light_level,       6)
#define SIMPLE_SENSOR_READING_CALLBACK NULL
#define SIMPLE_SENSOR_READING_DEFAULT NULL

#define SENSOR_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   packet_index,      1) \
X(a, STATIC,   SINGULAR, UINT32,   sample_period,     2) \
X(a, CALLBACK, REPEATED, MESSAGE,  payload,           3)
#define SENSOR_READING_CALLBACK pb_default_field_callback
#define SENSOR_READING_DEFAULT NULL
#define sensor_reading_t_payload_MSGTYPE sensor_reading_payload_t

#define SENSOR_READING_PAYLOAD_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FIXED64,  timestamp_sensor,   1) \
X(a, STATIC,   SINGULAR, UINT64,   timestamp_unix,    2) \
X(a, STATIC,   SINGULAR, UINT32,   timestamp_ms_from_start,   3) \
X(a, STATIC,   SINGULAR, FLOAT,    signal,            4) \
X(a, STATIC,   SINGULAR, UINT32,   signal_dimensions,   5) \
X(a, STATIC,   SINGULAR, UENUM,    sensor_id,         6) \
X(a, STATIC,   SINGULAR, UENUM,    accuracy,          7)
#define SENSOR_READING_PAYLOAD_CALLBACK NULL
#define SENSOR_READING_PAYLOAD_DEFAULT NULL

#define SENSOR_CONFIG_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     enable_temperature,   1) \
X(a, STATIC,   SINGULAR, BOOL,     enable_humidity,   2) \
X(a, STATIC,   SINGULAR, BOOL,     enable_gas,        3)
#define SENSOR_CONFIG_CALLBACK NULL
#define SENSOR_CONFIG_DEFAULT NULL

#define SD_CARD_STATE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     detected,          1) \
X(a, STATIC,   SINGULAR, UINT64,   space_remaining,   2) \
X(a, STATIC,   SINGULAR, UINT64,   estimated_remaining_recording_time,   3)
#define SD_CARD_STATE_CALLBACK NULL
#define SD_CARD_STATE_DEFAULT NULL

#define MARK_STATE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   mark_number,       1) \
X(a, STATIC,   SINGULAR, UINT64,   timestamp_unix,    2)
#define MARK_STATE_CALLBACK NULL
#define MARK_STATE_DEFAULT NULL

#define MARK_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, STRING,   annotation,        1) \
X(a, STATIC,   SINGULAR, BOOL,     beep_enabled,      2)
#define MARK_PACKET_CALLBACK NULL
#define MARK_PACKET_DEFAULT NULL

#define BATTERY_STATE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     charging,          1) \
X(a, STATIC,   SINGULAR, FLOAT,    voltage,           2) \
X(a, STATIC,   OPTIONAL, FLOAT,    percentage,        3)
#define BATTERY_STATE_CALLBACK NULL
#define BATTERY_STATE_DEFAULT NULL

#define DEVICE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   uid,               1) \
X(a, STATIC,   SINGULAR, FLOAT,    range,             2)
#define DEVICE_CALLBACK NULL
#define DEVICE_DEFAULT NULL

#define SYSTEM_INFO_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  simple_sensor_reading,   1) \
X(a, STATIC,   SINGULAR, BOOL,     device_recording,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  sdcard_state,      3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  mark_state,        4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  battery_state,     5) \
X(a, STATIC,   REPEATED, MESSAGE,  discovered_devices,   6)
#define SYSTEM_INFO_PACKET_CALLBACK NULL
#define SYSTEM_INFO_PACKET_DEFAULT NULL
#define system_info_packet_t_simple_sensor_reading_MSGTYPE simple_sensor_reading_t
#define system_info_packet_t_sdcard_state_MSGTYPE sd_card_state_t
#define system_info_packet_t_mark_state_MSGTYPE mark_state_t
#define system_info_packet_t_battery_state_MSGTYPE battery_state_t
#define system_info_packet_t_discovered_devices_MSGTYPE device_t

#define AUDIO_COMPRESSION_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     enabled,           1) \
X(a, STATIC,   SINGULAR, UENUM,    compression_type,   2) \
X(a, STATIC,   SINGULAR, UINT32,   compression_factor,   3)
#define AUDIO_COMPRESSION_CALLBACK NULL
#define AUDIO_COMPRESSION_DEFAULT NULL

#define AUDIO_CONFIG_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     channel_1,         1) \
X(a, STATIC,   SINGULAR, BOOL,     channel_2,         2) \
X(a, STATIC,   SINGULAR, UENUM,    sample_freq,       3) \
X(a, STATIC,   SINGULAR, UENUM,    mic_gain,          4) \
X(a, STATIC,   SINGULAR, UENUM,    bit_resolution,    5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  audio_compression,   6) \
X(a, STATIC,   SINGULAR, FLOAT,    estimated_record_time,   7) \
X(a, STATIC,   SINGULAR, BOOL,     free_run_mode,     8) \
X(a, STATIC,   SINGULAR, BOOL,     chirp_enable,      9)
#define AUDIO_CONFIG_CALLBACK NULL
#define AUDIO_CONFIG_DEFAULT NULL
#define audio_config_t_audio_compression_MSGTYPE audio_compression_t

#define SCHEDULE_CONFIG_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     sunday,            1) \
X(a, STATIC,   SINGULAR, BOOL,     monday,            2) \
X(a, STATIC,   SINGULAR, BOOL,     tuesday,           3) \
X(a, STATIC,   SINGULAR, BOOL,     wednesday,         4) \
X(a, STATIC,   SINGULAR, BOOL,     thursday,          5) \
X(a, STATIC,   SINGULAR, BOOL,     friday,            6) \
X(a, STATIC,   SINGULAR, BOOL,     saturday,          7) \
X(a, STATIC,   SINGULAR, UINT32,   start_hour,        8) \
X(a, STATIC,   SINGULAR, UINT32,   start_minute,      9) \
X(a, STATIC,   SINGULAR, UINT32,   stop_hour,        10) \
X(a, STATIC,   SINGULAR, UINT32,   stop_minute,      11)
#define SCHEDULE_CONFIG_CALLBACK NULL
#define SCHEDULE_CONFIG_DEFAULT NULL

#define LOW_POWER_CONFIG_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     low_power_mode,    1)
#define LOW_POWER_CONFIG_CALLBACK NULL
#define LOW_POWER_CONFIG_DEFAULT NULL

#define CAMERA_CONTROL_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     pair_with_nearby_cameras,   1) \
X(a, STATIC,   SINGULAR, BOOL,     wakeup_cameras,    2) \
X(a, STATIC,   SINGULAR, BOOL,     capture,           3)
#define CAMERA_CONTROL_CALLBACK NULL
#define CAMERA_CONTROL_DEFAULT NULL

#define DEVICE_UID_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   addr,              1)
#define DEVICE_UID_CALLBACK NULL
#define DEVICE_UID_DEFAULT NULL

#define NETWORK_STATE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   number_of_discovered_devices,   1) \
X(a, STATIC,   REPEATED, MESSAGE,  discovered_device_uid,   2) \
X(a, STATIC,   SINGULAR, UINT32,   channel,           3) \
X(a, STATIC,   SINGULAR, UINT32,   pan_id,            4) \
X(a, STATIC,   SINGULAR, BOOL,     slave_sync,        5) \
X(a, STATIC,   SINGULAR, BOOL,     master_node,       6)
#define NETWORK_STATE_CALLBACK NULL
#define NETWORK_STATE_DEFAULT NULL
#define network_state_t_discovered_device_uid_MSGTYPE device_uid_t

#define CONFIG_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  audio_config,      1) \
X(a, STATIC,   REPEATED, MESSAGE,  schedule_config,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  sensor_config,     3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  low_power_config,   4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  network_state,     5) \
X(a, STATIC,   SINGULAR, BOOL,     enable_recording,   6)
#define CONFIG_PACKET_CALLBACK NULL
#define CONFIG_PACKET_DEFAULT NULL
#define config_packet_t_audio_config_MSGTYPE audio_config_t
#define config_packet_t_schedule_config_MSGTYPE schedule_config_t
#define config_packet_t_sensor_config_MSGTYPE sensor_config_t
#define config_packet_t_low_power_config_MSGTYPE low_power_config_t
#define config_packet_t_network_state_MSGTYPE network_state_t

#define PEER_ADDRESS_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   pan_id,            1) \
X(a, STATIC,   SINGULAR, BYTES,    address,           2)
#define PEER_ADDRESS_CALLBACK NULL
#define PEER_ADDRESS_DEFAULT NULL

#define UWB_RANGE_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  openthread_uid,    1) \
X(a, STATIC,   SINGULAR, UINT32,   system_uid,        2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  uwb_addr,          3) \
X(a, STATIC,   SINGULAR, UINT32,   range,             4) \
X(a, STATIC,   SINGULAR, FLOAT,    std_dev,           5)
#define UWB_RANGE_CALLBACK NULL
#define UWB_RANGE_DEFAULT NULL
#define uwb_range_t_openthread_uid_MSGTYPE device_uid_t
#define uwb_range_t_uwb_addr_MSGTYPE peer_address_t

#define UWB_INFO_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  openthread_uid,    1) \
X(a, STATIC,   SINGULAR, UINT32,   system_uid,        2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  uwb_addr,          3)
#define UWB_INFO_CALLBACK NULL
#define UWB_INFO_DEFAULT NULL
#define uwb_info_t_openthread_uid_MSGTYPE device_uid_t
#define uwb_info_t_uwb_addr_MSGTYPE peer_address_t

#define UWB_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     start_ranging,     1) \
X(a, STATIC,   SINGULAR, BOOL,     turn_on_uwb,       2) \
X(a, STATIC,   REPEATED, MESSAGE,  ranges,            3)
#define UWB_PACKET_CALLBACK NULL
#define UWB_PACKET_DEFAULT NULL
#define uwb_packet_t_ranges_MSGTYPE uwb_range_t

#define SPECIAL_FUNCTION_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    BOOL,     (payload,format_sdcard,payload.format_sdcard),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,camera_control,payload.camera_control),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,uwb_packet,payload.uwb_packet),   3) \
X(a, STATIC,   ONEOF,    BOOL,     (payload,openthread_sync_time,payload.openthread_sync_time),   4) \
X(a, STATIC,   ONEOF,    BOOL,     (payload,mag_calibration,payload.mag_calibration),   5) \
X(a, STATIC,   ONEOF,    BOOL,     (payload,slave_req_config,payload.slave_req_config),   6) \
X(a, STATIC,   ONEOF,    UINT32,   (payload,timestamp,payload.timestamp),   7) \
X(a, STATIC,   ONEOF,    BOOL,     (payload,dfu_mode,payload.dfu_mode),   8) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,uwb_info,payload.uwb_info),   9)
#define SPECIAL_FUNCTION_CALLBACK NULL
#define SPECIAL_FUNCTION_DEFAULT NULL
#define special_function_t_payload_camera_control_MSGTYPE camera_control_t
#define special_function_t_payload_uwb_packet_MSGTYPE uwb_packet_t
#define special_function_t_payload_uwb_info_MSGTYPE uwb_info_t

#define PACKET_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  header,            1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,system_info_packet,payload.system_info_packet),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,mark_packet,payload.mark_packet),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,config_packet,payload.config_packet),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,special_function,payload.special_function),   5)
#define PACKET_CALLBACK NULL
#define PACKET_DEFAULT NULL
#define packet_t_header_MSGTYPE packet_header_t
#define packet_t_payload_system_info_packet_MSGTYPE system_info_packet_t
#define packet_t_payload_mark_packet_MSGTYPE mark_packet_t
#define packet_t_payload_config_packet_MSGTYPE config_packet_t
#define packet_t_payload_special_function_MSGTYPE special_function_t

extern const pb_msgdesc_t packet_header_t_msg;
extern const pb_msgdesc_t simple_sensor_reading_t_msg;
extern const pb_msgdesc_t sensor_reading_t_msg;
extern const pb_msgdesc_t sensor_reading_payload_t_msg;
extern const pb_msgdesc_t sensor_config_t_msg;
extern const pb_msgdesc_t sd_card_state_t_msg;
extern const pb_msgdesc_t mark_state_t_msg;
extern const pb_msgdesc_t mark_packet_t_msg;
extern const pb_msgdesc_t battery_state_t_msg;
extern const pb_msgdesc_t device_t_msg;
extern const pb_msgdesc_t system_info_packet_t_msg;
extern const pb_msgdesc_t audio_compression_t_msg;
extern const pb_msgdesc_t audio_config_t_msg;
extern const pb_msgdesc_t schedule_config_t_msg;
extern const pb_msgdesc_t low_power_config_t_msg;
extern const pb_msgdesc_t camera_control_t_msg;
extern const pb_msgdesc_t device_uid_t_msg;
extern const pb_msgdesc_t network_state_t_msg;
extern const pb_msgdesc_t config_packet_t_msg;
extern const pb_msgdesc_t peer_address_t_msg;
extern const pb_msgdesc_t uwb_range_t_msg;
extern const pb_msgdesc_t uwb_info_t_msg;
extern const pb_msgdesc_t uwb_packet_t_msg;
extern const pb_msgdesc_t special_function_t_msg;
extern const pb_msgdesc_t packet_t_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define PACKET_HEADER_FIELDS &packet_header_t_msg
#define SIMPLE_SENSOR_READING_FIELDS &simple_sensor_reading_t_msg
#define SENSOR_READING_FIELDS &sensor_reading_t_msg
#define SENSOR_READING_PAYLOAD_FIELDS &sensor_reading_payload_t_msg
#define SENSOR_CONFIG_FIELDS &sensor_config_t_msg
#define SD_CARD_STATE_FIELDS &sd_card_state_t_msg
#define MARK_STATE_FIELDS &mark_state_t_msg
#define MARK_PACKET_FIELDS &mark_packet_t_msg
#define BATTERY_STATE_FIELDS &battery_state_t_msg
#define DEVICE_FIELDS &device_t_msg
#define SYSTEM_INFO_PACKET_FIELDS &system_info_packet_t_msg
#define AUDIO_COMPRESSION_FIELDS &audio_compression_t_msg
#define AUDIO_CONFIG_FIELDS &audio_config_t_msg
#define SCHEDULE_CONFIG_FIELDS &schedule_config_t_msg
#define LOW_POWER_CONFIG_FIELDS &low_power_config_t_msg
#define CAMERA_CONTROL_FIELDS &camera_control_t_msg
#define DEVICE_UID_FIELDS &device_uid_t_msg
#define NETWORK_STATE_FIELDS &network_state_t_msg
#define CONFIG_PACKET_FIELDS &config_packet_t_msg
#define PEER_ADDRESS_FIELDS &peer_address_t_msg
#define UWB_RANGE_FIELDS &uwb_range_t_msg
#define UWB_INFO_FIELDS &uwb_info_t_msg
#define UWB_PACKET_FIELDS &uwb_packet_t_msg
#define SPECIAL_FUNCTION_FIELDS &special_function_t_msg
#define PACKET_FIELDS &packet_t_msg

/* Maximum encoded size of messages (where known) */
/* SensorReading_size depends on runtime parameters */
#define AUDIO_COMPRESSION_SIZE                   10
#define AUDIO_CONFIG_SIZE                        31
#define BATTERY_STATE_SIZE                       12
#define CAMERA_CONTROL_SIZE                      6
#define CONFIG_PACKET_SIZE                       602
#define DEVICE_SIZE                              11
#define DEVICE_UID_SIZE                          11
#define LOW_POWER_CONFIG_SIZE                    2
#define MARK_PACKET_SIZE                         53
#define MARK_STATE_SIZE                          17
#define NETWORK_STATE_SIZE                       152
#define PACKET_HEADER_SIZE                       23
#define PACKET_SIZE                              995
#define PEER_ADDRESS_SIZE                        14
#define SCHEDULE_CONFIG_SIZE                     38
#define SD_CARD_STATE_SIZE                       24
#define SENSOR_CONFIG_SIZE                       6
#define SENSOR_READING_PAYLOAD_SIZE              41
#define SIMPLE_SENSOR_READING_SIZE               32
#define SPECIAL_FUNCTION_SIZE                    967
#define SYSTEM_INFO_PACKET_SIZE                  355
#define UWB_INFO_SIZE                            35
#define UWB_PACKET_SIZE                          964
#define UWB_RANGE_SIZE                           46

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
