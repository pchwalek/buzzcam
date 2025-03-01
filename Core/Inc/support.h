#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#define DISABLE_WIRELESS 0

#define SIMULATE_GPS 	1

#define LORA_SEND_INTERVAL_MINS 1
#define GPS_FIX_INTERVAL_MINS	1440

/*     LoRa Settings     */
#define MAX_PWR			3 // 22 dBm
#define HIGH_PWR		2 // 20 dBm
#define MID_PWR			1 // 17 dBm
#define LOW_PWR			0 // 14 dBm
#define ULTRA_LOW_PWR	-1 // 0 dBm

//#define LORA_FREQ		868000000
#define LORA_FREQ		915000000
#define LORA_POWER_LVL	HIGH_PWR

#define LORA_LONG_RANGE_LOW_BW		3
#define LORA_MID_RANGE_MID_BW		2
#define LORA_SHORT_RANGE_HIGH_BW	1
#define LORA_MAX_BW					0
#define LORA_MAX_RANGE				4 // this one takes a long time to send a packet

#define LORA_RANGE_BW	LORA_LONG_RANGE_LOW_BW

#define LORA_TX_PERIOD_MS		5000
#define LORA_PKT_RETRY	5


typedef enum {
    OFF = 0,
    SD1_EN = 1,
	SD2_EN = 2
} SDCardState;

typedef struct {
    bool isLoRaActive;
    bool isAccelerometerActive;
    bool isMicrophoneActive;
    bool isEnvironmentalSensorActive;
    bool isGPSActive;
    bool isBatteryLevelSensingActive;
    bool isBuzzerActive;
    bool isMAX78000Active;
    bool isFRAMActive;
    bool isUWBActive;
    SDCardState SDCardState;
} SystemState;

typedef struct {
    bool isLoRaEnabled;
    bool isAccelerometerEnabled;
    bool isMicrophoneEnabled;
    bool isEnvironmentalSensorEnabled;
    bool isGPSEnabled;
    bool isBatteryLevelSensingEnabled;
    bool isBuzzerEnabled;
    bool isMAX78000Enabled;
    bool isUWBEnabled;
    bool isSDEnabled;
} SystemPowerSupervisor;

typedef enum {
    FULL = 0,
    REDUCED = 1,
    LOW = 2,
    CRITICAL = 3
} PowerRegime;


// Create a global instance of SystemState to keep track of subsystem states
extern SystemState systemState;

extern SystemPowerSupervisor systemPowerSupervisor;
extern PowerRegime powerRegime;


// Function prototypes for turning on/off various systems
void Control_Microphone_FRAM_Power(bool enable);
void Control_Secondary_Power(bool enable);
void Control_GPS_Power(bool enable);
void Control_BatteryMonitor_Power(bool enable);
void Control_Buzzer_Power(bool enable);
void Control_MAX78000_Power(bool enable);
void Control_SDCard_Power(SDCardState sdcardState);
void Control_UWB_Power(bool enable);

void TurnOffAllSystems();

bool updateSystemPowerSupervisor(SystemPowerSupervisor* supervisor, PowerRegime* regime);
float getBattVltg(void);
float calculate_battery_percentage(double x);

// Function prototypes for checking if systems are enabled
bool Is_Microphone_Enabled();
bool Is_Secondary_Enabled();
bool Is_BatteryMonitor_Enabled();
bool Is_GPS_Enabled();
bool Is_Buzzer_Enabled();
bool Is_MAX78000_Enabled();
bool Is_UWB_Enabled();

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STATE_H */
