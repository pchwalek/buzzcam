#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STATE_H */
