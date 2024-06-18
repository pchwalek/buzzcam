import CoreBluetooth
import Foundation
import SwiftUI
import SwiftProtobuf



class BluetoothModel: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    @Published var centralManager: CBCentralManager!
    @Published var peripherals: [CBPeripheral] = [] // For testing purposes only
    @Published var filteredPeripherals: [CBPeripheral] = []
    @Published var connectedPeripheral: CBPeripheral?
    @Published var targetCharacteristic: CBCharacteristic?
    private var isReconnecting = false
    
    // Structs to populate upon each read, used to store recieved packets
    @Published var systemInfoPacketData: SystemInfoPacketData?
    @Published var configPacketData_Audio: ConfigPacketData_Audio?
    @Published var configPacketData_Schedule: ConfigPacketData_Schedule?
    @Published var configPacketData_Sensor: ConfigPacketData_Sensor?
    @Published var configPacketData_NetworkState: ConfigPacketData_NetworkState?
    @Published var configPacketData_LowPower: ConfigPacketData_LowPower?
    @Published var specialFunctionData: SpecialFunctionData?
    @Published var configPacketData: ConfigPacketData?
    
    // Service and characteristic UUIDs
    private let serviceUUID = CBUUID(string: "CE80")
    private let characteristicUUID_CE71 = CBUUID(string: "CE71")
    private let characteristicUUID_CE72 = CBUUID(string: "CE72")
    private let characteristicUUID_CE73 = CBUUID(string: "CE73")
    
    // New packets to send
    var markPacket: Packet = Packet()
    var systemInfoPacket: Packet = Packet()
    var configPacket: Packet = Packet()
    var specialFunction: Packet = Packet()
    
    var isConnected: Bool = false
    var updatedConfigPacket = false
    var isScanning = true
    var isUserInitiatedDisconnect = false
    
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
        print("Initialized")
    }
    
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn && isScanning {
            centralManager.scanForPeripherals(withServices: nil, options: nil)
        } else {
            // Handle other states as needed
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
        if let peripheralName = peripheral.name, (peripheralName.contains("BuzzCam") || peripheralName.contains("STM") || peripheralName.contains("Buzz") || peripheralName.contains("BUZZ")) {
                // Check if the peripheral is not already in the list
                if !filteredPeripherals.contains(peripheral) {
                    filteredPeripherals.append(peripheral)
                    print("Discovered Peripheral: \(peripheralName) with Identifier: \(peripheral.identifier)")
                }
            }

            if !peripherals.contains(peripheral) { // for testing purposes
                peripherals.append(peripheral)
            }
    }
    
    func connectToPeripheral(_ peripheral: CBPeripheral) {
        centralManager.connect(peripheral, options: nil)
        print("Connected to peripheral")
        isConnected = true;
    }
    
    func disconnect() {
        if let connectedPeripheral = connectedPeripheral {
            centralManager.cancelPeripheralConnection(connectedPeripheral)
        }
        print("Disconnected from peripheral")
        
        reset() // Call the reset method when disconnecting
    }
    
    func reset() {
        // empty everything
        peripherals = []
        filteredPeripherals = []
        connectedPeripheral = nil
        targetCharacteristic = nil
        
        // reset packets
        systemInfoPacketData?.reset()
        configPacketData_Audio?.reset()
        configPacketData_Schedule?.reset()
        configPacketData_Sensor?.reset()
        configPacketData_NetworkState?.reset()
        configPacketData_LowPower?.reset()
        specialFunctionData?.reset()
        configPacketData?.reset()
        
        // Start scanning again when reset and disconnected
        if centralManager.state == .poweredOn {
            centralManager.scanForPeripherals(withServices: nil, options: nil)
        }
        
        updatedConfigPacket = false
        
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.delegate = self
        connectedPeripheral = peripheral
        peripheral.discoverServices([serviceUUID])
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            print("Discovered Services for \(peripheral.name ?? "Unknown"):")
            for service in services {
                print("Service UUID: \(service.uuid)")
                
                // After discovering services, also discover characteristics for each service
                peripheral.discoverCharacteristics([characteristicUUID_CE71, characteristicUUID_CE72, characteristicUUID_CE73], for: service)
            }
        }
        
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard service.uuid == CBUUID(string: "CE80") else {
            return
        }
        
        guard let characteristics = service.characteristics else {
            print("Characteristics are nil for service \(service.uuid)")
            return
        }
        
        for characteristic in characteristics {
            switch characteristic.uuid {
            case characteristicUUID_CE71, characteristicUUID_CE72:
                if characteristics.contains(characteristic) {
                    print("Target characteristic found: \(characteristic.uuid)")
                    readFromCharacteristic(characteristic)
                    subscribeToCharacteristic(characteristic)
                } else {
                    peripheral.discoverCharacteristics([characteristic.uuid], for: service)
                }
            default:
                // Handle other characteristics if needed
                break
            }
        }
    }
    
    func subscribeToCharacteristic(_ characteristic: CBCharacteristic) {
        guard let peripheral = connectedPeripheral else {
            print("Peripheral is nil. Cannot subscribe to notifications.")
            return
        }
        
        if peripheral.state == .connected {
            peripheral.setNotifyValue(true, for: characteristic)
            print("Subscribed to characteristic: \(characteristic.uuid)")
        } else {
            // Handle the case where the peripheral is not connected (e.g., reconnect)
            connectToPeripheral(peripheral)
        }
    }
    
    
    func readFromCharacteristic(_ characteristic: CBCharacteristic) {
        print("in readFromCharacteristic")
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        if peripheral.state == .connected {
            peripheral.readValue(for: characteristic)
        } else {
            // Handle the case where the peripheral is not connected (e.g., reconnect
            connectToPeripheral(peripheral)
        }
    }
    
    func writeToCharacteristic(message: Packet, characteristic: CBCharacteristic) {
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        if peripheral.state == .connected {
            do {
                let data = try message.serializedData()
                peripheral.writeValue(data, for: characteristic, type: .withResponse)
            } catch {
                print("Failed to serialize the protobuf message: \(error)")
            }
        } else {
            // Handle the case where the peripheral is not connected (e.g., reconnect)
            connectToPeripheral(peripheral)
        }
    }
    
    func decodeAndPrintMessage(from characteristic: CBCharacteristic) {
        guard let data = characteristic.value else {
            print("Characteristic decodeAndPrintMessagelue is nil. Cannot decode and print.")
            return
        }
        
        do {
            // get a packet
            let message = try Packet(serializedData: data)
            // Use a switch statement to handle different characteristics
            
            switch characteristic.uuid {
            case characteristicUUID_CE71:
                // Update systemInfoPacketData only when the characteristic is CE71
                // Check if the packet has the field before accessing/storing it
                
                if let sdCardDetected = message.systemInfoPacket.hasSdcardState ? message.systemInfoPacket.sdcardState.detected : self.systemInfoPacketData?.sd_detected {
                }
                
                DispatchQueue.main.async {
                    self.systemInfoPacketData = SystemInfoPacketData(
                        index: message.systemInfoPacket.hasSimpleSensorReading ? message.systemInfoPacket.simpleSensorReading.index :
                            self.systemInfoPacketData?.index ?? 0,
                        
                        temperature: message.systemInfoPacket.hasSimpleSensorReading ? message.systemInfoPacket.simpleSensorReading.temperature :
                            self.systemInfoPacketData?.temperature ?? 0,
                        
                        humidity: message.systemInfoPacket.hasSimpleSensorReading ? message.systemInfoPacket.simpleSensorReading.humidity :
                            self.systemInfoPacketData?.humidity ?? 0,
                        
                        co2: message.systemInfoPacket.hasSimpleSensorReading ? message.systemInfoPacket.simpleSensorReading.co2 :
                            self.systemInfoPacketData?.co2 ?? 0,
                        
                        light_level: message.systemInfoPacket.hasSimpleSensorReading ? message.systemInfoPacket.simpleSensorReading.lightLevel :
                            self.systemInfoPacketData?.light_level ?? 0,
                        
                        sd_detected: message.systemInfoPacket.hasSdcardState ? message.systemInfoPacket.sdcardState.detected : self.systemInfoPacketData?.sd_detected ?? false,
                        
                        space_remaining: message.systemInfoPacket.hasSdcardState ? message.systemInfoPacket.sdcardState.spaceRemaining :
                            self.systemInfoPacketData?.space_remaining ?? 0,
                        
                        estimated_recording_time: message.systemInfoPacket.hasSdcardState ? message.systemInfoPacket.sdcardState.estimatedRemainingRecordingTime : self.systemInfoPacketData?.estimated_recording_time ?? 0,
                        
                        battery_charging: message.systemInfoPacket.hasBatteryState ? message.systemInfoPacket.batteryState.charging :
                            self.systemInfoPacketData?.battery_charging ?? false,
                        
                        battery_voltage: message.systemInfoPacket.hasBatteryState ? message.systemInfoPacket.batteryState.voltage :
                            self.systemInfoPacketData?.battery_voltage ?? 0,
                        
                        device_recording: message.systemInfoPacket.deviceRecording,
                        
                        mark_number: message.systemInfoPacket.hasMarkState ? message.systemInfoPacket.markState.markNumber :
                            self.systemInfoPacketData?.mark_number ?? 0,
                        
                        discovered_devices: message.systemInfoPacket.discoveredDevices
                    )
                }
                print("Updated systemInfoPacketData with CE71 characteristic")
            case characteristicUUID_CE72:
                // Update configPacketData only when the characteristic is CE72
                // check if hasAudioConfig
                
                // populate initial config
                configPacket.configPacket = message.configPacket
                
                DispatchQueue.main.async {
                    if message.configPacket.hasAudioConfig {
                        self.configPacketData_Audio = ConfigPacketData_Audio(
                            channel1: message.configPacket.audioConfig.channel1,
                            channel2: message.configPacket.audioConfig.channel2,
                            sampleFreq: message.configPacket.audioConfig.sampleFreq,
                            bitResolution: message.configPacket.audioConfig.bitResolution,
                            audioCompressionEnabled: message.configPacket.audioConfig.hasAudioCompression ? message.configPacket.audioConfig.audioCompression.enabled :
                                self.configPacketData_Audio?.audioCompressionEnabled ?? false,
                            audioCompressionType: message.configPacket.audioConfig.hasAudioCompression ? message.configPacket.audioConfig.audioCompression.compressionType :
                                self.configPacketData_Audio?.audioCompressionType ?? .opus,
                            audioCompressionFactor: message.configPacket.audioConfig.hasAudioCompression ? message.configPacket.audioConfig.audioCompression.compressionFactor :
                                self.configPacketData_Audio?.audioCompressionFactor ?? 0,
                            estimatedRecordTime: message.configPacket.audioConfig.estimatedRecordTime,
                            freeRunMode: message.configPacket.audioConfig.freeRunMode,
                            chirpEnable: message.configPacket.audioConfig.chirpEnable,
                            micGain: message.configPacket.audioConfig.micGain
                        )
                    }
                    self.configPacketData_Schedule = ConfigPacketData_Schedule(scheduleConfig: message.configPacket.scheduleConfig)
                    
                    if message.configPacket.hasSensorConfig {
                        self.configPacketData_Sensor = ConfigPacketData_Sensor(
                            enableTemperature: message.configPacket.sensorConfig.enableTemperature,
                            enableHumidity: message.configPacket.sensorConfig.enableHumidity,
                            enableGas: message.configPacket.sensorConfig.enableGas)
                    }
                    
                    if message.configPacket.hasNetworkState {
                        self.configPacketData_NetworkState = ConfigPacketData_NetworkState(
                            numberOfDiscoveredDevices: message.configPacket.networkState.numberOfDiscoveredDevices,
                            discoveredDeviceUid: message.configPacket.networkState.discoveredDeviceUid,
                            slaveSync: message.configPacket.networkState.slaveSync,
                            masterNode: message.configPacket.networkState.masterNode,
                            panID: message.configPacket.networkState.panID,
                            channel: message.configPacket.networkState.channel
                        )
                    }
                    
                    if message.configPacket.hasLowPowerConfig {
                        self.configPacketData_LowPower = ConfigPacketData_LowPower(lowPowerMode: message.configPacket.lowPowerConfig.lowPowerMode)
                    }
                    
                    self.specialFunctionData = SpecialFunctionData(uwbPacket: message.specialFunction.uwbPacket)
                    
                    self.configPacketData = ConfigPacketData(enableRecording: message.configPacket.enableRecording,
                                                             enableLed: message.configPacket.enableLed)
                    print("configPacket enableLed:", message.configPacket.enableLed)
                }
                print("Updated configPacketData with CE72 characteristic")
                updatedConfigPacket = true
            default:
                // Handle other characteristics if needed
                break
            }
            
        } catch {
            print("Failed to decode and print message: \(error)")
        }
    }
    
    
    
    
    
    // Can receive notifications through the didUpdateValueFor delegate method
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        print("in didUpdateValue")
        
        switch characteristic.uuid {
            case characteristicUUID_CE71:
                print("this is CE71")
                decodeAndPrintMessage(from: characteristic)
            case characteristicUUID_CE72:
                print("this is CE72")
                decodeAndPrintMessage(from: characteristic)
            default:
                // Handle other characteristics if needed
                break
        }
    }
    

    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        // Remove the disconnected peripheral from the list of filtered peripherals
        filteredPeripherals.removeAll { $0.identifier == peripheral.identifier }
        print("Disconnected Peripheral: \(peripheral.name ?? "Unknown") with Identifier: \(peripheral.identifier)")
        isConnected = false
        reset()
    }
    
    
    
    // PACKET BEHAVIOR
    
    func sendMarkPacket() {
        do {
            let markPacketData = try markPacket.serializedData()
            sendMarkPacketData()
        } catch {
            print("Error serializing MarkPacket: \(error)")
        }
    }
    
    func sendSystemInfoPacket() {
        do {
            let systemInfoPacketData = try systemInfoPacket.serializedData()
            sendSystemInfoPacketData()
        } catch {
            print("Error serializing SystemInfoPacket: \(error)")
        }
    }
    
    func sendConfigPacket() {
        do {
            let configPacketData = try configPacket.serializedData()
            sendConfigPacketData()
        } catch {
            print("Error serializing configPacket: \(error)")
        }
    }
    
    func sendSpecialFunction() {
        do {
            let specialFunctionData = try specialFunction.serializedData()
            sendSpecialFunctionData()
        } catch {
            print("Error serializing configPacket: \(error)")
        }
    }
    
    func sendMarkPacketData() {
        // Ensure that the peripheral is connected
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        // Check if the service and characteristic have been discovered
        if let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE72 }) {
            // If discovered, proceed to send data
            sendMarkPacketDataHelper(peripheral: peripheral, characteristic: characteristic)
        } else {
            // If not discovered, initiate service discovery
            peripheral.discoverServices([serviceUUID])
        }
    }
    
    // Helper method to send data when service and characteristic are available
    func sendMarkPacketDataHelper(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        do {
            // Serialize the markPacket into Data
            let markPacketData = try markPacket.serializedData() ?? Data()
            peripheral.writeValue(markPacketData, for: characteristic, type: .withResponse)
            print("Mark packet sent")
        } catch {
            print("Error sending MarkPacket: \(error)")
        }
    }
    
    func sendSystemInfoPacketData() {
        // Ensure that the peripheral is connected
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        // Check if the service and characteristic have been discovered
        if let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE72 }) {
            // If discovered, proceed to send data
            sendSystemInfoPacketDataHelper(peripheral: peripheral, characteristic: characteristic)
        } else {
            // If not discovered, initiate service discovery
            peripheral.discoverServices([serviceUUID])
        }
    }
    
    // Helper method to send system info data when service and characteristic are available
    func sendSystemInfoPacketDataHelper(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        do {
            // Serialize the systemInfoPacket into Data
            let systemInfoPacketData = try systemInfoPacket.serializedData() ?? Data()
            peripheral.writeValue(systemInfoPacketData, for: characteristic, type: .withResponse)
            print("System info packet sent")
        } catch {
            print("Error serializing SystemInfoPacket: \(error)")
        }
    }
    
    func sendConfigPacketData() {
        // Ensure that the peripheral is connected
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        // Check if the service and characteristic have been discovered
        if let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE72 }) {
            // If discovered, proceed to send data
            sendConfigPacketDataHelper(peripheral: peripheral, characteristic: characteristic)
        } else {
            // If not discovered, initiate service discovery
            peripheral.discoverServices([serviceUUID])
        }
    }
    
    // Helper method to send system info data when service and characteristic are available
    func sendConfigPacketDataHelper(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        do {
            // Serialize the configPacket into Data
            let configPacketData = try configPacket.serializedData() ?? Data()
            peripheral.writeValue(configPacketData, for: characteristic, type: .withResponse)
            print("Config packet sent")
        } catch {
            print("Error serializing configPacket: \(error)")
        }
    }
    
    func sendSpecialFunctionData() {
        // Ensure that the peripheral is connected
        guard let peripheral = connectedPeripheral else {
            print("Peripheral not connected.")
            return
        }
        
        // Check if the service and characteristic have been discovered
        if let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE72 }) {
            // If discovered, proceed to send data
            sendSpecialFunctionDataHelper(peripheral: peripheral, characteristic: characteristic)
        } else {
            // If not discovered, initiate service discovery
            peripheral.discoverServices([serviceUUID])
        }
    }
    
    // Helper method to send system info data when service and characteristic are available
    func sendSpecialFunctionDataHelper(peripheral: CBPeripheral, characteristic: CBCharacteristic) {
        do {
            // Serialize the specialFunction into Data
            let specialFunctionData = try specialFunction.serializedData() ?? Data()
            peripheral.writeValue(specialFunctionData, for: characteristic, type: .withResponse)
            print("Special function sent")
        } catch {
            print("Error serializing specialFunction: \(error)")
        }
    }
    
}
