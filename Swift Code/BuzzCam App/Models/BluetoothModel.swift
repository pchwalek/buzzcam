import CoreBluetooth
import Foundation
import SwiftUI
import SwiftProtobuf



class BluetoothModel: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    private var centralManager: CBCentralManager!
    @Published var peripherals: [CBPeripheral] = [] // for testing purposes only
    @Published var filteredPeripherals: [CBPeripheral] = []
    @Published var connectedPeripheral: CBPeripheral?
    @Published var targetCharacteristic: CBCharacteristic?
    private var isReconnecting = false
    
    // Structs to populate upon each read
    @Published var systemInfoPacketData: SystemInfoPacketData?
    @Published var configPacketData_Audio: ConfigPacketData_Audio?
    @Published var configPacketData_Schedule: ConfigPacketData_Schedule?
    @Published var configPacketData_Sensor: ConfigPacketData_Sensor?
    @Published var configPacketData_Discover: ConfigPacketData_Discover?
    @Published var configPacketData_LowPower: ConfigPacketData_LowPower?
    @Published var specialFunctionData: SpecialFunctionData?
    
    // Global message packet
    //    @Published var currentMessage: Packet?
    
    
    var isScanning = true
    var isUserInitiatedDisconnect = false
    
    // Service and characteristic UUIDs here
    private let serviceUUID = CBUUID(string: "CE70")
    private let characteristicUUID_CE71 = CBUUID(string: "CE71")
    private let characteristicUUID_CE72 = CBUUID(string: "CE72")
    private let characteristicUUID_CE73 = CBUUID(string: "CE73")
    
    // packets to send
    var markPacket: Packet?
    var systemInfoPacket: Packet?
    var configPacket: Packet?
    var specialFunction: Packet?
    
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
        if let peripheralName = peripheral.name, (peripheralName.contains("BuzzCam") ||  peripheralName.contains("STM")){
            if !filteredPeripherals.contains(peripheral) {
                filteredPeripherals.append(peripheral)
                print("Discovered Peripheral: \(peripheral.name ?? "Unknown") with Identifier: \(peripheral.identifier)")
            }
        }
        if !peripherals.contains(peripheral) { // for testing purposes
            peripherals.append(peripheral)
        }
    }
    
    func connectToPeripheral(_ peripheral: CBPeripheral) {
        centralManager.connect(peripheral, options: nil)
        print("Connected to peripheral")
    }
    
    func disconnect() {
        if let connectedPeripheral = connectedPeripheral {
            centralManager.cancelPeripheralConnection(connectedPeripheral)
        }
        print("Disconnected from peripheral")
        
        reset() // Call the reset method when disconnecting
    }
    
    func reset() {
        peripherals = []
        filteredPeripherals = []
        connectedPeripheral = nil
        targetCharacteristic = nil
        
        // reset packets
        systemInfoPacketData?.reset()
        configPacketData_Audio?.reset()
        configPacketData_Schedule?.reset()
        configPacketData_Sensor?.reset()
        configPacketData_Discover?.reset()
        configPacketData_LowPower?.reset()
        specialFunctionData?.reset()
        // start scanning again when reset and disconnected
        if centralManager.state == .poweredOn {
            centralManager.scanForPeripherals(withServices: nil, options: nil)
        }
        
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
        guard service.uuid == CBUUID(string: "CE70") else {
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
            print("Characteristic value is nil. Cannot decode and print.")
            return
        }
        
        do {
            let message = try Packet(serializedData: data)
            
            // Use a switch statement to handle different characteristics
            switch characteristic.uuid {
            case characteristicUUID_CE71:
                // Update systemInfoPacketData only when the characteristic is CE71
                DispatchQueue.main.async {
                    self.systemInfoPacketData = SystemInfoPacketData(
                        index: message.systemInfoPacket.simpleSensorReading.index,
                        temperature: message.systemInfoPacket.simpleSensorReading.temperature,
                        humidity: message.systemInfoPacket.simpleSensorReading.humidity,
                        co2: message.systemInfoPacket.simpleSensorReading.co2,
                        light_level: message.systemInfoPacket.simpleSensorReading.lightLevel,
                        sd_detected: message.systemInfoPacket.sdcardState.detected,
                        space_remaining: message.systemInfoPacket.sdcardState.spaceRemaining,
                        estimated_recording_time: message.systemInfoPacket.sdcardState.estimatedRemainingRecordingTime,
                        battery_charging: message.systemInfoPacket.batteryState.charging,
                        battery_voltage: message.systemInfoPacket.batteryState.voltage,
                        device_recording: message.systemInfoPacket.deviceRecording,
                        mark_number: message.systemInfoPacket.markState.markNumber
                    )
                }
                print("Updated systemInfoPacketData with CE71 characteristic")
            case characteristicUUID_CE72:
                // Update configPacketData only when the characteristic is CE72
                DispatchQueue.main.async {
                    self.configPacketData_Audio = ConfigPacketData_Audio(
                        channel1: message.configPacket.audioConfig.channel1,
                        channel2: message.configPacket.audioConfig.channel2,
                        sampleFreq: message.configPacket.audioConfig.sampleFreq,
                        bitResolution: message.configPacket.audioConfig.bitResolution,
                        audioCompressionEnabled: message.configPacket.audioConfig.audioCompression.enabled,
                        audioCompressionType: message.configPacket.audioConfig.audioCompression.compressionType,
                        audioCompressionFactor: message.configPacket.audioConfig.audioCompression.compressionFactor,
                        estimatedRecordTime: message.configPacket.audioConfig.estimatedRecordTime,
                        freeRunMode: message.configPacket.audioConfig.freeRunMode
                    )
                    
                    self.configPacketData_Schedule = ConfigPacketData_Schedule(scheduleConfig: message.configPacket.scheduleConfig)
                    
                    self.configPacketData_Sensor = ConfigPacketData_Sensor(
//                        samplePeriodMs: message.configPacket.sensorConfig.samplePeriodMs,
                        enableTemperature: message.configPacket.sensorConfig.enableTemperature,
                        enableHumidity: message.configPacket.sensorConfig.enableHumidity,
                        enableGas: message.configPacket.sensorConfig.enableGas)
                    
                    self.configPacketData_Discover = ConfigPacketData_Discover(
                        numberOfDiscoveredDevices: message.configPacket.networkState.numberOfDiscoveredDevices,
                        discoveredDeviceUid: message.configPacket.networkState.discoveredDeviceUid
                    )
                    
                    self.configPacketData_LowPower = ConfigPacketData_LowPower(lowPowerMode: message.configPacket.lowPowerConfig.lowPowerMode)
                    
                    self.specialFunctionData = SpecialFunctionData(uwbPacket: message.specialFunction.uwbPacket)
                }
                print("Updated configPacketData with CE72 characteristic")
            default:
                // Handle other characteristics if needed
                break
            }
            
            //            //            print("Decoded Message: \(message)")
            //            print("Temperature: \(message.systemInfoPacket.simpleSensorReading.temperature)")
            //            print("Audio compression factor: \(message.configPacket.audioConfig.audioCompression.compressionFactor)")
            //            print("Bit resolution: \(message.configPacket.audioConfig.bitResolution)")
            //            print("Sampling Frequency: \(message.configPacket.audioConfig.sampleFreq)")
            //            print("Channel 1 enabled: \(message.configPacket.audioConfig.channel1)")
            //            print("Channel 2 enabled: \(message.configPacket.audioConfig.channel2)")
            //            //            print("Wakeup Cameras: \(message.configPacket.cameraControl.wakeupCameras)")
            //            //            print("Beep Enabled: \(message.systemInfoPacket.markState.beepEnabled)")
        } catch {
            print("Failed to decode and print message: \(error)")
        }
    }
    
    
    
    
    
    // Then you can receive notifications through the didUpdateValueFor delegate method
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        print("in didUpdateValue")
        
        switch characteristic.uuid {
        case characteristicUUID_CE71, characteristicUUID_CE72:
            decodeAndPrintMessage(from: characteristic)
        default:
            // Handle other characteristics if needed
            break
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        //      // Attempt to reconnect after a delay if not already in progress
        // Attempt automatic reconnection only if it's not a user-initiated disconnect
        //        if !isUserInitiatedDisconnect {
        //            if !isReconnecting {
        //                isReconnecting = true
        //                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
        //                    central.connect(peripheral, options: nil)
        //                }
        //            }
        //        }
    }
    
    
    // PACKET BEHAVIOR
    
    func sendMarkPacket() {
        if let markPacket = markPacket {
            do {
                let markPacketData = try markPacket.serializedData()
                sendMarkPacketData()
            } catch {
                print("Error serializing MarkPacket: \(error)")
            }
        }
    }
    
    func sendSystemInfoPacket() {
        if let systemInfoPacket = systemInfoPacket {
            do {
                let systemInfoPacketData = try systemInfoPacket.serializedData()
                sendSystemInfoPacketData()
            } catch {
                print("Error serializing SystemInfoPacket: \(error)")
            }
        }
    }
    
    func sendConfigPacket() {
        if let configPacket = configPacket {
            do {
                let configPacketData = try configPacket.serializedData()
                sendConfigPacketData()
            } catch {
                print("Error serializing configPacket: \(error)")
            }
        }
    }
    
    func sendSpecialFunction() {
        if let specialFunction = specialFunction {
            do {
                let specialFunctionData = try specialFunction.serializedData()
                sendSpecialFunctionData()
            } catch {
                print("Error serializing configPacket: \(error)")
            }
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
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) {
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
            let markPacketData = try markPacket?.serializedData() ?? Data()
            peripheral.writeValue(markPacketData, for: characteristic, type: .withoutResponse)
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
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) {
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
            let systemInfoPacketData = try systemInfoPacket?.serializedData() ?? Data()
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
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) {
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
            let configPacketData = try configPacket?.serializedData() ?? Data()
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
           let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) {
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
            let specialFunctionData = try specialFunction?.serializedData() ?? Data()
            peripheral.writeValue(specialFunctionData, for: characteristic, type: .withResponse)
            print("Special function sent")
        } catch {
            print("Error serializing specialFunction: \(error)")
        }
    }
    
}

