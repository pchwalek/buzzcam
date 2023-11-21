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
    @Published var systemInfoPacketData: SystemInfoPacketData?
    
    @Published var characteristicValue: String = "" // A property to hold the characteristic value
    
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
        if let peripheralName = peripheral.name, peripheralName.contains("BuzzCam") {
            if !filteredPeripherals.contains(peripheral) {
                filteredPeripherals.append(peripheral)
                print("Discovered Peripheral: \(peripheral.name ?? "Unknown") with Identifier: \(peripheral.identifier)")
//                connectToPeripheral(peripheral)
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
        // Set the flag to indicate user-initiated disconnect
//        isUserInitiatedDisconnect = true

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
        characteristicValue = ""
        // reset packet
        systemInfoPacketData?.reset()
        // Reset the flag after performing the disconnect and reset logic
//        isUserInitiatedDisconnect = false
        // start scanning again when reset and disconnected
        if centralManager.state == .poweredOn {
            centralManager.scanForPeripherals(withServices: nil, options: nil)
        }
        
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        // Handle the reconnection
//        isReconnecting = false
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
                        mark_number: message.systemInfoPacket.markState.markNumber,
                        beep_enabled: message.systemInfoPacket.markState.beepEnabled
                    )
                }
                print("Updated systemInfoPacketData with CE71 characteristic")
            default:
                // Handle other characteristics if needed
                break
            }

            print("Decoded Message: \(message)")
            print("Temperature: \(message.systemInfoPacket.simpleSensorReading.temperature)")
            print("Beep Enabled: \(message.systemInfoPacket.markState.beepEnabled)")
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
    
//    func sendMarkPacketData() {
//        // Ensure that the peripheral has the service and characteristic
//        guard let peripheral = connectedPeripheral,
//              let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
//              let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) else {
//            print("Service or characteristic not found on the peripheral.")
//            return
//        }
//
//        do {
//            // Serialize the markPacket into Data
////            print("markPacketData \(markPacket)")
//            let markPacketData = try markPacket?.serializedData() ?? Data()
////            
////            let hexString = markPacketData.map { String(format: "%02x", $0) }.joined()
////            print("Serialized Data as Hex String: \(hexString)")
//            peripheral.writeValue(markPacketData, for: characteristic, type: .withResponse)
//            print("Mark packet sent")
//        } catch {
//            print("Error sending MarkPacket: \(error)")
//        }
//    }
    
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
            peripheral.writeValue(markPacketData, for: characteristic, type: .withResponse)
            print("Mark packet sent")
        } catch {
            print("Error sending MarkPacket: \(error)")
        }
    }
    
//    func sendSystemInfoPacketData() {
//        // Ensure that the peripheral has the service and characteristic
//        guard let peripheral = connectedPeripheral,
//              let service = peripheral.services?.first(where: { $0.uuid == serviceUUID }),
//              let characteristic = service.characteristics?.first(where: { $0.uuid == characteristicUUID_CE73 }) else {
//            print("Service or characteristic not found on the peripheral.")
//            return
//        }
//
//        do {
//            // Serialize the systemInfoPacket into Data
//            let systemInfoPacketData = try systemInfoPacket?.serializedData() ?? Data()
////            let hexString = systemInfoPacketData.map { String(format: "%02x", $0) }.joined()
////            print("Serialized Data as Hex String: \(hexString)")
//            peripheral.writeValue(systemInfoPacketData, for: characteristic, type: .withResponse)
//            print("System info packet sent")
//        } catch {
//            print("Error serializing SystemInfoPacket: \(error)")
//        }
//    }
    
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
    
}
//------------------------

//import CoreBluetooth
//import Foundation
//import SwiftUI
//
//class BluetoothModel: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
//    private var centralManager: CBCentralManager!
//    @Published var peripherals: [CBPeripheral] = []
//    @Published var filteredPeripherals: [CBPeripheral] = []
//    @Published var connectedPeripheral: CBPeripheral?
//    @Published var targetCharacteristic: CBCharacteristic?
//
//    @Published var characteristicValue: String = "" // A property to hold the characteristic value
//
//    var isScanning = true
//
//    private let serviceUUID = CBUUID(string: "CE70")
//    private let characteristicUUID_CE71 = CBUUID(string: "CE71")
//    private let characteristicUUID_CE72 = CBUUID(string: "CE72")
//    private let characteristicUUID_CE73 = CBUUID(string: "CE73")
//
//    enum ConnectionState {
//        case disconnected
//        case connecting
//        case connected
//    }
//
//    @Published var connectionState: ConnectionState = .disconnected
//
//    override init() {
//        super.init()
//        centralManager = CBCentralManager(delegate: self, queue: nil)
//        print("initialized")
//    }
//
//    func centralManagerDidUpdateState(_ central: CBCentralManager) {
//        if central.state == .poweredOn && isScanning {
//            centralManager.scanForPeripherals(withServices: nil, options: nil)
//        } else {
//            // Handle other states as needed
//        }
//    }
//
//    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
//        if let peripheralName = peripheral.name, peripheralName.contains("BuzzCam") {
//            if !filteredPeripherals.contains(peripheral) {
//                filteredPeripherals.append(peripheral)
//                print("Discovered Peripheral: \(peripheral.name ?? "Unknown") with Identifier: \(peripheral.identifier)")
//            }
//        }
//    }
//
//    func connectToPeripheral(_ peripheral: CBPeripheral) {
//        centralManager.connect(peripheral, options: nil)
//        connectionState = .connecting
//    }
//
//    func disconnect() {
//        if let connectedPeripheral = connectedPeripheral {
//            centralManager.cancelPeripheralConnection(connectedPeripheral)
//            connectionState = .disconnected
//        }
//    }
//
//    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
//        peripheral.delegate = self
//        connectedPeripheral = peripheral
//        peripheral.discoverServices(nil)
//        connectionState = .connected
//    }
//
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
//        if let services = peripheral.services {
//            for service in services {
//                if service.uuid == serviceUUID {
//                    targetCharacteristic = service.characteristics?.first { $0.uuid == characteristicUUID_CE71 }
//                    if let targetCharacteristic = targetCharacteristic {
//                        peripheral.discoverCharacteristics([targetCharacteristic.uuid], for: service)
//                    }
//                }
//            }
//        }
//    }
//
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
//        if let characteristics = service.characteristics {
//            for characteristic in characteristics {
//                print("Discovered Characteristic: \(characteristic.uuid)")
//            }
//        }
//    }
//
//    func readFromCharacteristic() {
//        guard let peripheral = connectedPeripheral, let characteristic = targetCharacteristic, connectionState == .connected else {
//            print("Peripheral or characteristic is nil or not connected. Cannot read.")
//            return
//        }
//
//        peripheral.readValue(for: characteristic)
//    }
//
//    func writeToCharacteristic(_ value: String) {
//        guard let peripheral = connectedPeripheral, let characteristic = targetCharacteristic, connectionState == .connected else {
//            print("Peripheral or characteristic is nil or not connected. Cannot write.")
//            return
//        }
//
//        if let data = value.data(using: .utf8) {
//            peripheral.writeValue(data, for: characteristic, type: .withResponse)
//        }
//    }
//
//    func decodeAndPrintMessage(from characteristic: CBCharacteristic) {
//        guard let data = characteristic.value, let message = String(data: data, encoding: .utf8) else {
//            print("Failed to decode and print message.")
//            return
//        }
//
//        print("Decoded Message: \(message)")
//    }
//
//    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
//        // Handle disconnection
//        connectionState = .disconnected
//    }
//}




//-----------------
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
//        if let services = peripheral.services {
//            print("Discovered Services for \(peripheral.name ?? "Unknown"):")
//            for service in services {
//                print("Service UUID: \(service.uuid)")
//                
//                // After discovering services, also discover characteristics for each service
//                peripheral.discoverCharacteristics(nil, for: service)
//            }
//        }
//    }

//    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
//        if let characteristics = service.characteristics {
//            print("Characteristics for Service \(service.uuid) of \(peripheral.name ?? "Unknown"):")
//            for characteristic in characteristics {
//                print("Characteristic UUID: \(characteristic.uuid)")
//            }
//        }
//    }
//}

//////
//////  BluetoothModel.swift
//////  BuzzCam App
//////
//////  Created by Responsive Environments on 10/23/23.
//////
////
//
////
////class BluetoothManager: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
////    private var centralManager: CBCentralManager!
////    @Published var discoveredDevices: [BluetoothDevice] = []
////    
////    override init() {
////        super.init()
////        centralManager = CBCentralManager(delegate: self, queue: nil)
////    }
////    
////    func centralManagerDidUpdateState(_ central: CBCentralManager) {
////        if central.state == .poweredOn {
////            centralManager.scanForPeripherals(withServices: nil, options: nil)
////        } else {
////            // Handle Bluetooth not available
////        }
////    }
////    
////    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
////        let device = BluetoothDevice(peripheral: peripheral, rssi: rssi.intValue)
////        if !discoveredDevices.contains(device) {
////            discoveredDevices.append(device)
////        }
////    }
////    
////    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
////        peripheral.delegate = self
////        peripheral.discoverServices(nil)
////    }
////    
////    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
////        // Handle disconnection
////    }
////    
////    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
////        guard let services = peripheral.services else { return }
////        for service in services {
////            peripheral.discoverCharacteristics(nil, for: service)
////        }
////    }
////}
////
////struct BluetoothDevice: Identifiable, Equatable {
////    let id = UUID()
////    let peripheral: CBPeripheral
////    let rssi: Int
////    var characteristics: [CBCharacteristic] = []
////    
////    init(peripheral: CBPeripheral, rssi: Int) {
////        self.peripheral = peripheral
////        self.rssi = rssi
////    }
////}
////
////
////class BluetoothViewModel: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
////    private var centralManager: CBCentralManager!
////    private var peripheral: CBPeripheral?
////    private var characteristic: CBCharacteristic?
////
////    @Published var filteredPeripherals: [CBPeripheral] = []
////    @Published var characteristicValue: String = ""
////
////    override init() {
////        super.init()
////        centralManager = CBCentralManager(delegate: self, queue: nil)
////    }
////
////    func startScanning() {
////        centralManager.scanForPeripherals(withServices: nil, options: nil)
////    }
////
////    func connect(peripheral: CBPeripheral) {
////        self.peripheral = peripheral
////        centralManager.connect(peripheral, options: nil)
////    }
////
////    func centralManagerDidUpdateState(_ central: CBCentralManager) {
////        if central.state == .poweredOn {
////            startScanning()
////        }
////    }
////
////    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
////        if let peripheralName = peripheral.name, peripheralName.contains("BuzzCam") {
////            filteredPeripherals.append(peripheral)
////        }
////    }
////
////    func readCharacteristic(peripheral: CBPeripheral) {
////        guard let characteristic = characteristic else { return }
////        peripheral.readValue(for: characteristic)
////    }
////
////    func writeCharacteristic(peripheral: CBPeripheral, value: String) {
////        guard let characteristic = characteristic else { return }
////        let data = value.data(using: .utf8)!
////        peripheral.writeValue(data, for: characteristic, type: .withResponse)
////    }
////
////    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
////        peripheral.delegate = self
////        peripheral.discoverServices(nil)
////    }
////
////    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
////        if let services = peripheral.services {
////            for service in services {
////                if service.uuid == CBUUID(string: "YourServiceUUID") {
////                    peripheral.discoverCharacteristics(nil, for: service)
////                }
////            }
////        }
////    }
////
////    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
////        if let characteristics = service.characteristics {
////            for characteristic in characteristics {
////                if characteristic.uuid == CBUUID(string: "YourCharacteristicUUID") {
////                    self.characteristic = characteristic
////                }
////            }
////        }
////    }
////
////    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
////        if let data = characteristic.value, let value = String(data: data, encoding: .utf8) {
////            characteristicValue = value
////        }
////    }
////}
//
//
//import CoreBluetooth
//import Combine
//import Foundation
//import SwiftUI
//
//class BluetoothModel: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
//    private var centralManager: CBCentralManager!
//    @Published var peripherals: [CBPeripheral] = []
//    @Published var filteredPeripherals: [CBPeripheral] = []
//    @Published var connectedPeripheral: CBPeripheral?
//    @Published var targetCharacteristic: CBCharacteristic?
//    
//    @Published var characteristicValue: String = "" // A property to hold the characteristic value
//    
//    var isScanning = true
////    var yourDesiredNumberOfDevices = 10
//
//    // Add your service and characteristic UUIDs here
////    private let serviceUUID = CBUUID(string: "YourServiceUUID")
////    private let characteristicUUID = CBUUID(string: "YourCharacteristicUUID")
//
//
//    
//    override init() {
//        super.init()
//        centralManager = CBCentralManager(delegate: self, queue: nil)
//        print("initialized")
//    }
//    
//    func centralManagerDidUpdateState(_ central: CBCentralManager) {
//        if central.state == .poweredOn && isScanning {
//            centralManager.scanForPeripherals(withServices: nil, options: nil)
//        } else {
//            // Handle other states as needed
//        }
//    }
//    
//    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String: Any], rssi: NSNumber) {
//        if let peripheralName = peripheral.name, peripheralName.contains("BuzzCam") {
//            if !filteredPeripherals.contains(peripheral) {
//                filteredPeripherals.append(peripheral)
//                print("Discovered Peripheral: \(peripheral.name ?? "Unknown") with Identifier: \(peripheral.identifier)")
//                connectToPeripheral(peripheral)
////                readFromCharacteristic()
//                
//            }
//        }
////    }
//        
//        // Check if you've discovered all the devices you need
////        if peripherals.count >= yourDesiredNumberOfDevices {
////            stopScanning()
////        }
//    }
//    
////    func stopScanning() {
////        centralManager.stopScan()
////        isScanning = false
////    }
//    
//    func connectToPeripheral(_ peripheral: CBPeripheral) {
//        connectedPeripheral = peripheral
//        peripheral.delegate = self
//        peripheral.discoverServices(nil)
//    }
//    
//    func disconnect() {
//        if let connectedPeripheral = connectedPeripheral {
//            centralManager.cancelPeripheralConnection(connectedPeripheral)
//        }
//    }
//    
//    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
//        peripheral.delegate = self
//        connectedPeripheral = peripheral
//        peripheral.discoverServices(nil)
//        readFromCharacteristic()
//    }
//    
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
//        if let services = peripheral.services {
//            for service in services {
//                if service.uuid == CBUUID(string:  "0000ce70-0000-1000-8000-00805f9b34fb") {
//                    targetCharacteristic = service.characteristics?.first { $0.uuid == CBUUID(string: "0000ce72-0000-1000-8000-00805f9b34fb") }
//                    if let targetCharacteristic = targetCharacteristic {
//                        peripheral.discoverCharacteristics([targetCharacteristic.uuid], for: service)
//                    }
//                }
//            }
//        }
//    }
//
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
//        if let characteristics = service.characteristics {
//            for characteristic in characteristics {
//                print("Discovered Characteristic: \(characteristic.uuid)")
//            }
//        }
//    }
//    
//    func readFromCharacteristic() {
//        guard let peripheral = connectedPeripheral, let characteristic = targetCharacteristic else {
//            print("Peripheral or characteristic is nil. Cannot read.")
//            return
//        }
//
//        if peripheral.state == .connected {
//            peripheral.readValue(for: characteristic)
//        } else {
//            // Handle the case where the peripheral is not connected (e.g., reconnect)
//            connectToPeripheral(peripheral)
//        }
//    }
//
//    func writeToCharacteristic(_ value: String) {
//        guard let peripheral = connectedPeripheral, let characteristic = targetCharacteristic else {
//            print("Peripheral or characteristic is nil. Cannot write.")
//            return
//        }
//
//        if peripheral.state == .connected {
//            if let data = value.data(using: .utf8) {
//                peripheral.writeValue(data, for: characteristic, type: .withResponse)
//            }
//        } else {
//            // Handle the case where the peripheral is not connected (e.g., reconnect)
//            connectToPeripheral(peripheral)
//        }
//    }
//    
//    func decodeAndPrintMessage(from characteristic: CBCharacteristic) {
//        guard let data = characteristic.value, let message = String(data: data, encoding: .utf8) else {
//            print("Failed to decode and print message.")
//            return
//        }
//
//        print("Decoded Message: \(message)")
//    }
//    
//    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
//        if characteristic.uuid == targetCharacteristic?.uuid {
//            decodeAndPrintMessage(from: characteristic)
//        }
//    }
//    
////    func writeToCharacteristic(_ value: String) {
////        guard let peripheral = connectedPeripheral, let characteristic = targetCharacteristic else {
////            print("Peripheral or characteristic is nil. Cannot write.")
////            return
////        }
////
////        if let data = value.data(using: .utf8) {
////            peripheral.writeValue(data, for: characteristic, type: .withResponse)
////        }
////    }
//    
//    
//    
////    func startScanning() {
////         print("startScanning")
////         myCentral.scanForPeripherals(withServices: nil, options: nil)
////     }
////    
////    func stopScanning() {
////        print("stopScanning")
////        myCentral.stopScan()
////    }
//    
//    // Implement characteristic reading and writing here
//    
////    // Function to read from the selected characteristic
////    func readCharacteristic() {
////        if let connectedPeripheral = connectedPeripheral,
////           let characteristic = connectedPeripheral.services?.first(where: { $0.uuid == serviceUUID })?.characteristics?.first(where: { $0.uuid == characteristicUUID }) {
////            connectedPeripheral.readValue(for: characteristic)
////        }
////    }
////
////    // Function to write to the selected characteristic
////    func writeCharacteristic(_ value: Data) {
////        if let connectedPeripheral = connectedPeripheral,
////           let characteristic = connectedPeripheral.services?.first(where: { $0.uuid == serviceUUID })?.characteristics?.first(where: { $0.uuid == characteristicUUID }) {
////            connectedPeripheral.writeValue(value, for: characteristic, type: .withResponse)
////        }
////    }
////    
////    func centralManager(_ central: CBCentralManager, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
////        if let data = characteristic.value {
////            characteristicValue = String(data: data, encoding: .utf8) ?? ""
////        }
////    }
//}
