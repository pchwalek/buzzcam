//
//  BluetoothModelHelper.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 11/10/23.
//

// Stores functionality to update packets

import CoreBluetooth
import Foundation
import SwiftUI
import SwiftProtobuf


extension BluetoothModel {
    
    // send packets when mark is updated
    func markUpdates(annotationText: String, beep: Bool) {
        // Create new Packet

        // Retrieve the current values of MarkPacket and SystemInfoPacket (if they exist)
        var currentMarkPacket = markPacket ?? Packet()
        var currentSystemInfoPacket = systemInfoPacket ?? Packet()

        // Set header to true
        currentMarkPacket.header = PacketHeader()
        print("has header \(currentMarkPacket.header)" )
        currentSystemInfoPacket.header = PacketHeader()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentMarkPacket.header.systemUid = UInt32(currentTimestamp)
        currentSystemInfoPacket.header.systemUid = UInt32(currentTimestamp)

        // Set the currentMarkPacket fields
        currentMarkPacket.payload = .markPacket(MarkPacket())
        currentSystemInfoPacket.payload = .systemInfoPacket(SystemInfoPacket())

        // Make changes to the specific fields
        // Set annotationText if it's not empty
        if !annotationText.isEmpty {
            currentMarkPacket.markPacket.annotation = annotationText
        }
        if (currentSystemInfoPacket.systemInfoPacket.markState.beepEnabled != beep) {
            currentSystemInfoPacket.systemInfoPacket.markState.beepEnabled = beep
        }
        
        // remove this later
        currentSystemInfoPacket.systemInfoPacket.simpleSensorReading.co2 = 10
        
        // Update markPacket and systemInfoPacket
        markPacket = currentMarkPacket
        systemInfoPacket = currentSystemInfoPacket
        
        print("markPacket.markPacket.annotation", markPacket?.markPacket.annotation)
        print("systemInfoPacket.systemInfoPacket.markState.beepEnabled", systemInfoPacket?.systemInfoPacket.markState.beepEnabled)

        // Send both packets over BLE
        // sendMarkPacket()
        // usleep(100000)
        sendSystemInfoPacket()
        sendMarkPacket()
    }
    
    // send packet when device is disabled/enabled
    func deviceEnabledUpdates(deviceEnabled: Bool) {
        // Create new Packet

        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentSystemInfoPacket = systemInfoPacket ?? Packet()

        // Set header to true
        currentSystemInfoPacket.header = PacketHeader()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSystemInfoPacket.header.systemUid = UInt32(currentTimestamp)

        // Set the currentMarkPacket fields
        currentSystemInfoPacket.payload = .systemInfoPacket(SystemInfoPacket())

        if (currentSystemInfoPacket.systemInfoPacket.deviceRecording != deviceEnabled) {
            currentSystemInfoPacket.systemInfoPacket.deviceRecording = deviceEnabled
        }
        
        // Update markPacket and systemInfoPacket
        systemInfoPacket = currentSystemInfoPacket
        
        sendSystemInfoPacket()
    }
    
    // send packet to force a camera capture
    func forceCameraCapture() {
        // Create new Packet

        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set header to true
        currentConfigPacket.header = PacketHeader()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)

        // Set the currentMarkPacket fields
        currentConfigPacket.payload = .configPacket(ConfigPacket())

        if (!currentConfigPacket.configPacket.cameraControl.capture) {
            currentConfigPacket.configPacket.cameraControl.capture = true
        }
        
        // Update markPacket and systemInfoPacket
        configPacket = currentConfigPacket
        
        print("Forced camera capture")
        
        sendConfigPacket()
    }
    
}
