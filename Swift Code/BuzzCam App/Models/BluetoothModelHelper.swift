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
        // Retrieve the current values of MarkPacket and SystemInfoPacket (if they exist)
        var currentMarkPacket = markPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentMarkPacket.header.systemUid = UInt32(currentTimestamp)
        
        // Set annotationText if it's not empty
        if !annotationText.isEmpty {
            currentMarkPacket.markPacket.annotation = annotationText
        }
        
        currentMarkPacket.markPacket.beepEnabled = beep
        

        // Update markPacket
        markPacket = currentMarkPacket

        sendSystemInfoPacket()
        sendMarkPacket()
    }
    
    // send packet when device is disabled/enabled
    func deviceEnabledUpdates(deviceEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentSystemInfoPacket = systemInfoPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSystemInfoPacket.header.systemUid = UInt32(currentTimestamp)
        
        if (currentSystemInfoPacket.systemInfoPacket.deviceRecording != deviceEnabled) {
            currentSystemInfoPacket.systemInfoPacket.deviceRecording = deviceEnabled
        }
        
        systemInfoPacket = currentSystemInfoPacket
        
        sendSystemInfoPacket()
    }
    
    // send packet to force a camera capture
    func forceCameraCapture() {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        currentConfigPacket.configPacket.cameraControl.capture = true
        
        configPacket = currentConfigPacket
        
        print("Forced camera capture")
        
        sendConfigPacket()
    }
    
    //send packet to enable/disable audio channels
    func enableAudioChannel1(channel1: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        if (currentConfigPacket.configPacket.audioConfig.channel1 != channel1) {
            // edit field if changed
            currentConfigPacket.configPacket.audioConfig.channel1 = channel1
            
            configPacket = currentConfigPacket
            
            print("in enableAudioChannel1, channel1 is \(channel1)")
            print("Sent enable audio channel1")
            
            sendConfigPacket()
        }
    }
    
    //send packet to enable/disable audio channels
    func enableAudioChannel2(channel2: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        // also add epoch
        
        // edit field if changed
        if (currentConfigPacket.configPacket.audioConfig.channel2 != channel2) {
            print("in if")
            
            currentConfigPacket.configPacket.audioConfig.channel2 = channel2
            
            configPacket = currentConfigPacket
            
            print("2 in enableAudioChannel2, channel2 is \(channel2)")
            print("Sent enable audio channel2")
            
            sendConfigPacket()
        }
    }
    
    //send packet to set sample freq
    func changeSampleFreq(sampleFreq: MicSampleFreq) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.audioConfig.sampleFreq != sampleFreq) {
            currentConfigPacket.configPacket.audioConfig.sampleFreq = sampleFreq
            
            configPacket = currentConfigPacket
            
            print("Sent new sample freq, \(sampleFreq)")
            
            sendConfigPacket()
        }
    }
    
    func setBitResolution(bitResolution: MicBitResolution) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if(currentConfigPacket.configPacket.audioConfig.bitResolution != bitResolution) {
            currentConfigPacket.configPacket.audioConfig.bitResolution = bitResolution
            
            configPacket = currentConfigPacket
            
            print("Set bit resolution")
            
            sendConfigPacket()
        }
    }
    
    //send packet to enable/disable audio channels
    func enableAudioCompression(audioCompressionEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.audioConfig.audioCompression.enabled != audioCompressionEnabled) {
            
            currentConfigPacket.configPacket.audioConfig.audioCompression.enabled = audioCompressionEnabled
            
            configPacket = currentConfigPacket
                        
            print("Sent enable audio compression")
            
            sendConfigPacket()
        }
    }
    
    //send packet to set compression type
    func changeCompressionType(compressionType: CompressionType) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.audioConfig.audioCompression.compressionType != compressionType) {
            currentConfigPacket.configPacket.audioConfig.audioCompression.compressionType = compressionType
            
            configPacket = currentConfigPacket
            
            print("Sent new compression type", compressionType)
            
            sendConfigPacket()
        }
    }
    
    // send packet to change compression factor
    func changeCompressionFactor(compressionFactor: UInt32) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.audioConfig.audioCompression.compressionFactor != compressionFactor) {
            currentConfigPacket.configPacket.audioConfig.audioCompression.compressionFactor = compressionFactor
            
            configPacket = currentConfigPacket
            
            print("Sent new compression factor")
            
            sendConfigPacket()
        }
    }
    
    // send [packet to change sample period
    func changeSamplePeriod(samplePeriod: UInt32) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.sensorConfig.samplePeriodMs != samplePeriod) {
            currentConfigPacket.configPacket.sensorConfig.samplePeriodMs = samplePeriod
            
            configPacket = currentConfigPacket
            
            print("Sent new samplePeriod")
            
            sendConfigPacket()
        }
    }
    
    //send packet to enable/disable gas
    func enableGasSensing(enableGas: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.sensorConfig.enableGas != enableGas) {
            
            currentConfigPacket.configPacket.sensorConfig.enableGas = enableGas
            
            configPacket = currentConfigPacket
            
            print("Sent enableGas")
            
            sendConfigPacket()
        }
    }
    
    //send packet to enable/disable temperature
    func enableTemperatureSensing(enableTemperature: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.sensorConfig.enableTemperature != enableTemperature) {
            
            currentConfigPacket.configPacket.sensorConfig.enableTemperature = enableTemperature
            
            configPacket = currentConfigPacket
            
            print("Sent enableTemperature")
            
            sendConfigPacket()
        }
    }
    
    //send packet to enable/disable humidity
    func enableHumiditySensing(enableHumidity: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        // edit field if changed
        if (currentConfigPacket.configPacket.sensorConfig.enableHumidity != enableHumidity) {
            
            currentConfigPacket.configPacket.sensorConfig.enableHumidity = enableHumidity
            
            configPacket = currentConfigPacket
            
            print("Sent enableHumidity")
            
            sendConfigPacket()
        }
    }
    
    // camera functions
    
    // send packet to pair with nearby cameras
    func pairWithNearbyCameras() {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        currentConfigPacket.configPacket.cameraControl.pairWithNearbyCameras = true
        
        
        configPacket = currentConfigPacket
        
        print("Sent pairWithNearbyCameras")
        
        sendConfigPacket()
    }
    
    // send packet to wakeup cameras
    func wakeupCameras() {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        currentConfigPacket.configPacket.cameraControl.wakeupCameras = true
        
        configPacket = currentConfigPacket
        
        print("Sent wakeupCameras")
        
        sendConfigPacket()
    }
    
    // send packet to force rediscovery
    func forceRediscovery() {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        currentConfigPacket.configPacket.networkState.forceRediscovery = true
        
        configPacket = currentConfigPacket
        
        print("Sent forceRediscovery")
        
        sendConfigPacket()
    }
    
    // functions for schedule
    
    // send packet with schedules
    func sendSchedules(_ schedules: [ScheduleConfig]) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.systemUid = UInt32(currentTimestamp)
        
        currentConfigPacket.configPacket.scheduleConfig = schedules
        
        configPacket = currentConfigPacket
        
        print("Sent schedules")
        print("Schedules: \(schedules)")
        
        sendConfigPacket()
    }
    
}
