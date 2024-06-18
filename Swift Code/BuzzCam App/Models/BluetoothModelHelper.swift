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
        var currentMarkPacket = Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentMarkPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // Set annotationText if it's not empty
        if !annotationText.isEmpty {
            currentMarkPacket.markPacket.annotation = annotationText
        }
        
        currentMarkPacket.markPacket.beepEnabled = beep
        
        // Update markPacket
        markPacket = currentMarkPacket
        
        print("sent markPacket")

        sendMarkPacket()
        
        markPacket.markPacket.clearAnnotation() //clear annotation and reset hasAnnotation
    }
    
    // send packet when device is disabled/enabled
    func deviceEnabledUpdates(deviceEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
//        if (currentConfigPacket.configPacket.enableRecording != deviceEnabled) {
            currentConfigPacket.configPacket.enableRecording = deviceEnabled
            
            configPacket = currentConfigPacket
            
            print("currentTimestamp",currentConfigPacket.header.epoch)
            
            sendConfigPacket()
//        }
    }
    
    // send packet to force a camera capture
    func forceCameraCapture() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.cameraControl.capture = true
        
        specialFunction = currentSpecialFunction
                
        sendSpecialFunction()
    }
    
    //send packet to enable/disable audio channels
    func enableAudioChannel1(channel1: Bool) {
        print("called changeChannel1")

        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        print("currentConfigPacket.configPacket.audioConfig.channel1", currentConfigPacket.configPacket.audioConfig.channel1)
        print("channel1", channel1)
        

        currentConfigPacket.configPacket.audioConfig.channel1 = channel1
        
        configPacket = currentConfigPacket
        
        print("in enableAudioChannel1, channel1 is \(channel1)")
        print("Sent enable audio channel1")
        
        sendConfigPacket()
    }
    
    //send packet to enable/disable audio channels
    func enableAudioChannel2(channel2: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        // also add epoch
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.channel2 != channel2) {
            print("in if")
            
            currentConfigPacket.configPacket.audioConfig.channel2 = channel2
            
            configPacket = currentConfigPacket
            
            print("2 in enableAudioChannel2, channel2 is \(channel2)")
            print("Sent enable audio channel2")
            
            sendConfigPacket()
//        }
    }
    
    //send packet to set sample freq
    func changeSampleFreq(sampleFreq: MicSampleFreq) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.sampleFreq != sampleFreq) {
            currentConfigPacket.configPacket.audioConfig.sampleFreq = sampleFreq
            
            configPacket = currentConfigPacket
            
            print("Sent new sample freq, \(sampleFreq)")
            
            sendConfigPacket()
//        }
    }
    
    func setBitResolution(bitResolution: MicBitResolution) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if(currentConfigPacket.configPacket.audioConfig.bitResolution != bitResolution) {
            currentConfigPacket.configPacket.audioConfig.bitResolution = bitResolution
            
            configPacket = currentConfigPacket
            
            print("Set bit resolution in helper", currentConfigPacket.configPacket.audioConfig.bitResolution)
            
            sendConfigPacket()
//        }
    }
    
    //send packet to enable/disable audio channels
    func enableAudioCompression(audioCompressionEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.audioCompression.enabled != audioCompressionEnabled) {
            
            currentConfigPacket.configPacket.audioConfig.audioCompression.enabled = audioCompressionEnabled
            
            configPacket = currentConfigPacket
                        
            print("Sent enable audio compression")
            
            sendConfigPacket()
//        }
    }
    
    //send packet to set compression type
    func changeCompressionType(compressionType: CompressionType) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.audioCompression.compressionType != compressionType) {
            currentConfigPacket.configPacket.audioConfig.audioCompression.compressionType = compressionType
            
            configPacket = currentConfigPacket
            
            print("Sent new compression type", compressionType)
            
            sendConfigPacket()
//        }
    }
    
    // send packet to change compression factor
    func changeCompressionFactor(compressionFactor: UInt32) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.audioCompression.compressionFactor != compressionFactor) {
            currentConfigPacket.configPacket.audioConfig.audioCompression.compressionFactor = compressionFactor
            
            configPacket = currentConfigPacket
            
            print("Sent new compression factor")
            
            sendConfigPacket()
//        }
    }
    
    //send packet to set compression type
    func changeMicGain(micGain: MicGain) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.micGain != micGain) {
            currentConfigPacket.configPacket.audioConfig.micGain = micGain
            
            configPacket = currentConfigPacket
            
            print("Sent new micGain", micGain)
            
            sendConfigPacket()
//        }
    }
    
    //send packet to enable/disable free run mode
    func enableFreeRunMode(enableFreeRunMode: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.freeRunMode != enableFreeRunMode) {
            
            currentConfigPacket.configPacket.audioConfig.freeRunMode = enableFreeRunMode
            
            configPacket = currentConfigPacket
            
            print("Sent enableFreeRunMode, ", enableFreeRunMode)
            
            sendConfigPacket()
//        }
    }
    
    
    //send packet to enable/disable gas
    func enableGasSensing(enableGas: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.sensorConfig.enableGas != enableGas) {
            
            currentConfigPacket.configPacket.sensorConfig.enableGas = enableGas
            
            configPacket = currentConfigPacket
            
            print("Sent enableGas")
            
            sendConfigPacket()
//        }
    }
    
    //send packet to enable/disable temperature
    func enableTemperatureSensing(enableTemperature: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.sensorConfig.enableTemperature != enableTemperature) {
            
            currentConfigPacket.configPacket.sensorConfig.enableTemperature = enableTemperature
            
            configPacket = currentConfigPacket
            
            print("Sent enableTemperature")
            
            sendConfigPacket()
//        }
    }
    
    //send packet to enable/disable humidity
    func enableHumiditySensing(enableHumidity: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.sensorConfig.enableHumidity != enableHumidity) {
            
            currentConfigPacket.configPacket.sensorConfig.enableHumidity = enableHumidity
            
            configPacket = currentConfigPacket
            
            print("Sent enableHumidity")
            
            sendConfigPacket()
//        }
    }
    
    // camera functions
    
    // send packet to pair with nearby cameras
    func pairWithNearbyCameras() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.cameraControl.pairWithNearbyCameras = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent pairWithNearbyCameras")
        
        sendSpecialFunction()
    }
    
    // send packet to wakeup cameras
    func wakeupCameras() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentConfigPacket.specialFunction.cameraControl.wakeupCameras = true
        
        configPacket = currentConfigPacket
        
        print("Sent wakeupCameras")
        
        sendConfigPacket()
    }
    
    
    // send packet with schedules
    func sendSchedules(_ schedules: [ScheduleConfig]) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentConfigPacket.configPacket.scheduleConfig = schedules
        
        configPacket = currentConfigPacket
        
        print("Sent schedules")
        print("Schedules: \(schedules)")
        
        sendConfigPacket()
    }
    
    // functions for system control dropdown
    
    // send packet to format SD card
    func formatSDCard() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.formatSdcard = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent formatSDCard")
        
        sendSpecialFunction()
    }
    
    // send packet to open thread sync time
    func openThreadSyncTime() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.openthreadSyncTime = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent openThreadSyncTime")
        
        sendSpecialFunction()
    }
    
    // send packet to calibrate magnetometer
    func magCalibration() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.magCalibration = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent magCalibration")
        
        sendSpecialFunction()
    }
    
    // send packet to trigger DFU Mode
    func triggerDFUMode() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.dfuMode = true

        
        specialFunction = currentSpecialFunction
        
        print("Sent triggerDFUMode")
        
        sendSpecialFunction()
    }
    
    // send packet to reset config
    func resetConfig() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.resetConfig = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent resetConfig")
        
        sendSpecialFunction()
    }
    
    //send packet to enable/disable audio channels
    func enableLowPowerMode(lowPowerModeEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.lowPowerConfig.lowPowerMode != lowPowerModeEnabled) {
            
            currentConfigPacket.configPacket.lowPowerConfig.lowPowerMode = lowPowerModeEnabled
            
            configPacket = currentConfigPacket
                        
            print("Sent enableLowPowerMode")
            
            sendConfigPacket()
//        }
    }
    
    // network state toggles
    
    //send packet to enable/disable master node
    func enableMasterNode(masterNodeEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.networkState.masterNode != masterNodeEnabled) {
            
            currentConfigPacket.configPacket.networkState.masterNode = masterNodeEnabled
            
            configPacket = currentConfigPacket
            
            print("Sent enableMasterNode, ", masterNodeEnabled)
            
            sendConfigPacket()
//        }
    }
        
    //send packet to enable/disable slave sync
    func enableSlaveSync(slaveSyncEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.networkState.slaveSync != slaveSyncEnabled) {
            
            currentConfigPacket.configPacket.networkState.slaveSync = slaveSyncEnabled
            
            configPacket = currentConfigPacket
            
            print("Sent enableSlaveSync, ", slaveSyncEnabled)
            
            sendConfigPacket()
//        }
    }
    
    //send packet to enable/disable master chirp
    func enableMasterChirp(masterChirpEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.audioConfig.chirpEnable != masterChirpEnabled) {
            
            currentConfigPacket.configPacket.audioConfig.chirpEnable = masterChirpEnabled
            
            configPacket = currentConfigPacket
            
            print("Sent enableMasterChirp, ", masterChirpEnabled)
            
            sendConfigPacket()
//        }
    }
    
    // send packet with PAN ID
    func sendPanID(panID: UInt32) {
        // Retrieve the current values of MarkPacket and SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        // edit field if changed
//        if (currentConfigPacket.configPacket.networkState.panID != panID) {
            
            currentConfigPacket.configPacket.networkState.panID = panID
            
            configPacket = currentConfigPacket
            
            print("Sent sendPanID", panID)
            
            sendConfigPacket()
//        }
        
    }
    
    // send packet for new channel
    func changeChannel(channel: UInt32) {
        // Retrieve the current values of configPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000

        // edit field if changed
//        if (currentConfigPacket.configPacket.networkState.channel != channel) {
            currentConfigPacket.configPacket.networkState.channel = channel

            configPacket = currentConfigPacket

            print("Sent new channel, ", channel)

            sendConfigPacket()
//        }
    }
    
    // send packet to start ranging
    func startRanging() {
        // Retrieve the current values of SpecialFunction (if they exist)
        var currentSpecialFunction = Packet()
        
        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentSpecialFunction.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentSpecialFunction.specialFunction.uwbPacket.startRanging = true
        
        
        specialFunction = currentSpecialFunction
        
        print("Sent startRanging")
        
        sendSpecialFunction()
    }
    
    // send packet to enable/disable led
    func enableLed(ledEnabled: Bool) {
        // Retrieve the current values of SystemInfoPacket (if they exist)
        var currentConfigPacket = configPacket ?? Packet()

        // Set unix time
        let currentTimestamp = Date().timeIntervalSince1970
        currentConfigPacket.header.epoch = UInt64(currentTimestamp) * 1000
        
        currentConfigPacket.configPacket.enableLed = ledEnabled
        
        configPacket = currentConfigPacket
                    
        sendConfigPacket()
        print("sent enableLed,", ledEnabled)
    }
    
}
