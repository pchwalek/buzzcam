//
//  BluetoothConstants.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/27/23.
//

import Foundation
import CoreBluetooth

enum BluetoothConstants {
    
}

struct SystemInfoPacketData {
    var index: UInt32
    var temperature: Float
    var humidity: Float
    var co2: Float
    var light_level: Float
    var sd_detected: Bool
    var space_remaining: UInt64
    var estimated_recording_time: UInt64
    var battery_charging: Bool
    var battery_voltage: Float
    var device_recording: Bool
    var mark_number: UInt32
    var beep_enabled: Bool
    
    mutating func reset() {
        // set property default values
        index = 0
        temperature = 0.0
        humidity = 0.0
        co2 = 0.0
        light_level = 0.0
        sd_detected = false
        space_remaining = 0
        estimated_recording_time = 0
        battery_charging = false
        battery_voltage = 0.0
        device_recording = false
        mark_number = 0
        beep_enabled = false
    }
}

// config data
struct ConfigPacketData_Audio {
    var channel1: Bool
    var channel2: Bool
    var sampleFreq: MicSampleFreq
    var bitResolution: MicBitResolution
    var audioCompressionEnabled: Bool
    var audioCompressionType: CompressionType
    var audioCompressionFactor: UInt32
    var estimatedRecordTime: Float
}


