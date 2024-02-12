//
//  StatusesView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI
import Combine

struct StatusesView: View {
    @State private var isExpanded = false
    @State private var deviceEnabled = true
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var cancellables: Set<AnyCancellable> = Set()
    
    let customFontTitle = Font.custom("Futura-Bold", size: 20) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Statuses")
                    .font(customFontTitle)
                    .foregroundColor(Color.white)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(Color(white:0.75)).onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                }
            }
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    HStack {
                        Text("Device enabled").font(.title2)
                            .fontWeight(.bold).padding()
                        Toggle("",isOn: $deviceEnabled).labelsHidden().onAppear {
                            // Add an observer to monitor changes to systemInfoPacketData
                            bluetoothModel.$systemInfoPacketData
                                .sink { systemInfoPacketData in
                                    // Update deviceEnabled when systemInfoPacketData changes
                                    self.updateDeviceEnabledOn(systemInfoPacketData)
                                }
                                .store(in: &cancellables) // Store the cancellable to avoid memory leaks
                            
                            // Trigger the initial update
                            self.updateDeviceEnabledOn(bluetoothModel.systemInfoPacketData)
                        }.onChange(of: deviceEnabled) {
                            // Call your function when the toggle is changed
                            bluetoothModel.deviceEnabledUpdates(deviceEnabled: deviceEnabled)
                        }
                    }
                    
                    
                    VStack(alignment: .leading) {
                        Text("SD Card Status")
                            .font(.title2)
                            .fontWeight(.bold)
                        VStack (alignment: .leading, spacing: 10){
                            Text("Detected: " + String(bluetoothModel.systemInfoPacketData?.sd_detected ?? false))
                                .font(.body)
                            Text("Space remaining: " + String(bluetoothModel.systemInfoPacketData?.space_remaining ?? 0))
                                .font(.body)
                            Text("Estimated recording time: " + String(bluetoothModel.systemInfoPacketData?.estimated_recording_time ?? 0))
                                .font(.body)
                        }
                        .padding()
                    }
                    .padding()
                    .frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                    
                    
                    
                    VStack(alignment: .leading) {
                        Text("Battery Status")
                            .font(.title2)
                            .fontWeight(.bold)
                        VStack (alignment: .leading, spacing: 10){
                            Text("Is charging: " + String(bluetoothModel.systemInfoPacketData?.battery_charging ?? false))
                                .font(.body)
                            Text("Battery Voltage: " +  String(bluetoothModel.systemInfoPacketData?.battery_voltage ?? 0))
                                .font(.body)
                        }
                        .padding()
                    }
                    .padding()
                    .frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                    
                    
                }
                .padding()
                
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    private func updateDeviceEnabledOn(_ systemInfoPacketData: SystemInfoPacketData?) {
        // Update deviceEnabled based on systemInfoPacketData
        deviceEnabled = systemInfoPacketData?.device_recording ?? false
    }
}




#Preview {
    StatusesView()
}
