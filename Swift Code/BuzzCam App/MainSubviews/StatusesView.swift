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
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Statuses")
                    .font(customFontTitle)
                    .foregroundColor(Color.white)
                    .shadow(color: .black, radius: 5, x: 0, y: 2)
                    .padding()
                
                Image(systemName: "chevron.down.circle.fill")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                    .shadow(color: .black, radius: 5, x: 0, y: 2)
                    .foregroundColor(Color.white)
                Spacer()
            }
            .frame(maxWidth: .infinity)
            .background(
                GeometryReader { proxy in
                        Image("flowers 5")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }
            )
                .onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                }
            }
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    VStack (alignment: .leading) {
                        HStack {
                            Text("Device enabled").font(customFontTextBold)
                                .fontWeight(.bold).padding(.horizontal)
                            
                            Toggle("",isOn: $deviceEnabled).labelsHidden().onChange(of: deviceEnabled) {
                                // Call your function when the toggle is changed
                                bluetoothModel.deviceEnabledUpdates(deviceEnabled: deviceEnabled)
                            }.tint(Color(red: 117/255, green: 13/255, blue: 55/255, opacity: 0.5))
                        }
                        
                        Text("Needs to be activated for the recorder to run. However, this will be disable in slave mode").font(customFontText).padding(.leading)
                    }
                    
                    
                    VStack(alignment: .leading) {
                        Text("SD Card Status")
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                        VStack (alignment: .leading, spacing: 10){
                            Text("Detected: " + String(bluetoothModel.systemInfoPacketData?.sd_detected ?? false))
                                .font(customFontText)
                            Text("Space remaining: " + String(bluetoothModel.systemInfoPacketData?.space_remaining ?? 0))
                                .font(customFontText)
                            Text("Estimated recording time: " + String(bluetoothModel.systemInfoPacketData?.estimated_recording_time ?? 0))
                                .font(customFontText)
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
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                        VStack (alignment: .leading, spacing: 10){
                            Text("Is charging: " + String(bluetoothModel.systemInfoPacketData?.battery_charging ?? false))
                                .font(customFontText)
                            Text("Battery Voltage: " +  String(bluetoothModel.systemInfoPacketData?.battery_voltage ?? 0))
                                .font(customFontText)
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
        }.onAppear {
            // Add an observer to monitor changes to systemInfoPacketData
            bluetoothModel.$configPacketData
                .sink { configPacketData in
                    // Update deviceEnabled when systemInfoPacketData changes
                    self.updateDeviceEnabledOn(configPacketData)
                }
                .store(in: &cancellables) // Store the cancellable to avoid memory leaks
            
            // Trigger the initial update
            self.updateDeviceEnabledOn(bluetoothModel.configPacketData)
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    private func updateDeviceEnabledOn(_ configPacketData: ConfigPacketData?) {
        // Update deviceEnabled based on systemInfoPacketData
        deviceEnabled = configPacketData?.enableRecording ?? false
    }
}




#Preview {
    StatusesView()
}
