//
//  CameraConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/11/23.
//

import SwiftUI
import Combine

struct SystemControlView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    @State private var lowPowerModeEnabled = false
    @State private var ledEnabled = false
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("System Control")
                    .font(.title)
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
                    VStack(alignment: .leading) {
                        VStack(alignment: .leading) {
                            HStack {
                                Text("Lower power mode").fontWeight(.bold)
                                Toggle("",isOn: $lowPowerModeEnabled).labelsHidden()
                                    .onChange(of: lowPowerModeEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableLowPowerMode(lowPowerModeEnabled: lowPowerModeEnabled)
                                    }.padding()
                            }
                            
                            HStack {
                                Text("Enabled led").fontWeight(.bold)
                                
                                Toggle("",isOn: $ledEnabled).labelsHidden()
                                    .onChange(of: ledEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableLed(ledEnabled: ledEnabled)
                                    }.padding()
                            }
                            
                            HStack {
                                Text("Reset config")
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.resetConfig()
                                }) {
                                    Image(systemName: "rays").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Format SD card")
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.pairWithNearbyCameras()
                                }) {
                                    Image(systemName: "sdcard").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Open thread sync time")
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.openThreadSyncTime()
                                }) {
                                    Image(systemName: "clock.arrow.2.circlepath").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Magnetometer calibration")
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.magCalibration()
                                }) {
                                    Image(systemName: "location.north.line").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Trigger DFU Mode")
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.triggerDFUMode()
                                }) {
                                    Image(systemName: "cursorarrow.click").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            
                        }
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
        .onAppear{
            bluetoothModel.$configPacketData_LowPower
                .sink { configPacketData_LowPower in
                    // Update beepOn when systemInfoPacketData changes
                    self.updateLowPowerModeToggle(configPacketData_LowPower)
                }
                .store(in: &cancellables) // Store the cancellable to avoid memory leaks
            
            // Trigger the initial update
            self.updateLowPowerModeToggle(bluetoothModel.configPacketData_LowPower)
            
            bluetoothModel.$configPacketData
                .sink { configPacketData in
                    // Update beepOn when systemInfoPacketData changes
                    self.updateLedToggle(configPacketData)
                }
                .store(in: &cancellables) // Store the cancellable to avoid memory leaks
            
            // Trigger the initial update
            self.updateLedToggle(bluetoothModel.configPacketData)
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
        
    }
    
    private func updateLowPowerModeToggle(_ configPacketData_LowPower: ConfigPacketData_LowPower?) {
        // Update channel2 based on configPacketData_Audio
        lowPowerModeEnabled = configPacketData_LowPower?.lowPowerMode ?? false
    }
    
    private func updateLedToggle(_ configPacketData: ConfigPacketData?) {
        // Update channel2 based on configPacketData_Audio
        ledEnabled = configPacketData?.enableLed ?? false
    }
    
}

#Preview {
    CameraConfigView()
}
