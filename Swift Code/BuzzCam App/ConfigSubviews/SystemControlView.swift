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
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("System Control")
                    .font(customFontTextBoldLarge)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(
                GeometryReader { proxy in
                        Image("IMG_4587 (5)")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }).onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                }
            }
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    VStack(alignment: .leading) {
                        VStack(alignment: .leading) {
                            HStack {
                                Text("Lower power mode").font(customFontTextBoldSmall)
                                Toggle("",isOn: $lowPowerModeEnabled).labelsHidden()
                                    .onChange(of: lowPowerModeEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableLowPowerMode(lowPowerModeEnabled: lowPowerModeEnabled)
                                    }.padding()
                            }
                            
                            HStack {
                                Text("Enabled led").font(customFontTextBoldSmall)
                                
                                Toggle("",isOn: $ledEnabled).labelsHidden()
                                    .onChange(of: ledEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableLed(ledEnabled: ledEnabled)
                                    }.padding()
                            }
                            
                            HStack {
                                Text("Reset config")
                                    .font(customFontText)
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
                                    .font(customFontText)
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
                                    .font(customFontText)
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
                                    .font(customFontText)
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
                                    .font(customFontText)
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
