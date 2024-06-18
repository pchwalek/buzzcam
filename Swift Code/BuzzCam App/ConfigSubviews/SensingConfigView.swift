//
//  SensingConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/6/23.
//

import SwiftUI
import Combine

struct SensingConfigView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    @State var selectedSamplePeriod: UInt32 = 0
    @State private var enableTemperature = false
    @State private var enableGas = false
    @State private var enableHumidity = false
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Sensings")
                    .font(customFontTextBoldLarge)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(
                GeometryReader { proxy in
                        Image("IMG_4587 (2)")
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
//                    VStack(alignment: .leading) {
//                        Text("Sensor Sample Period: \(Int(selectedSamplePeriod))").font(.title3).fontWeight(.bold)
//                        Slider(value: Binding(
//                            get: {
//                                Double(selectedSamplePeriod)
//                            },
//                            set: { newValue in
//                                selectedSamplePeriod = UInt32(newValue)
//                                // This code will be executed when the user starts dragging
//                            }
//                        ), in: 1...300, step: 1, onEditingChanged: { editingChanged in
//                            if !editingChanged {
//                                // This code will be executed when the user finishes dragging
//                                bluetoothModel.changeSamplePeriod(samplePeriod: UInt32(selectedSamplePeriod))
//                            }
//                        })
//                        .padding()
//                        Text("1") // Display the left end value
//                            .frame(maxWidth: .infinity, alignment: .leading)
//                            .padding(.horizontal)
//                        
//                        Text("300") // Display the right end value
//                            .frame(maxWidth: .infinity, alignment: .trailing)
//                            .padding(.horizontal)
//                    }
//                    .padding()
//                    .frame(
//                        minWidth: 0,
//                        maxWidth: .infinity,
//                        alignment: .leading)
//                    .background(Color(white: 0.98))
//                    .cornerRadius(10)
                    
                    VStack(alignment: .leading) {
                        HStack {
                            Text("Enable temperature sensing")
                                .font(customFontTextBoldSmall)
                                .padding()
                            
                            Toggle("", isOn: $enableTemperature)
                                .labelsHidden()
                                .onChange(of: enableTemperature) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableTemperatureSensing(enableTemperature: enableTemperature)
                                }
                        }
                        
                        HStack {
                            Text("Enable humidity sensing")
                                .font(customFontTextBoldSmall)
                                .padding()
                            
                            Toggle("", isOn: $enableHumidity)
                                .labelsHidden()
                                .onChange(of: enableHumidity) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableHumiditySensing(enableHumidity: enableHumidity)
                                }
                        }
                        
                        HStack {
                            Text("Enable gas sensing")
                                .font(customFontTextBoldSmall)
                                .padding()
                            
                            Toggle("", isOn: $enableGas)
                                .labelsHidden()
                                .onChange(of: enableGas) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableGasSensing(enableGas: enableGas)
                                }
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
        }.onAppear {
            // Set the initial value of selectedSampleFreq based on the stored value in bluetoothModel
            print("initialized sensingconfigview")
            selectedSamplePeriod = bluetoothModel.configPacketData_Sensor?.samplePeriodMs ?? 0
            
            // Add an observer to monitor changes to configPacketData_Sensor
            bluetoothModel.$configPacketData_Sensor
                .sink { configPacketData_Sensor in
                    self.updateHumiditySensing(configPacketData_Sensor)
                    self.updateTemperatureSensing(configPacketData_Sensor)
                    self.updateGasSensing(configPacketData_Sensor)
                }
                .store(in: &cancellables)
            
            // Trigger the initial update
            self.updateHumiditySensing(bluetoothModel.configPacketData_Sensor)
            self.updateTemperatureSensing(bluetoothModel.configPacketData_Sensor)
            self.updateGasSensing(bluetoothModel.configPacketData_Sensor)
            
            if let initialSamplePeriod = bluetoothModel.configPacketData_Sensor?.samplePeriodMs {
                selectedSamplePeriod = UInt32(initialSamplePeriod)
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    
    private func updateHumiditySensing(_ configPacketData_Sensor: ConfigPacketData_Sensor?) {
        // Update channel1 based on configPacketData_Audio
        guard let configData = configPacketData_Sensor, enableHumidity != configData.enableHumidity else {
            return
        }
        enableHumidity = configData.enableHumidity
    }
    
    private func updateGasSensing(_ configPacketData_Sensor: ConfigPacketData_Sensor?) {
        // Update channel2 based on configPacketData_Audio
        guard let configData = configPacketData_Sensor, enableGas != configData.enableGas else {
            return
        }
        enableGas = configData.enableGas
    }
    
    private func updateTemperatureSensing(_ configPacketData_Sensor: ConfigPacketData_Sensor?) {
        // Update channel2 based on configPacketData_Audio
        guard let configData = configPacketData_Sensor, enableTemperature != configData.enableTemperature else {
            return
        }
        enableTemperature = configData.enableTemperature
    }
}

#Preview {
    SensingConfigView()
}
