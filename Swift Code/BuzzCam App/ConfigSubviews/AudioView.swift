//
//  AudioView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 11/30/23.
//

import SwiftUI
import Combine

struct AudioView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    @State private var channel1 = false
    @State private var channel2 = false
    let sampleFreq: [MicSampleFreq] = [.sampleRate16000, .sampleRate20500, .sampleRate44100, .sampleRate48000, .sampleRate96000]
    @State var selectedSampleFreq: MicSampleFreq? = .sampleRate16000
    @State var selectedBitResolution: MicBitResolution? = .bitRes8
    
    var body: some View {
        VStack (alignment: .leading) {
                HStack {
                    Spacer()
                    Text("Audio")
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
                        HStack {
                            Text("Enable channel 1").font(.title2)
                                .fontWeight(.bold).padding()
                            Toggle("",isOn: $channel1).labelsHidden().onAppear {
                                // Add an observer to monitor changes to systemInfoPacketData
                                bluetoothModel.$configPacketData_Audio
                                    .sink { configPacketData_Audio in
                                        // Update beepOn when systemInfoPacketData changes
                                        self.updateChannel1(configPacketData_Audio)
                                    }
                                    .store(in: &cancellables) // Store the cancellable to avoid memory leaks

                                // Trigger the initial update
                                self.updateChannel1(bluetoothModel.configPacketData_Audio)
                            }.onChange(of: channel1) {
                                // Call your function when the toggle is changed
                                print("calling enableAudioChannel1, channel1 is \(channel1)")
                                bluetoothModel.enableAudioChannel1(channel1: channel1)
                            }
                            
                        }
                        HStack {
                            Text("Enable channel 2").font(.title2)
                                .fontWeight(.bold).padding()
                            Toggle("",isOn: $channel2).labelsHidden().onAppear {
                                // Add an observer to monitor changes to systemInfoPacketData
                                bluetoothModel.$configPacketData_Audio
                                    .sink { configPacketData_Audio in
                                        // Update beepOn when systemInfoPacketData changes
                                        self.updateChannel2(configPacketData_Audio)
                                    }
                                    .store(in: &cancellables) // Store the cancellable to avoid memory leaks

                                // Trigger the initial update
                                self.updateChannel2(bluetoothModel.configPacketData_Audio)
                            }.onChange(of: channel2) {
                                // Call your function when the toggle is changed
                                print("calling enableAudioChannel2, channel2 is \(channel2)")
                                bluetoothModel.enableAudioChannel2(channel2: channel2)
                            }
                            
                        }
                        
                        
                        
                        VStack(alignment: .leading) {
                            
                            Text("Sampling Frequency")
                                .font(.title2)
                                .fontWeight(.bold)
                            VStack (alignment: .leading, spacing: 10){
//                                List {
//                                            ForEach(sampleFreq, id: \.self) { item in
//                                                FrequencyCell(sampleFreq: item, selectedSampleFreq: self.$selectedSampleFreq)
//                                            }
//                                        }
                                ScrollView {
                                    VStack {
                                        ForEach(sampleFreq, id: \.self) { item in
                                            FrequencyCell(sampleFreq: item, selectedSampleFreq: self.$selectedSampleFreq)
                                        }
                                    }
                                }.onAppear {
                                    // Set the initial value of selectedSampleFreq based on the stored value in bluetoothModel
                                    selectedSampleFreq = bluetoothModel.configPacketData_Audio?.sampleFreq
                                }
                                .onChange(of: selectedSampleFreq ?? .sampleRate16000) { newSelectedSampleFreq in
                                    // Call your function here with the new selected sample frequency
                                    bluetoothModel.changeSampleFreq(sampleFreq: newSelectedSampleFreq)
                                }
//                                Text("Detected: " + String(bluetoothModel.systemInfoPacketData?.sd_detected ?? false))
//                                    .font(.body)
//                                Text("Space remaining: " + String(bluetoothModel.systemInfoPacketData?.space_remaining ?? 0))
//                                    .font(.body)
//                                Text("Estimated recording time: " + String(bluetoothModel.systemInfoPacketData?.estimated_recording_time ?? 0))
//                                    .font(.body)
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
                            Text("Bit Resolution")
                                .font(.title2)
                                .fontWeight(.bold)
                            HStack {
                                Button(action: {
                                    // Handle 8-bit button tap
                                    bluetoothModel.setBitResolution(bitResolution: .bitRes8)
                                }) {
                                    Text("8-bit")
                                        .padding()
                                        .foregroundColor(.white)
                                        .background(bluetoothModel.configPacketData_Audio?.bitResolution == .bitRes8 ? Color.blue : Color.gray)
                                        .cornerRadius(8)
                                }

                                Button(action: {
                                    // Handle 16-bit button tap
                                    bluetoothModel.setBitResolution(bitResolution: .bitRes16)
                                }) {
                                    Text("16-bit")
                                        .padding()
                                        .foregroundColor(.white)
                                        .background(bluetoothModel.configPacketData_Audio?.bitResolution == .bitRes16 ? Color.blue : Color.gray)
                                        .cornerRadius(8)
                                }
                            }
                        }
                        .padding()
                        .frame(minWidth: 0, maxWidth: .infinity, alignment: .leading)
                        .background(Color(white: 0.98))
                        .cornerRadius(10)
                        .onAppear {
                            // Set the initial value of selectedBitResolution based on the stored value in bluetoothModel
                            selectedBitResolution = bluetoothModel.configPacketData_Audio?.bitResolution
                        }
                        
                        Text("Estimated recording time: " + String(bluetoothModel.configPacketData_Audio?.estimatedRecordTime ?? 0))
                            .font(.body)
                        
                    }
                    .padding()
                    
                }
            }
            .frame(maxWidth: .infinity)
            .background(Color(white:0.90))
    }
    private func updateChannel1(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update channel1 based on configPacketData_Audio
        channel1 = configPacketData_Audio?.channel1 ?? false
    }

    private func updateChannel2(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update channel2 based on configPacketData_Audio
        channel2 = configPacketData_Audio?.channel2 ?? false
    }

    }

struct FrequencyCell: View {

    let sampleFreq: MicSampleFreq
    @Binding var selectedSampleFreq: MicSampleFreq?

    var body: some View {
        HStack {
            Text("Frequency: \(sampleFreq.rawValue)")
            Spacer()
            if sampleFreq == selectedSampleFreq {
                Image(systemName: "largecircle.fill.circle")
                    .foregroundColor(.accentColor)
            } else {
                Image(systemName: "circle")
                    .foregroundColor(.secondary)
            }
        }
        .onTapGesture {
            self.selectedSampleFreq = self.sampleFreq
        }
    }
}

#Preview {
    AudioView()
}
