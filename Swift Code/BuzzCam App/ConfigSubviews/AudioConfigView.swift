//
//  AudioConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 11/30/23.
//

import SwiftUI
import Combine

struct AudioConfigView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    
    @State private var channel1 = false
    @State private var channel2 = false
    
    @State private var audioCompressionEnabled = false
    
    let sampleFreq: [MicSampleFreq] = [.sampleRate8000, .sampleRate11025, .sampleRate16000, .sampleRate22500, .sampleRate24000, .sampleRate32000, .sampleRate44100, .sampleRate48000, .sampleRate96000]
    @State var selectedSampleFreq: MicSampleFreq? = .sampleRate16000
    @State var selectedBitResolution: MicBitResolution? = .bitRes8
    let compressionType: [CompressionType] = [.opus, .flac]
    @State var selectedCompressionType: CompressionType? = .opus
    @State var selectedCompressionFactor: Double = 5
    
    
    
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
                        Text("Enable channel 1")
                            .font(.title2)
                            .fontWeight(.bold)
                            .padding()
                        
                        Toggle("", isOn: $channel1)
                            .labelsHidden()
                            .onChange(of: channel1) {
                                // Call your function when the toggle is changed
//                                print("calling enableAudioChannel1, channel1 is \(channel1)")
                                bluetoothModel.enableAudioChannel1(channel1: channel1)
                            }
                    }
 
                    HStack {
                        Text("Enable channel 2")
                            .font(.title2)
                            .fontWeight(.bold)
                            .padding()
                        
                        Toggle("", isOn: $channel2)
                            .labelsHidden()
                            .onChange(of: channel2) {
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
                            ScrollView {
                                VStack {
                                    ForEach(sampleFreq, id: \.self) { item in
                                        FrequencyCell(sampleFreq: item, selectedSampleFreq: self.$selectedSampleFreq)
                                    }
                                }
                            }
                            .onChange(of: selectedSampleFreq ?? .sampleRate16000) {
                                // Call your function here with the new selected sample frequency
                                bluetoothModel.changeSampleFreq(sampleFreq: selectedSampleFreq ?? .sampleRate16000)
                            }
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
                            
                            Button(action: {
                                // Handle 16-bit button tap
                                bluetoothModel.setBitResolution(bitResolution: .bitRes24)
                            }) {
                                Text("24-bit")
                                    .padding()
                                    .foregroundColor(.white)
                                    .background(bluetoothModel.configPacketData_Audio?.bitResolution == .bitRes24 ? Color.blue : Color.gray)
                                    .cornerRadius(8)
                            }
                        }
                    }
                    .padding()
                    .frame(minWidth: 0, maxWidth: .infinity, alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                    
                    VStack(alignment: .leading) {
                        Text("Audio compression")
                            .font(.title2)
                            .fontWeight(.bold)
                        VStack (alignment: .leading){
                            HStack {
                                Text("Enabled").fontWeight(.bold)
                                Toggle("",isOn: $audioCompressionEnabled).labelsHidden()
                                    .onChange(of: audioCompressionEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableAudioCompression(audioCompressionEnabled: audioCompressionEnabled)
                                    }.padding()
                            }
                            
                            VStack(alignment: .leading) {
                                Text("Type").fontWeight(/*@START_MENU_TOKEN@*/.bold/*@END_MENU_TOKEN@*/)
                                VStack (alignment: .leading){
                                    ScrollView {
                                        VStack {
                                            ForEach(compressionType, id: \.self) { item in
                                                CompressionTypeCell(compressionType: item, selectedCompressionType: self.$selectedCompressionType)
                                            }
                                        }.padding()
                                    }
                                    .onChange(of: selectedCompressionType ?? .opus) {
                                        // Call your function here with the new selected sample frequency
                                        bluetoothModel.changeCompressionType(compressionType: selectedCompressionType ?? .opus)
                                    }
                                }
                            }
                            
                            VStack(alignment: .leading) {
                                Text("Compression Factor: \(Int(selectedCompressionFactor))").fontWeight(.bold)
                                Slider(value: Binding(
                                    get: {
                                        selectedCompressionFactor
                                    },
                                    set: { newValue in
                                        selectedCompressionFactor = newValue
                                        // This code will be executed when the user starts dragging
                                    }
                                ), in: 1...10, step: 1, onEditingChanged: { editingChanged in
                                    if !editingChanged {
                                        // This code will be executed when the user finishes dragging
                                        bluetoothModel.changeCompressionFactor(compressionFactor: UInt32(selectedCompressionFactor))
                                    }
                                })
                                .padding()
                                Text("Estimated recording time: " + String(bluetoothModel.configPacketData_Audio?.estimatedRecordTime ?? 0)).fontWeight(.bold)
                                    .font(.body)
                            }
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
            // Set the initial value of selectedSampleFreq based on the stored value in bluetoothModel
            print("initialized")
            selectedSampleFreq = bluetoothModel.configPacketData_Audio?.sampleFreq
            
            // move all the onAppears here
            // Add an observer to monitor changes to configPacketData_Audio
            bluetoothModel.$configPacketData_Audio
                .sink { configPacketData_Audio in
                    self.updateChannel1(configPacketData_Audio)
                }
                .store(in: &cancellables)
            
            // Trigger the initial update
            self.updateChannel1(bluetoothModel.configPacketData_Audio)
            
            // Add an observer to monitor changes to configPacketData_Audio
            bluetoothModel.$configPacketData_Audio
                .sink { configPacketData_Audio in
                    self.updateChannel2(configPacketData_Audio)
                }
                .store(in: &cancellables)
            
            // Trigger the initial update
            self.updateChannel2(bluetoothModel.configPacketData_Audio)
            
            //                selectedSampleFreq = bluetoothModel.configPacketData_Audio?.sampleFreq
            
            // Set the initial value of selectedSampleFreq based on the stored value in bluetoothModel
            selectedCompressionType = bluetoothModel.configPacketData_Audio?.audioCompressionType
            
            if let initialFactor = bluetoothModel.configPacketData_Audio?.audioCompressionFactor {
                selectedCompressionFactor = Double(initialFactor)
            }
            
            selectedBitResolution = bluetoothModel.configPacketData_Audio?.bitResolution
            
            bluetoothModel.$configPacketData_Audio
                .sink { configPacketData_Audio in
                    // Update beepOn when systemInfoPacketData changes
                    self.updateAudioCompressionToggle(configPacketData_Audio)
                }
                .store(in: &cancellables) // Store the cancellable to avoid memory leaks
            
            // Trigger the initial update
            self.updateAudioCompressionToggle(bluetoothModel.configPacketData_Audio)
            
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    
    private func updateChannel1(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update channel1 based on configPacketData_Audio
        guard let configData = configPacketData_Audio, channel1 != configData.channel1 else {
            return
        }
        channel1 = configData.channel1
    }
    
    private func updateChannel2(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update channel2 based on configPacketData_Audio
        guard let configData = configPacketData_Audio, channel2 != configData.channel2 else {
            return
        }
        channel2 = configData.channel2
    }
    
    private func updateAudioCompressionToggle(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update channel2 based on configPacketData_Audio
        audioCompressionEnabled = configPacketData_Audio?.audioCompressionEnabled ?? false
    }
    
}

struct FrequencyCell: View {
    
    let sampleFreq: MicSampleFreq
    @Binding var selectedSampleFreq: MicSampleFreq?
    let sampleFreqInt: [Int] = [8000, 11025, 16000, 22500, 24000, 32000, 44100, 48000, 96000]
    
    
    var body: some View {
        HStack {
            Text("\(sampleFreqInt[sampleFreq.rawValue])")
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

struct CompressionTypeCell: View {
    
    let compressionType: CompressionType
    @Binding var selectedCompressionType: CompressionType?
    let compressionStrings: [String] = ["Opus", "Flac"]
    
    var body: some View {
        HStack {
            Text("\(compressionStrings[compressionType.rawValue])")
            Spacer()
            if compressionType == selectedCompressionType {
                Image(systemName: "largecircle.fill.circle")
                    .foregroundColor(.accentColor)
            } else {
                Image(systemName: "circle")
                    .foregroundColor(.secondary)
            }
        }
        .onTapGesture {
            self.selectedCompressionType = self.compressionType
        }
    }
}

#Preview {
    AudioConfigView()
}
