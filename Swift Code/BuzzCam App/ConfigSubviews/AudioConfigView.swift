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
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    @State private var channel1 = false
    @State private var channel2 = false
    
    @State private var audioCompressionEnabled = false
    
    let sampleFreq: [MicSampleFreq] = [.sampleRate8000, .sampleRate11025, .sampleRate16000, .sampleRate22500, .sampleRate24000, .sampleRate32000, .sampleRate44100, .sampleRate48000, .sampleRate96000]
    @State var selectedSampleFreq: MicSampleFreq? = .sampleRate16000
    @State var selectedBitResolution: MicBitResolution? = .bitRes8
//    @State var selectedMicGain: MicGain? = .gain60Db
//    @State var tempMicGain = "-15 dB"
    @State private var selectedMicGain: MicGain? = .gain60Db // Set default value
    @State private var tempMicGain = "-15 dB" // Set default selection

    let compressionType: [CompressionType] = [.opus, .flac]
    let micGains: [MicGain] = [
        .gainNeg15Db, .gainNeg12Db, .gainNeg9Db,
        .gainNeg6Db, .gainNeg3Db, .gain0Db, .gain3Db,
        .gain6Db, .gain9Db, .gain12Db, .gain15Db,
        .gain18Db, .gain21Db, .gain24Db, .gain27Db,
        .gain30Db, .gain33Db, .gain36Db, .gain39Db,
        .gain42Db, .gain45Db, .gain48Db, .gain51Db,
        .gain54Db, .gain57Db, .gain60Db
    ]
    var micGainLabels: [String] = [
            "-15 dB", "-12 dB", "-9 dB", "-6 dB", "-3 dB", "0 dB", "3 dB", "6 dB", "9 dB",
            "12 dB", "15 dB", "18 dB", "21 dB", "24 dB", "27 dB", "30 dB", "33 dB", "36 dB",
            "39 dB", "42 dB", "45 dB", "48 dB", "51 dB", "54 dB", "57 dB", "60 dB"
        ]
    @State var selectedCompressionType: CompressionType? = .opus
    @State var selectedCompressionFactor: Double = 5
    
    
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Audio")
                    .font(customFontTextBoldLarge)
                    .padding()
                
                Image(systemName: "chevron.down") //IMG_4587 (6)
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(
                GeometryReader { proxy in
                        Image("IMG_4587 (1)")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }
            ).onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                }
            }
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    HStack {
                        Text("Enable channel 1")
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                            .padding()
                        
                        Toggle("", isOn: $channel1)
                            .labelsHidden()
                            .onChange(of: channel1) {
                                // Call your function when the toggle is changed
                                print("calling enableAudioChannel1, channel1 is \(channel1)")
                                bluetoothModel.enableAudioChannel1(channel1: channel1)
                            }
                    }
 
                    HStack {
                        Text("Enable channel 2")
                            .font(customFontTextBold)
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
                            .font(customFontTextBold)
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
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                        HStack {
                            Button(action: {
                                // Handle 8-bit button tap
                                bluetoothModel.setBitResolution(bitResolution: .bitRes8)
                                print("sent bitRes8")

                            }) {
                                Text("8-bit")
                                    .font(customFontText)
                                    .padding()
                                    .foregroundColor(.white)
                                    .background(bluetoothModel.configPacketData_Audio?.bitResolution == .bitRes8 ? Color.blue : Color.gray)
                                    .cornerRadius(8)
                            }
                            
                            Button(action: {
                                // Handle 16-bit button tap
                                bluetoothModel.setBitResolution(bitResolution: .bitRes16)
                                print("sent bitRes16")

                            }) {
                                Text("16-bit")
                                    .font(customFontText)
                                    .padding()
                                    .foregroundColor(.white)
                                    .background(bluetoothModel.configPacketData_Audio?.bitResolution == .bitRes16 ? Color.blue : Color.gray)
                                    .cornerRadius(8)

                            }
                            
                            Button(action: {
                                // Handle 16-bit button tap
                                bluetoothModel.setBitResolution(bitResolution: .bitRes24)
                                print("sent bitRes24")
                            }) {
                                Text("24-bit")
                                    .font(customFontText)
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

                    
                    VStack (alignment: .leading) { 
                        Text("Microphone Gain")
                            .font(customFontTextBold)
                        .fontWeight(.bold)
                        
                        Picker("Select Mic Gain", selection: $tempMicGain) {
                            ForEach(micGainLabels, id: \.self) { gainLabel in
                                Text(gainLabel)
                                    .font(customFontText)
                                    .tag(gainLabel)
                            }
                        }
                        .onChange(of: tempMicGain) { newValue in
                            if let index = micGainLabels.firstIndex(of: newValue) {
                                selectedMicGain = micGains[index]
                                bluetoothModel.changeMicGain(micGain: selectedMicGain ?? MicGain.gain60Db)
                            }
                        }
                        .pickerStyle(.menu) // Style the picker as a dropdown menu
                        .background(Color(white: 0.90))
                        
                        
                    }.padding()
                        .frame(minWidth: 0, maxWidth: .infinity, alignment: .leading)
                        .background(Color(white: 0.98))
                        .cornerRadius(10)
                    
                    VStack(alignment: .leading) {
                        Text("Audio compression")
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                        VStack (alignment: .leading){
                            HStack {
                                Text("Enabled")
                                    .font(customFontTextBoldSmall)

                                Toggle("",isOn: $audioCompressionEnabled).labelsHidden()
                                    .onChange(of: audioCompressionEnabled) {
                                        // Call your function when the toggle is changed
                                        bluetoothModel.enableAudioCompression(audioCompressionEnabled: audioCompressionEnabled)
                                    }.padding()
                            }
                            
                            VStack(alignment: .leading) {
                                Text("Type")
                                    .font(customFontTextBoldSmall)

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
                                Text("Compression Factor: \(Int(selectedCompressionFactor))")
                                    .font(customFontTextBoldSmall)
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
                                Text("Estimated recording time: ")
                                    .font(customFontTextBoldSmall)

                                Text(String(bluetoothModel.configPacketData_Audio?.estimatedRecordTime ?? 0))
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
            print("initialized audioconfigview")
            selectedSampleFreq = bluetoothModel.configPacketData_Audio?.sampleFreq
            
            // Set initial value of micGain
            if let micGain = bluetoothModel.configPacketData_Audio?.micGain,
               let index = micGains.firstIndex(of: micGain) {
                tempMicGain = micGainLabels[index]
            } else {
                // In case there's no valid micGain from bluetoothModel, set a default or handle the error accordingly
                // For example, set to the default value you've previously defined or the first item in micGainLabels
                tempMicGain = micGainLabels[0] // Adjust according to your needs, ensuring it never crashes
            }
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
            
            selectedSampleFreq = bluetoothModel.configPacketData_Audio?.sampleFreq
            
            // Set the initial value of selectedSampleFreq based on the stored value in bluetoothModel
            selectedCompressionType = bluetoothModel.configPacketData_Audio?.audioCompressionType
            
            selectedMicGain = bluetoothModel.configPacketData_Audio?.micGain
            
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
    
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)

    
    var body: some View {
        HStack {
            Text("\(sampleFreqInt[sampleFreq.rawValue])").font(customFontText)
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
    
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)

    
    var body: some View {
        HStack {
            Text("\(compressionStrings[compressionType.rawValue])").font(customFontText)
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
