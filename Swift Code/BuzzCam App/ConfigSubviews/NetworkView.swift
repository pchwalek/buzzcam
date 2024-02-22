//
//  NearbyDevicesConfig.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/11/23.
//

import SwiftUI
import Combine

struct NetworkView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    @State private var masterNodeEnabled = false
    @State private var slaveSyncEnabled = false
    @State private var masterChirpEnabled = false
    @State private var selectedChannel: Double = 32
    @State private var panID: String = ""
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Network")
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
                    VStack (alignment: .leading) {
                        HStack {
                            Text("Master node")
                                .fontWeight(.bold)
                                .padding()
                            
                            Toggle("", isOn: $masterNodeEnabled)
                                .labelsHidden()
                                .onChange(of: masterNodeEnabled) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableMasterNode(masterNodeEnabled: masterNodeEnabled)
                                }
                        }
                        
                        HStack {
                            Text("Slave sync")
                                .fontWeight(.bold)
                                .padding()
                            
                            Toggle("", isOn: $slaveSyncEnabled)
                                .labelsHidden()
                                .onChange(of: slaveSyncEnabled) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableSlaveSync(slaveSyncEnabled: slaveSyncEnabled)
                                }
                        }
                        
                        HStack {
                            Text("Master chirp")
                                .fontWeight(.bold)
                                .padding()
                            
                            Toggle("", isOn: $masterChirpEnabled)
                                .labelsHidden()
                                .onChange(of: masterChirpEnabled) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableMasterChirp(masterChirpEnabled: masterChirpEnabled)
                                }
                        }

                    }.padding()
                        .frame(
                            minWidth: 0,
                            maxWidth: .infinity,
                            alignment: .leading)
                        .background(Color(white: 0.98))
                        .cornerRadius(10)
                    
                    VStack(alignment: .leading) {
                        Text("Change channel: \(Int(selectedChannel))").fontWeight(.bold)
                        Slider(value: Binding(
                            get: {
                                selectedChannel
                            },
                            set: { newValue in
                                selectedChannel = newValue
                                // This code will be executed when the user starts dragging
                            }
                        ), in: 11...26, step: 1, onEditingChanged: { editingChanged in
                            if !editingChanged {
                                // This code will be executed when the user finishes dragging
                                bluetoothModel.changeChannel(channel: UInt32(selectedChannel))
                            }
                        })

                    }.padding().frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                    
                    VStack(alignment: .leading) {
                        Text("PanID: ").fontWeight(.bold)
                        HStack {
                            Text("0x")
                            TextField("", text: $panID)
                                .padding()
                                .background(
                                    RoundedRectangle(cornerRadius: 5)
                                        .fill(Color.white)
                                )
                                .overlay(
                                    RoundedRectangle(cornerRadius: 5)
                                        .stroke(Color.black, lineWidth: 1) // Border around the text input
                                )
                                .overlay(
                                    HStack {
                                        Spacer()
                                        Button("Done") {
                                            UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder), to: nil, from: nil, for: nil)
                                            bluetoothModel.sendPanID(panID: UInt32(panID, radix: 16) ?? 0)
                                        }
                                        .padding()
                                        .buttonStyle(BorderlessButtonStyle())
                                        .background(Color.gray)
                                        .cornerRadius(8)
                                    }
                                )
                                .foregroundColor(.black)
                        }
                    }.padding().frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                
                    
                    VStack(alignment: .leading) {
                        HStack {
                            Text("Number of Discovered Devices: ").fontWeight(.bold)
                            Text("\(bluetoothModel.configPacketData_NetworkState?.numberOfDiscoveredDevices ?? 0)")
                        }
//                        HStack {
//                            Text("Force rediscovery").fontWeight(.bold)
//                                .foregroundColor(.black)
//                            
//                            Button(action: {
//                                // Call the associated function when the button is pressed
//                                bluetoothModel.forceRediscovery()
//                            }) {
//                                Image(systemName: "arrow.clockwise").padding()
//                                    .foregroundColor(.black)
//                            }
//                            .buttonStyle(BorderlessButtonStyle())
//                            .background(Color.gray)
//                            .cornerRadius(8)
//                        }
                    }
                    .padding()
                    .frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                    
                    
                    
                    VStack(alignment: .leading) {
                        if let configPacketData = bluetoothModel.configPacketData_NetworkState {
                            VStack(alignment: .leading, spacing: 10) {
                                Text("Discovered Devices:")
                                    .fontWeight(.bold)
                                    .foregroundColor(.black)
                                    .padding(.bottom)

                                if !configPacketData.discoveredDeviceUid.isEmpty {
                                    ForEach(configPacketData.discoveredDeviceUid, id: \.self) { uid in
                                        HStack {
                                            Image(systemName: "circle.fill")
                                                .foregroundColor(.black)
                                                .font(.system(size: 10))
                                            Text("\(uid.addr)")
                                        }
                                    }
                                } else {
                                    Text("No devices discovered.")
                                }
                            }
                            .padding()
                        } else {
                            Text("NetworkState nil")
                                .padding()
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
        .onAppear {
            // Add an observer to monitor changes to configPacketData
            bluetoothModel.$configPacketData_NetworkState
                .sink { configPacketData_NetworkState in
                    self.updateMasterNode(configPacketData_NetworkState)
                    self.updateSlaveSync(configPacketData_NetworkState)
                }
                .store(in: &cancellables)
            
            bluetoothModel.$configPacketData_Audio
                .sink { configPacketData_Audio in
                    self.updateMasterChirp(configPacketData_Audio)
                }
                .store(in: &cancellables)
            
            // Trigger the initial update
            self.updateMasterNode(bluetoothModel.configPacketData_NetworkState)
            self.updateSlaveSync(bluetoothModel.configPacketData_NetworkState)
            self.updateMasterChirp(bluetoothModel.configPacketData_Audio)
            
            self.panID = "\(bluetoothModel.configPacketData_NetworkState?.panID ?? 0)"
            
            if let initialChannel = bluetoothModel.configPacketData_NetworkState?.channel {
                selectedChannel = Double(initialChannel)
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    
    private func updateMasterNode(_ configPacketData_NetworkState: ConfigPacketData_NetworkState?) {
        // Update masterNode based on configPacketData_NetworkState
        guard let configData = configPacketData_NetworkState, masterNodeEnabled != configData.masterNode else {
            return
        }
        masterNodeEnabled = configData.masterNode
    }
    
    private func updateSlaveSync(_ configPacketData_NetworkState: ConfigPacketData_NetworkState?) {
        // Update slaveSync based on configPacketData_NetworkState
        guard let configData = configPacketData_NetworkState, slaveSyncEnabled != configData.slaveSync else {
            return
        }
        slaveSyncEnabled = configData.slaveSync
    }
    
    private func updateMasterChirp(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        // Update masterNode based on configPacketData_NetworkState
        guard let configData = configPacketData_Audio, masterChirpEnabled != configData.chirpEnable else {
            return
        }
        masterChirpEnabled = configData.chirpEnable
    }
    
}

#Preview {
    NetworkView()
}
