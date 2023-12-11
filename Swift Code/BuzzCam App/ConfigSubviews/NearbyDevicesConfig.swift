//
//  NearbyDevicesConfig.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/11/23.
//

import SwiftUI

struct NearbyDevicesConfig: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Nearby Devices")
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
                        if let configPacketData = bluetoothModel.configPacketData_Discover {
                            Text("Number of Discovered Devices: \(configPacketData.numberOfDiscoveredDevices)")
                                .padding()

                            if !configPacketData.discoveredDeviceUid.isEmpty {
                                List(configPacketData.discoveredDeviceUid, id: \.self) { uid in
                                    Text("Device UID: \(uid)")
                                }
                            } else {
                                Text("No devices discovered.")
                                    .padding()
                            }
                        } else {
                            Text("ConfigPacketData_Discover is nil.")
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
                    
                    VStack(alignment: .leading) {
                        
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
}

#Preview {
    NearbyDevicesConfig()
}
