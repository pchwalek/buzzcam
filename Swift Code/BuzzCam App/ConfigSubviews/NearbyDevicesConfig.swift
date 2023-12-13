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
                        HStack {
                            Text("Number of Discovered Devices: ").fontWeight(.bold)
                            Text("\(bluetoothModel.configPacketData_Discover?.numberOfDiscoveredDevices ?? 0)")
                        }
                        HStack {
                            Text("Force rediscovery").fontWeight(.bold)
                                .foregroundColor(.black)

                            Button(action: {
                                // Call the associated function when the button is pressed
                                bluetoothModel.forceRediscovery()
                            }) {
                                Image(systemName: "arrow.clockwise").padding()
                                    .foregroundColor(.black)
                            }
                            .buttonStyle(BorderlessButtonStyle())
                            .background(Color.gray)
                            .cornerRadius(8)
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
                        if let configPacketData = bluetoothModel.configPacketData_Discover {
                            
                            VStack (alignment: .leading) {
                                Text("Discovered Devices: ").fontWeight(.bold).padding(.bottom)
                                    .foregroundColor(.black)
                                if !configPacketData.discoveredDeviceUid.isEmpty {
                                    ForEach(configPacketData.discoveredDeviceUid, id: \.self) { uid in
                                        HStack {
                                            Image(systemName: "circle.fill") // You can use any bullet point icon here
                                                .foregroundColor(.black)
                                                .font(.system(size: 10))
                                            Text("\(uid)")
                                        }
                                    }
                                } else {
                                    Text("No devices discovered.")
                                        .padding()
                                }
                            }.padding()
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
