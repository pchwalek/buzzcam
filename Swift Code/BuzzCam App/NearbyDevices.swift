//
//  NearbyDevices.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI
import CoreBluetooth

struct NearbyDevices: View {
    @State private var isExpanded = false
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
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
                VStack {
                    HStack {
                        Text("Number of Nearby Devices: ").fontWeight(.bold)
                        Text("\(bluetoothModel.configPacketData_Discover?.numberOfDiscoveredDevices ?? 0)")
                    }
                }
                .frame(maxWidth: .infinity)
                .padding(30)
                
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
}

#Preview {
    NearbyDevices()
}
