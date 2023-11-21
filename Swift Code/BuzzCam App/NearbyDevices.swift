//
//  NearbyDevices.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI
import CoreBluetooth



struct NearbyDevices: View {
    @State private var isExpanded = false //change to false after cleanup
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    var body: some View {
//        VStack {
//            Text("test Discover Devices")
//            // loop thru devices with "BuzzCam" in name
//            List(bluetoothModel.peripherals) { peripheral in
//                Text(peripheral.name ?? "Unknown")
//                
//            }
//        }
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
                        List(bluetoothModel.filteredPeripherals) { peripheral in
                            Text(peripheral.name ?? "Unknown")
                        }
                        Text("test nearby device")
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
