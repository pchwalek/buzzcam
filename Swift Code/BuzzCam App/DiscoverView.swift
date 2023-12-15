//
//  DiscoverDevicesView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/23/23.
//

import SwiftUI
import CoreBluetooth

// Extend CBPeripheral to conform to Identifiable
extension CBPeripheral: Identifiable {
    public var id: UUID {
        return identifier
    }
}

struct DiscoverView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isConnected = false
    @Binding var connected: Bool
    
    
    var body: some View {
        NavigationView {
            VStack {
                Text("Discover Devices").font(.title)
                // loop thru devices with "BuzzCam" or "STM" in name
                List(bluetoothModel.filteredPeripherals) { peripheral in
                    Text(peripheral.name ?? "Unknown")
                    Button(action: {
                        // Action when Disconnect button is clicked
                        connected = true // Set this to true to trigger the navigation
                        bluetoothModel.connectToPeripheral(peripheral)
                        isConnected = true
                    }) {
                        Text("Connect")
                            .foregroundColor(Color.white)
                    }
                    .padding()
                    .background(Color(white: 0.2))
                    .cornerRadius(5)
                    
                }
            }
        }
    }
}

struct DiscoverView_Previews: PreviewProvider {
    static var previews: some View {
        DiscoverView(connected: .constant(false)).environmentObject(BluetoothModel())
    }
}

//#Preview {
//    DiscoverView(bluetoothModel: BluetoothModel())
//}
