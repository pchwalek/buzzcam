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
    @State private var isAnimated = false
    
    let customFont = Font.custom("Helvetica-Bold", size: 30) // Define a custom font

    var body: some View {
        NavigationView {
            ZStack {
                Color(red: 243/255, green: 237/255, blue: 151/255)
                    .edgesIgnoringSafeArea(.all)
                
                VStack {
                    Text("Discover Devices")
//                        .font(.title)
                        .font(customFont)
                        .foregroundColor(Color.white)
                        .padding(.vertical, 40)
                    
                    
                    List {
                        ForEach(bluetoothModel.filteredPeripherals) { peripheral in
                            HStack {
                                Text(peripheral.name ?? "Unknown")
                                    .font(.body)
                                
                                Spacer()
                                
                                Button(action: {
                                    // Action when Connect button is clicked
                                    connected = true // Set this to true to trigger the navigation
                                    bluetoothModel.connectToPeripheral(peripheral)
                                    isConnected = true
                                }) {
                                    Text("Connect")
                                        .foregroundColor(.white)
                                        .padding(.horizontal, 10)
                                        .padding(.vertical, 5)
                                        .background(Color.green)
                                        .cornerRadius(5)
                                }
                            }
                            .padding(.vertical, 8)
                        }
                        .transition(.fanInOut)
                    }
                }
            }
        }
        .onAppear {
            withAnimation(.easeInOut(duration: 0.5)) {
                self.isAnimated = true
            }
        }
    }
}

extension AnyTransition {
    static var fanInOut: AnyTransition {
        let insertion = AnyTransition.move(edge: .top).combined(with: .opacity)
        let removal = AnyTransition.move(edge: .bottom).combined(with: .opacity)
        return .asymmetric(insertion: insertion, removal: removal)
    }
}


//struct DiscoverView: View {
//    @EnvironmentObject var bluetoothModel: BluetoothModel
//    @State private var isConnected = false
//    @Binding var connected: Bool
//    
//    
//    var body: some View {
//        NavigationView {
//            VStack {
//                Text("Discover Devices").font(.title)
//                // loop thru devices with "BuzzCam" or "STM" in name
//                List(bluetoothModel.filteredPeripherals) { peripheral in
//                    Text(peripheral.name ?? "Unknown")
//                    Button(action: {
//                        // Action when Disconnect button is clicked
//                        connected = true // Set this to true to trigger the navigation
//                        bluetoothModel.connectToPeripheral(peripheral)
//                        isConnected = true
//                    }) {
//                        Text("Connect")
//                            .foregroundColor(Color.white)
//                    }
//                    .padding()
//                    .background(Color(white: 0.2))
//                    .cornerRadius(5)
//                    
//                }
//            }
//        }
//    }
//}

struct DiscoverView_Previews: PreviewProvider {
    static var previews: some View {
        DiscoverView(connected: .constant(false)).environmentObject(BluetoothModel())
    }
}

//#Preview {
//    DiscoverView(bluetoothModel: BluetoothModel())
//}
