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
    
    let scanInterval: TimeInterval = 10 // Adjust the interval as needed
    
    let customFontTitle = Font.custom("Futura-Bold", size: 30) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-Bold", size: 18) // Define a custom font
    //HiraginoSans-W3
//    let customFontText = Font.custom("HiraginoSans-W3", size: 17) // Define a custom font

    var body: some View {
//        NavigationView {
            ZStack {
//                Color(red: 243/255, green: 237/255, blue: 151/255)
//                    .edgesIgnoringSafeArea(.all)
                
                StarrySkyView()
                
                VStack {
                    Text("Discover Devices")
//                        .font(.title)
                        .font(customFontTitle)
//                        .foregroundColor(Color.white)
                        .padding(.top, 40)
                    
                    Text("All discoverable devices shown below. Click on any to connect.")
                        .font(customFontText)
                        .padding(.horizontal, 40)
                        .padding(.vertical, 20)
//                        .foregroundColor(Color.white)
                        .multilineTextAlignment(.center)
                    
                    ScrollView {
                        VStack(spacing: 16) {
                            ForEach(bluetoothModel.filteredPeripherals) { peripheral in
                                VStack {
                                    Rectangle()
                                        .fill(Color(red: 243/255, green: 237/255, blue: 151/255).opacity(0.5))
                                        .frame(height: 60)
                                        .cornerRadius(8)
                                        .overlay(
                                            HStack {
                                                Text(peripheral.name ?? "Unknown")
                                                    .font(customFontTextBold)
                                                
                                                
                                                
                                                Spacer()
                                                
                                                Button(action: {
                                                    // Action when Connect button is clicked
                                                    connected = true // Set this to true to trigger the navigation
                                                    bluetoothModel.connectToPeripheral(peripheral)
                                                    isConnected = true
                                                }) {
                                                    Text("Connect")
                                                        .font(customFontText)
                                                        .foregroundColor(.white)
                                                        .padding(.horizontal, 13)
                                                        .padding(.vertical, 7)
                                                        .background(Color(red: 0.4588, green: 0.0510, blue: 0.2157).opacity(0.4))
                                                        .cornerRadius(5)
                                                }
                                            }
                                            .padding()
                                        )
                                }
                            }
                        }
                        .padding(.horizontal)
                    }
//                    List {
//                        ForEach(bluetoothModel.filteredPeripherals) { peripheral in
//                            HStack {
//                                Text(peripheral.name ?? "Unknown")
//                                    .font(.body)
//                                
//                                Spacer()
//                                
//                                Button(action: {
//                                    // Action when Connect button is clicked
//                                    connected = true // Set this to true to trigger the navigation
//                                    bluetoothModel.connectToPeripheral(peripheral)
//                                    isConnected = true
//                                }) {
//                                    Text("Connect")
//                                        .foregroundColor(.white)
//                                        .padding(.horizontal, 10)
//                                        .padding(.vertical, 5)
//                                        .background(Color.green)
//                                        .cornerRadius(5)
//                                }
//                            }
//                            .padding(.vertical, 8)
//                        }
//                        .transition(.fanInOut)
//                    }
//                }
            }
        }
        .onAppear {
            withAnimation(.easeInOut(duration: 0.5)) {
                self.isAnimated = true
            }
            // Start scanning for peripherals periodically
//            startScanning()
        }
    }
    
    // Method to start scanning for peripherals periodically
//    func startScanning() {
//        print("in startScanning")
//        DispatchQueue.main.asyncAfter(deadline: .now() + scanInterval) { [self] in
//            // Access the CBCentralManager instance from bluetoothModel
//            if let centralManager = bluetoothModel.centralManager {
//                centralManager.scanForPeripherals(withServices: nil, options: nil)
//            } else {
//                print("Central manager not initialized.")
//            }
//            // Trigger scanning again after scanInterval
//            startScanning()
//        }
//    }
}



//extension AnyTransition {
//    static var fanInOut: AnyTransition {
//        let insertion = AnyTransition.move(edge: .top).combined(with: .opacity)
//        let removal = AnyTransition.move(edge: .bottom).combined(with: .opacity)
//        return .asymmetric(insertion: insertion, removal: removal)
//    }
//}


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
