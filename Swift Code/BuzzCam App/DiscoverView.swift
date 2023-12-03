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
//    @State private var shouldNavig/**/te = false
    @Binding var connected: Bool
//    let testList = ["a","b","c"]
    

    var body: some View {
        NavigationView {
            VStack {
                Text("test Discover Devices")
                // loop thru devices with "BuzzCam" in name
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
//                List(testList, id: \.self) { str in
//                    Text(str)
//                    Button("Connect") {
//                        isConnected = true
//                        shouldNavigate = true
//                    }
//                    .background(
//                        NavigationLink("", destination: MainView().navigationBarBackButtonHidden(true), isActive: $shouldNavigate)
//                    )
//                }
            }
        }
    }
}


//struct DiscoverView: View {
//    @EnvironmentObject var bluetoothModel: BluetoothModel
//    @State private var isConnected = false
//
//    var body: some View {
//        NavigationStack {
//            VStack {
//                Text("test")
//                List(bluetoothModel.peripherals) { peripheral in
//                    //            Button(action: { bluetoothModel.connect(to: peripheral) }) {
//                    Text(peripheral.name ?? "Unknown")
//                    //            }
////                    
////                    NavigationLink("Connect") {
////                      MainView()
////                    }
////                    .buttonStyle(.bordered)
////                    
//                    NavigationLink(destination: {
//                        MainView()
//                    }, label: {
//                        Text("Connect")
//                            .foregroundColor(.white)  // Set the modifiers to your liking.
//                            .padding(.vertical)
//                            .padding(.horizontal, 50)
//                            .background(.blue)
//                            .cornerRadius(10)
//                    })
//                    Button("Connect") {
//                        bluetoothModel.connectToPeripheral(peripheral)
//                        isConnected = true
//                    }.background(
//                        NavigationLink {
//                            MainView()
//                        } label: {
//                            Label("Work Folder", systemImage: "folder")
//                        }
//                    )
//                }
//            }
//        }
//    }
//}

//
//struct PeripheralDetailView: View {
//    @Binding var peripheral: CBPeripheral
//    @Binding var viewModel: BluetoothViewModel
//
//    var body: some View {
//        VStack {
//            Text("Peripheral Name: \(peripheral.name ?? "Unnamed")")
//            Button("Connect") {
//                viewModel.connect(peripheral: peripheral)
//            }
//            Button("Read Characteristic") {
//                viewModel.readCharacteristic(peripheral: peripheral)
//            }
//            Button("Write Characteristic") {
//                viewModel.writeCharacteristic(peripheral: peripheral, value: "YourValue")
//            }
//        }
//    }
//}

struct DiscoverView_Previews: PreviewProvider {
    static var previews: some View {
        // Create a BluetoothModel for the preview
//        let bluetoothModel = BluetoothModel()
//
//        return DiscoverView(bluetoothModel: bluetoothModel)
        DiscoverView(connected: .constant(false)).environmentObject(BluetoothModel())
    }
}

//#Preview {
//    DiscoverView(bluetoothModel: BluetoothModel())
//}
