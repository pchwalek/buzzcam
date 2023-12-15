//
//  MainView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/18/23.
//

// color palette;
// main yellow: #f9ce32
// light blue: #e6f3fd
// light yellow: #f2dabe
// dark brown: #764730

import SwiftUI
import CoreBluetooth
import SwiftProtobuf
import Combine

struct MainView: View {
    @State private var beepOn = false
    @State private var annotationText: String = ""
    @State private var shouldNavigate = false
    @Binding var connected: Bool // is connected to peripheral
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var cancellables: Set<AnyCancellable> = Set()
    
    var body: some View {
        NavigationView {
            ScrollView(showsIndicators: false) {
                ScrollViewReader(content: { proxy in
                    // show loading screen if info not available yet
                    if (bluetoothModel.connectedPeripheral?.name) != nil {
                        
                        VStack {
                            // upper box
                            VStack(alignment: .center){
                                HStack {
                                    Text(String(bluetoothModel.connectedPeripheral?.name ?? "BuzzCam")).font(.custom("Menlo-Bold", size: 30))
                                    Spacer()
                                    
                                    Button(action: {
                                        // Action when Disconnect button is clicked
                                        connected = false
                                        // disconnect from peripheral
                                        bluetoothModel.disconnect()
                                    }) {
                                        Text("Disconnect")
                                            .foregroundColor(Color.white)
                                    }
                                    .padding()
                                    .background(Color(white: 0.2))
                                    .cornerRadius(5)
                                }
                            }
                            .padding()
                            .frame(
                                minWidth: 0,
                                maxWidth: .infinity,
                                minHeight: 200,
                                maxHeight: .infinity,
                                alignment: .topLeading
                            )
                            .background(Color(white: 0.5))
                            
                            //mark box
                            VStack{
                                HStack{
                                    Text("Mark #")
                                    Text(String(bluetoothModel.systemInfoPacketData?.mark_number ?? 0)).font(.title)
                                }
                                HStack {
                                    Spacer()
                                    Button(action: {
                                        bluetoothModel.markUpdates(annotationText: annotationText, beep: beepOn)
                                        // clear annotation text
                                        annotationText = ""
                                    }) {
                                        Text("Mark")
                                            .padding()
                                            .background(Color(white: 0.7))
                                            .cornerRadius(5)
                                    }
                                    
                                    Toggle(isOn: $beepOn) {
                                        Text("Play Beep")
                                            .frame(maxWidth: .infinity, alignment: .trailing)
                                    }
                                    Spacer()
                                }
                                TextField(
                                    "Add observation (max 40 char.)",
                                    text: $annotationText
                                )
                                .textFieldStyle(.roundedBorder)
                            }
                            .padding()
                            .frame(
                                minWidth: 0,
                                maxWidth: 300,
                                minHeight: 200,
                                maxHeight: .infinity,
                                alignment: .center
                            )
                            .background(Color.white)
                            .cornerRadius(15)
                            .offset(y: -100)
                            .padding(.bottom, -100)
                            .shadow(color: .black, radius: 10, x: 0, y: 5)
                            
                            // camera capture
                            VStack {
                                HStack {
                                    Button(action: {bluetoothModel.forceCameraCapture()}) {
                                        Image("camera.circle.fill")
                                            .resizable()
                                            .frame(width: 100, height: 100)
                                            .scaledToFit()
                                            .foregroundColor(Color.black)
                                    }
                                    .frame(width: 100, height: 100)
                                    .padding()
                                    
                                    Text("Camera Capture").font(.title2).padding()
                                }.padding()
                                    .frame(
                                        maxHeight: .infinity,
                                        alignment: .center
                                    )
                                    .background(Color(white:0.89))
                                    .cornerRadius(15)
                            }.padding(.vertical, 25.0)
                            
                            // statuses dropdown
                            StatusesView()
                            Spacer()
                            // sensor readings dropdown
                            SensorReadingView()
                            Spacer()
                            // nearby devices dropdown
                            NearbyDevices()
                            
                            Image("BuzzCam Logo").resizable()
                                .frame(width: 100, height: 100)
                                .scaledToFit().padding() // replace with transparent background logo
                        }
                    } else {
                        // Placeholder view while the peripheral name is not available, waiting for it to load
                        Text("Loading...")
                    }
                })
            }
        }
    }
}


#Preview {
    MainView(connected: .constant(true))
}
