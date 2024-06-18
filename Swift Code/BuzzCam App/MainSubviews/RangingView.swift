//
//  RangingView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 2/20/24.
//

//
//  SensorReadingView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI
import Combine

struct RangingView: View {
    @State private var isExpanded = false
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    let customFontTitle = Font.custom("Futura-Bold", size: 20) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Ranging & Nearby Devices")
                    .font(customFontTitle)
                    .foregroundColor(Color.white)
                    .shadow(color: .black, radius: 5, x: 0, y: 2)
                    .padding()
                
                Image(systemName: "chevron.down.circle.fill")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                    .shadow(color: .black, radius: 5, x: 0, y: 2)
                    .foregroundColor(Color.white)
                Spacer()
            }.frame(maxWidth: .infinity)
                .background(
                    GeometryReader { proxy in
                        Image("flowers 3")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }
                )
                .onTapGesture {
                    withAnimation {
                        isExpanded.toggle()
                    }
                }
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    VStack(alignment: .leading) {
                        Text("Ranging List")
                            .font(customFontTextBold)
                            .fontWeight(.bold)
                        
                        HStack {
                            Text("Number of Nearby Devices: ").font(customFontText)
                            // \(bluetoothModel.systemInfoPacketData?.number_discovered_devices ?? 0)
                            Text("\(bluetoothModel.systemInfoPacketData?.discovered_devices.count ?? 0)").font(customFontText)
                        }.padding(.top, 10)
                        
                        HStack {
                            Text("Click to start ranging and generate a new list")
                                .font(customFontText)
                            Button(action: {
                                // Call the associated function when the button is pressed
                                bluetoothModel.startRanging()
                            }) {
                                Image(systemName: "dot.radiowaves.forward").padding()
                                    .foregroundColor(.black)
                            }
                            .buttonStyle(BorderlessButtonStyle())
                            .background(Color(white: 0.9))
                            .cornerRadius(8)
                        }.padding(.vertical, 30)

                        HStack {
                            VStack {
                                Text("System UIDs")
                                    .font(customFontText)
                                    .fontWeight(.bold)
                                
                                if let systemInfoPacketData = bluetoothModel.systemInfoPacketData {
                                    VStack{
                                        if !systemInfoPacketData.discovered_devices.isEmpty {
                                            ForEach(systemInfoPacketData.discovered_devices, id: \.self) { device in
                                                let hexUID = String(format: "%08X", device.uid)
                                                Text(hexUID).font(customFontText)
                                            }
                                        } else {
                                            Text("...").font(customFontText)
                                        }
                                    }
                                }
                            }
                            
                            Spacer()
                            
                            VStack {
                                Text("Range Values")
                                    .font(customFontText)
                                    .fontWeight(.bold)
                                
                                if let systemInfoPacketData = bluetoothModel.systemInfoPacketData {
                                    VStack{
                                        if !systemInfoPacketData.discovered_devices.isEmpty {
                                            ForEach(systemInfoPacketData.discovered_devices, id: \.self) { device in
                                                Text("\(Int(device.range))").font(customFontText)
                                            }
                                        } else {
                                            Text("...").font(customFontText)
                                        }
                                    }
                                }
                            }
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
                .padding(30)
                
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
        .padding(.bottom, UIScreen.main.bounds.height * 0.08) // 8% padding from the bottom of the screen
    }
}


#Preview {
    RangingView()
}
