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
    
    let customFontTitle = Font.custom("Futura-Bold", size: 20) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20) // Define a custom font
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Nearby Devices")
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
                            Image("flowers 3") // Replace "your_image_name" with the name of your image asset
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
                VStack {
                    HStack {
                        Text("Number of Nearby Devices: ").font(customFontTextBold)
                        Text("\(bluetoothModel.configPacketData_NetworkState?.numberOfDiscoveredDevices ?? 0)").font(customFontText)
                    }
                }
                .frame(maxWidth: .infinity)
                .padding(30)                
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
        .padding(.bottom, 40)
    }
}

#Preview {
    NearbyDevices()
}
