//
//  SensorReadingView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI

struct SensorReadingView: View {
    @State private var isExpanded = false
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    let customFontTitle = Font.custom("Futura-Bold", size: 20) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20) // Define a custom font
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Sensor Readings")
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
                            Image("flowers 1")
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
                        HStack {
                            Image(systemName: "thermometer").resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 30, height: 30)
                            Text("Temperature:")
                                .font(customFontText)
                            Text(String(bluetoothModel.systemInfoPacketData?.temperature ?? 0))
                                .font(customFontTextBold)
                        }
                        
                        HStack {
                            Image(systemName: "humidity").resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 30, height: 30)
                            Text("Humidity:")
                                .font(customFontText)
                            Text(String(bluetoothModel.systemInfoPacketData?.humidity ?? 0))
                                .font(customFontTextBold)
                        }
                        
                        HStack {
                            Image(systemName: "wind").resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 30, height: 30)
                            Text("CO2:")
                                .font(.body)
                            Text(String(bluetoothModel.systemInfoPacketData?.co2 ?? 0))
                                .font(customFontTextBold)
                        }
                        
                        HStack {
                            Image(systemName: "lightbulb.max").resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 30, height: 30)
                            Text("Light level:")
                                .font(customFontText)
                            Text(String(bluetoothModel.systemInfoPacketData?.light_level ?? 0))
                                .font(customFontTextBold)
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
    }
}

#Preview {
    SensorReadingView()
}
