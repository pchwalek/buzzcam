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
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Sensor Readings")
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
                VStack (alignment: .leading, spacing: 20) {
                    HStack {
                        Image(systemName: "thermometer").resizable()
                            .aspectRatio(contentMode: .fit)
                            .frame(width: 40, height: 40)
                        Text("Temperature:")
                            .font(.body)
                        Text(String(bluetoothModel.systemInfoPacketData?.temperature ?? 0))
                            .font(.title3)
                            .fontWeight(.bold)
                    }
                    
                    HStack {
                        Image(systemName: "humidity").resizable()
                            .aspectRatio(contentMode: .fit)
                            .frame(width: 40, height: 40)
                        Text("Humidity:")
                            .font(.body)
                        Text(String(bluetoothModel.systemInfoPacketData?.humidity ?? 0))
                            .font(.title3)
                            .fontWeight(.bold)
                    }
                    
                    HStack {
                        Image(systemName: "wind").resizable()
                            .aspectRatio(contentMode: .fit)
                            .frame(width: 40, height: 40)
                        Text("CO2:")
                            .font(.body)
                        Text(String(bluetoothModel.systemInfoPacketData?.co2 ?? 0))
                            .font(.title3)
                            .fontWeight(.bold)
                    }
                    
                    HStack {
                        Image(systemName: "lightbulb.max").resizable()
                            .aspectRatio(contentMode: .fit)
                            .frame(width: 40, height: 40)
                        Text("Light level:")
                            .font(.body)
                        Text(String(bluetoothModel.systemInfoPacketData?.light_level ?? 0))
                            .font(.title3)
                            .fontWeight(.bold)
                    }
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
