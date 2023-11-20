//
//  StatusesView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/20/23.
//

import SwiftUI

struct StatusesView: View {
    @State private var isExpanded = false //change to false after cleanup
    @State private var deviceEnabled = true
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    var body: some View {
        VStack (alignment: .leading) {
                HStack {
                    Spacer()
                    Text("Statuses")
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
                            Text("Device enabled").font(.title2)
                                .fontWeight(.bold).padding()
                            Toggle("",isOn: $deviceEnabled).labelsHidden()
                        }
                        
                        
                        VStack(alignment: .leading) {
                            Text("SD Card Status")
                                .font(.title2)
                                .fontWeight(.bold)
                            VStack (alignment: .leading, spacing: 10){
                                Text("Detected: " + String(bluetoothModel.systemInfoPacketData?.sd_detected ?? false))
                                    .font(.body)
                                Text("Space remaining: " + String(bluetoothModel.systemInfoPacketData?.space_remaining ?? 0))
                                    .font(.body)
                                Text("Estimated recording time: " + String(bluetoothModel.systemInfoPacketData?.estimated_recording_time ?? 0))
                                    .font(.body)
                            }
                            .padding()
                        }
                        .padding()
                        .frame(
                              minWidth: 0,
                              maxWidth: .infinity,
                              alignment: .leading)
                        .background(Color(white: 0.98))
                        .cornerRadius(10)
                        
                        
                        
                        VStack(alignment: .leading) {
                            Text("Battery Status")
                                .font(.title2)
                                .fontWeight(.bold)
                            VStack (alignment: .leading, spacing: 10){
                                Text("Is charging: " + String(bluetoothModel.systemInfoPacketData?.battery_charging ?? false))
                                    .font(.body)
                                Text("Battery Voltage: " +  String(bluetoothModel.systemInfoPacketData?.battery_voltage ?? 0))
                                    .font(.body)
                            }
                            .padding()
                        }
                        .padding()
                        .frame(
                              minWidth: 0,
                              maxWidth: .infinity,
                              alignment: .leading)
                        .background(Color(white: 0.98))
                        .cornerRadius(10)
                                
                        
                    }
                    .padding()
                    
                }
            }
            .frame(maxWidth: .infinity)
            .background(Color(white:0.90))
    }
}

#Preview {
    StatusesView()
}
