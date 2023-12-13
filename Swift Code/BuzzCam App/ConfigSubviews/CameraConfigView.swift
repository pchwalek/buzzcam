//
//  CameraConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/11/23.
//

import SwiftUI

struct CameraConfigView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false

    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Camera Control")
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
                    VStack(alignment: .leading) {
                        VStack(alignment: .leading) {
                            HStack {
                                Text("Pair with Nearby Cameras")
                                    .padding()
                                    .foregroundColor(.black)

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.pairWithNearbyCameras()
                                }) {
                                    Image(systemName: "antenna.radiowaves.left.and.right").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color.gray)
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Force Camera Capture")
                                    .padding()
                                    .foregroundColor(.black)

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.forceCameraCapture()
                                }) {
                                    Image(systemName: "camera").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color.gray)
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            
                            HStack {
                                Text("Wakeup Cameras")
                                    .padding()
                                    .foregroundColor(.black)

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.wakeupCameras()
                                }) {
                                    Image(systemName: "power").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color.gray)
                                .cornerRadius(8)
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
            .padding()
            }
        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
            
    }
        
}

#Preview {
    CameraConfigView()
}
