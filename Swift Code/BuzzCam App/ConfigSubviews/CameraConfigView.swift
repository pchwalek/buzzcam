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
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Camera Control")
                    .font(customFontTextBoldLarge)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(
                GeometryReader { proxy in
                        Image("IMG_4587 (3)")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }).onTapGesture {
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
                                    .font(customFontTextBoldSmall)
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.pairWithNearbyCameras()
                                }) {
                                    Image(systemName: "antenna.radiowaves.left.and.right").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            HStack {
                                Text("Force Camera Capture")
                                    .font(customFontTextBoldSmall)
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()

                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.forceCameraCapture()
                                }) {
                                    Image(systemName: "camera").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
                                .cornerRadius(8)
                            }
                            .padding(.bottom, 10)
                            
                            
                            HStack {
                                Text("Wakeup Cameras")
                                    .font(customFontTextBoldSmall)
                                    .padding()
                                    .foregroundColor(.black)
                                Spacer()
                                Button(action: {
                                    // Call the associated function when the button is pressed
                                    bluetoothModel.wakeupCameras()
                                }) {
                                    Image(systemName: "power").padding()
                                        .foregroundColor(.black)
                                }
                                .buttonStyle(BorderlessButtonStyle())
                                .background(Color(white: 0.8))
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
