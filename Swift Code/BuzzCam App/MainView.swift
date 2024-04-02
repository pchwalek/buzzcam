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
import Liquid
//import Shiny

struct MainView: View {
    @State private var beepOn = false
    @State private var annotationText: String = ""
    @State private var shouldNavigate = false
    @Binding var connected: Bool // is connected to peripheral
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var cancellables: Set<AnyCancellable> = Set()
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 40) // Define a custom font
    
    @State private var beeScale: CGFloat = 1.0 // Define beeScale here
    
    var body: some View {
        NavigationView {
            ScrollView(showsIndicators: false) {
                ScrollViewReader(content: { proxy in
                    // show loading screen if info not available yet
                    if ((bluetoothModel.connectedPeripheral?.name) != nil) {
                        
                        VStack {
                            // upper box
                            VStack(alignment: .center){
                                HStack {
                                    Text(String(bluetoothModel.connectedPeripheral?.name ?? "BuzzCam").split(separator: "_")[1]) // split by underscore
                                        .font(customFontTitle)
                                        .foregroundColor(Color.white)
                                        .shadow(color: .black, radius: 3, x: 0, y: 2)
                                    Spacer()
                                    
                                    Button(action: {
                                        // Action when Disconnect button is clicked
                                        connected = false
                                        // disconnect from peripheral
                                        bluetoothModel.disconnect()
                                    }) {
                                        Text("Disconnect")
                                            .foregroundColor(Color.white)
                                            .font(customFontText)
                                    }
                                    .padding()
                                    .background(Color(white: 0.2))
                                    .opacity(0.8)
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
                            .background(
                                Image("patagonia 4") // Replace "your_image_name" with the name of your image asset
                                    .resizable()
                                    .scaledToFill()
                                    .edgesIgnoringSafeArea(.all)
                                    .opacity(0.9) // Adjust the opacity of the image if needed
//                                    .blur(radius: 1)
                            )
                            
                            //mark box
                            VStack (alignment: .leading) {
                                
                                ZStack (alignment: .center){
                                    Liquid()
                                        .frame(width: 240, height: 240)
                                        .foregroundColor(Color(red: 0x3D / 255, green: 0xA5 / 255, blue: 0xD9 / 255))
                                        .opacity(0.3)
                                    
                                    
                                    Liquid()
                                        .frame(width: 220, height: 220)
                                        .foregroundColor(Color(red: 0x3D / 255, green: 0xA5 / 255, blue: 0xD9 / 255))
                                        .opacity(0.6)
                                    
                                    Liquid(samples: 5)
                                        .frame(width: 200, height: 200)
                                        .foregroundColor(Color(red: 0x3D / 255, green: 0xA5 / 255, blue: 0xD9 / 255))
                                    
                                    
                                    HStack{
                                        Text("Mark #").foregroundColor(Color.white).font(customFontTextBold)
                                        Text(String(bluetoothModel.systemInfoPacketData?.mark_number ?? 0))
                                            .font(customFontTextBoldLarge)
                                            .foregroundColor(Color.white)
                                            .contentTransition(.numericText())
                                    }
                                }.frame(maxWidth: .infinity)
                                
                                
                                //                                HStack{
                                //                                    Text("Mark #").foregroundColor(Color.black)
                                //                                    Text(String(bluetoothModel.systemInfoPacketData?.mark_number ?? 0))
                                //                                        .font(customFontTitle)
                                //                                        .foregroundColor(Color.black)
                                //                                }
                                VStack (alignment: .leading) {
                                    PresetView()
                                    
                                    Text("Custom").font(customFontTextBold)
                                    
                                    HStack {
                                        Button(action: {
                                            bluetoothModel.markUpdates(annotationText: annotationText, beep: beepOn)
                                            // clear annotation text
                                            annotationText = ""
                                        }) {
                                            Text("Mark")
                                                .font(customFontText)
                                                .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20)) // Adjusted padding for thinner buttons
                                                .background(Color(white: 0.7))
                                                .cornerRadius(5)
                                        }
                                        
                                        Toggle(isOn: $beepOn) {
                                            Text("Play Beep")
                                                .font(customFontText)
                                                .frame(maxWidth: .infinity, alignment: .trailing)
                                        }.tint(Color(red: 117/255, green: 13/255, blue: 55/255, opacity: 0.5))
                                        Spacer()
                                    }
                                    
                                    VStack(alignment: .leading) {
                                        Text("Add observation (max 40 char.)")
                                            .foregroundColor(.black)
                                            .font(customFontText)
                                            .padding(.top, 20) // Adjust padding as needed
                                        
                                        TextField("", text: $annotationText)
                                            .padding()
                                            .background(
                                                RoundedRectangle(cornerRadius: 5)
                                                    .fill(Color.white.opacity(0.8))
                                            )
                                            .foregroundColor(.black)
                                            .onChange(of: annotationText) {
                                                if annotationText.count > 40 {
                                                    annotationText = String(annotationText.prefix(40))
                                                }
                                            }
                                    }
                                    //                                    .padding(.horizontal, 8) // Adjust padding as needed
                                    //                                    .overlay(
                                    //                                        Text("Add observation (max 40 char.)")
                                    //                                            .foregroundColor(.black)
                                    //                                            .opacity(annotationText.isEmpty ? 1 : 0)
                                    //                                            .padding(.horizontal, 8) // Adjust padding as needed
                                    //                                    )
                                }
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
                            .opacity(0.85)
                            .cornerRadius(15)
                            .offset(y: -100)
                            .padding(.bottom, -100)
                            .shadow(color: .black, radius: 10, x: 0, y: 5)
                            
                            // camera capture
                            VStack {
                                HStack {
                                    ZStack {
                                        Button(action: {bluetoothModel.forceCameraCapture()}) {
                                            Image("camera.circle.fill")
                                                .resizable()
                                                .frame(width: 100, height: 100)
                                                .scaledToFit()
                                                .foregroundColor(Color(red: 0x3D / 255, green: 0xA5 / 255, blue: 0xD9 / 255))
//                                                .shiny(.glossy(.black))
                                        }
                                        .frame(width: 100, height: 100)
                                        .padding()
                                        
                                        Image("bee")
                                            .resizable()
                                            .aspectRatio(contentMode: .fit)
                                            .frame(width: 100, height: 100)
                                            .rotationEffect(.degrees(45))
                                            .offset(x: 35, y: -35)
//                                            .shadow(color: .white, radius: 3, x: 0, y: 2)
                                            .scaleEffect(beeScale)
                                            .onAppear {
                                                withAnimation(Animation.easeInOut(duration: 1.0).repeatForever()) {
                                                    self.beeScale = 1.08
                                                }
                                            }
                                        
                                    }
                                    
                                    Text("Camera Capture").font(customFontTextBold).padding()
                                }.padding(.vertical, 15)
                                    .frame(
                                        maxHeight: .infinity,
                                        alignment: .center
                                    )
                                    .background(Color(white:0.89))
                                    .cornerRadius(15)
                                
                                
                            }.padding(.vertical, 20)
                            
                            // statuses dropdown
                            StatusesView()
                            Spacer()
                            // sensor readings dropdown
                            SensorReadingView()
                            Spacer()
                            // ranging dropdown
                            RangingView()
//                            Spacer()
                            // nearby devices dropdown
//                            NearbyDevices()
                            
                            
//                            Image("BuzzCam Logo 1").resizable()
//                                .frame(width: 100, height: 100)
//                                .scaledToFit().padding() // replace with transparent background logo
                        }.background(Color(red: 36/255, green: 18/255, blue: 21/255))
                    } else {
                        // Placeholder view while the peripheral name is not available, waiting for it to load
                        VStack {
                            Spacer()
                            Text("Loading...").font(customFontTextBold).foregroundColor(.white)
                            Spacer()
                        }.padding(.top,300).frame(
                            maxHeight: .infinity,
                            alignment: .center
                        ) // This ensures the VStack fills the screen
                    }
                })
            }
        }.navigationViewStyle(StackNavigationViewStyle())
    }
}


#Preview {
    MainView(connected: .constant(true))
}
