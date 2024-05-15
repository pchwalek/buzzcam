//
//  ConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/23/23.
//

import SwiftUI

struct ConfigView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 34)
    
    var body: some View {
        
        ScrollView(showsIndicators: false) {
            ScrollViewReader(content: { proxy in
                if (bluetoothModel.updatedConfigPacket) {
                    
                    Text("Configuration").font(customFontTextBoldLarge).font(customFontTextBold).foregroundColor(.white).padding()
                    AudioConfigView()
                    Spacer()
                    SensingConfigView()
                    Spacer()
                    CameraConfigView()
                    Spacer()
                    NetworkView()
                    Spacer()
                    SystemControlView()
                    Spacer()
                    SchedulesView()

                }
                else {
                    // Placeholder view while the peripheral name is not available, waiting for it to load
                    VStack {
                        Spacer()
                        Text("Loading...").font(customFontTextBold).foregroundColor(.white)
                        Spacer()
                    }.padding(.top,300).frame(
                        maxHeight: .infinity,
                        alignment: .center
                    )
                }
            })
        }
    }
}

#Preview {
    ConfigView()
}
