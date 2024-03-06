//
//  PresetView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 3/6/24.
//

import SwiftUI

struct PresetView: View {
    
    @EnvironmentObject var bluetoothModel: BluetoothModel

    
    @State private var annotationText: String = ""

    let customFontTitle = Font.custom("Futura-Bold", size: 25) // Define a custom font
    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 40) // Define a custom font
    
    
    
    var body: some View {
        VStack (alignment: .leading) {

            Text("Preset").font(customFontTextBold)
            
            HStack {
                Button(action: {
                    bluetoothModel.markUpdates(annotationText: "Native", beep: false)
                    // clear annotation text
                    annotationText = ""
                }) {
                    Text("Native")
                        .font(customFontText)
                        .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20)) // Adjusted padding for thinner buttons
                        .background(Color.green.opacity(0.5))
                        .cornerRadius(5)
                }
                
                Spacer()
                
                Button(action: {
                    bluetoothModel.markUpdates(annotationText: "Invasive", beep: false)
                    // clear annotation text
                    annotationText = ""
                }) {
                    Text("Invasive")
                        .font(customFontText)
                        .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20)) // Adjusted padding for thinner buttons
                        .background(Color.red.opacity(0.5))
                        .cornerRadius(5)
                }
                Spacer()
            }

        }.padding(.bottom, 20)
    }
}

#Preview {
    PresetView()
}
