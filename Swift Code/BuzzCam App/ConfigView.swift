//
//  ConfigView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/23/23.
//

import SwiftUI

struct ConfigView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    var body: some View {
        
        ScrollView(showsIndicators: false) {
            ScrollViewReader(content: { proxy in
                if (bluetoothModel.updatedConfigPacket) {
                    
                    Text("Configuration").font(.title).foregroundColor(Color.white)
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
                        Text("Loading...").foregroundColor(.white)
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
