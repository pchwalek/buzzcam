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
                Text("Configuration").font(.title).foregroundColor(Color.white)
                AudioConfigView()
                Spacer()
                SensingConfigView()
                Spacer()
                CameraConfigView()
                Spacer()
                NearbyDevicesConfig()
                Spacer()
                SchedulesView()
            })
        }
    }
}

#Preview {
    ConfigView()
}
