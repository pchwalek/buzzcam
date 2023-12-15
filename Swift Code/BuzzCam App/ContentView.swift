//
//  ContentView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/17/23.
//

import SwiftUI

struct ContentView: View {
    
    @StateObject var bluetoothModel = BluetoothModel() // Initialize bluetoothModel, will be passed down to children
    @State var connected = false // When connected, display correct view, default show DiscoverView
    
    var body: some View {
        if connected { // if connected, show main view
            TabView {
                MainView(connected: $connected).environmentObject(bluetoothModel)
                    .tabItem {
                        Label("Main", systemImage: "house.fill")
                    }
                
                ConfigView().environmentObject(bluetoothModel)
                    .tabItem {
                        Label("Config", systemImage: "gearshape.fill")
                    }
            }
        }
        else { // show discover devices
            DiscoverView(connected: $connected).environmentObject(bluetoothModel)
        }
        
    }
}

#Preview {
    ContentView()
}
