//
//  ContentView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/17/23.
//

import SwiftUI

struct ContentView: View {

    @StateObject var bluetoothModel = BluetoothModel()
    @State var connected = false // dictates for all views, default show discoverview
    
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
//                TestView().tabItem{Label("test tab", systemImage: "gearshape.fill")}
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
