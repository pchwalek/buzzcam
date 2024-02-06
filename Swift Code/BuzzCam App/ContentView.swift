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
    
    @State private var showMainContent = false // Bool for splash screen
    
    var body: some View {
        Group {
            if showMainContent {
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
            else { // Show splash screen
                // Launch screen
                ZStack {
                    Color(red: 40/255, green: 35/255, blue: 41/255)
                        .edgesIgnoringSafeArea(.all) // Set the background color
                    
                    Image("BuzzCam Logo") // Replace "launch_image" with your image name
                        .resizable()
                        .scaledToFit()
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .edgesIgnoringSafeArea(.all)
                        .aspectRatio(contentMode: .fill)
                    
                    VStack(spacing: 20) {
                        Spacer().frame(height: 400)
                        Text("BuzzCam")
                            .font(.largeTitle)
                            .fontWeight(.bold)
                            .foregroundColor(.white)
                    }
                }
                .onAppear {
                    // Show launch screen for 3 seconds
                    DispatchQueue.main.asyncAfter(deadline: .now() + 3) {
                        withAnimation {
                            self.showMainContent = true
                        }
                    }
                }
            }
        }
        
    }
}

#Preview {
    ContentView()
}
