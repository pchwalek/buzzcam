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
                withAnimation {
                    ZStack {
                        Color.black
                            .edgesIgnoringSafeArea(.all) // Set the background color
//                            .opacity(showMainContent ? 0 : 1) // Fade out background color
                        
                        Image("BuzzCam Logo 2")
                            .resizable()
                            .scaledToFit()
                            .frame(width: 200, height: 200)
                            .edgesIgnoringSafeArea(.all)
                            .aspectRatio(contentMode: .fill)
//                            .opacity(showMainContent ? 0 : 1) // Fade out background color
                        
//                        VStack(spacing: 20) {
//                            Spacer().frame(height: 300)
//                            Text("BuzzCam")
//                                .font(.largeTitle)
//                                .fontWeight(.bold)
//                                .foregroundColor(.white)
////                                .opacity(showMainContent ? 0 : 1) // Fade out background color
//                        }
                    }
                }
                .onAppear {
                    // Show launch screen for 3 seconds
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1) {
                        withAnimation {
                            self.showMainContent = true
                        }
//                        self.showMainContent = true
                    }
                }
            }
        }
        
    }
}

#Preview {
    ContentView()
}
