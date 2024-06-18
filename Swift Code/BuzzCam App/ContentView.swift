//
//  ContentView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 10/17/23.
//

// tabview from https://medium.com/geekculture/custom-tabbar-in-swiftui-4d239410ee73

import SwiftUI


struct ContentView: View {
    @StateObject var bluetoothModel = BluetoothModel() // Initialize bluetoothModel, will be passed down to children
    @State var connected = false // When connected, display correct view, default show DiscoverView
    @State private var showMainContent = false // Bool for splash screen
    @State var selectedTab = TabbedItems.home // Default selected tab is home
    @ObservedObject var store = PresetButtonStore()


    var body: some View {
        NavigationView {
            Group {
                if showMainContent {
                    if bluetoothModel.isConnected { // if connected, show main view
                        ZStack(alignment: .bottom) {
                            // Show corresponding view based on selected tab
                            switch selectedTab {
                            case .home:
                                MainView(connected: $connected, store: store).environmentObject(bluetoothModel)
                            case .conf:
                                ConfigView().environmentObject(bluetoothModel)
                            case .cam:
                                CameraViewTab().environmentObject(bluetoothModel)
                            }
                            

                            VStack {
                                Spacer()
                                HStack {
                                    ForEach(TabbedItems.allCases, id: \.self) { item in
                                        Button {
                                            selectedTab = item // Update selected tab when tapped
                                        } label: {
                                            CustomTabItem(imageName: item.iconName, title: item.title, isActive: (selectedTab == item))
                                        }
                                        .padding(6)
                                    }
                                }
                                .background(Color(red: 117/255, green: 13/255, blue: 55/255, opacity: 0.5))
                                .cornerRadius(35)
                                .padding(.horizontal, 27)
                                .padding(.bottom, 20) // Add padding to the bottom of the tab bar
                            }
                        }.foregroundColor(Color.black)
                        .edgesIgnoringSafeArea(.bottom) // Ignore safe area for bottom of the screen
                    } else { // show discover devices
                        DiscoverView(connected: $connected).environmentObject(bluetoothModel)
                    }
                } 
                else { // Show splash screen
                    // Launch screen
                    withAnimation {
                        ZStack {
                            Color.black
                                .edgesIgnoringSafeArea(.all) // Set the background color
                                .opacity(showMainContent ? 0 : 1) // Fade out background color
    
                            Image("BuzzCam Logo 2")
                                .resizable()
                                .scaledToFit()
                                .frame(width: 200, height: 200)
                                .edgesIgnoringSafeArea(.all)
                                .aspectRatio(contentMode: .fill)
                                .opacity(showMainContent ? 0 : 1) // Fade out when showMainContent is true
                        }
                    }
                    .animation(.easeInOut(duration: 0.5)) // Add fade animation
                    .onAppear {
                        // Show launch screen for 3 seconds
                        DispatchQueue.main.asyncAfter(deadline: .now() + 1) { // 1 second splash screen
                            withAnimation {
                                self.showMainContent = true
                            }
                        }
                    }
                }

            }
            .background(Color.clear) // Set the background color of the Group to clear
        }.navigationViewStyle(StackNavigationViewStyle())
    }
}


enum TabbedItems: CaseIterable {
    case home
    case conf
    case cam
    
    var title: String {
        switch self {
        case .home:
            return "Home"
        case .conf:
            return "Config"
        case .cam:
            return "Camera"
        }
    }
    
    var iconName: String {
        switch self {
        case .home:
            return "house.fill"
        case .conf:
            return "gearshape.fill"
        case .cam:
            return "camera.fill"
        }
        
    }
}

extension TabbedItems: Identifiable {
    var id: TabbedItems { self }
}

extension ContentView {
    func CustomTabItem(imageName: String, title: String, isActive: Bool) -> some View {
        HStack {
            Spacer()
            Image(systemName: imageName)
                .resizable()
                .aspectRatio(contentMode: .fit)
                .frame(width: 20, height: 20) // Adjust the size of the image
                .foregroundColor(isActive ? .white : .gray)
            if isActive {
                Text(title)
                    .font(.system(size: 14))
                    .foregroundColor(isActive ? .white : .gray)
            }
            Spacer()
        }
        .frame(height: 60) // Adjust the height of the tab item
        .background(isActive ? Color(red: 117/255, green: 13/255, blue: 55/255, opacity: 0.7) : Color.clear)
        .cornerRadius(30)
    }
}

#Preview {
    ContentView()
}

