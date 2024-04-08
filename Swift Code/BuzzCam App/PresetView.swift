////
////  PresetView.swift
////  BuzzCam App
////
////  Created by Responsive Environments on 3/6/24.
////

import SwiftUI
import Foundation

extension Color {
    init(hex: String) {
        let scanner = Scanner(string: hex.trimmingCharacters(in: CharacterSet.alphanumerics.inverted))
        var rgb: UInt64 = 0
        
        guard scanner.scanHexInt64(&rgb) else {
            self.init(red: 0, green: 0, blue: 0)
            return
        }
        
        let red = Double((rgb & 0xFF0000) >> 16) / 255.0
        let green = Double((rgb & 0x00FF00) >> 8) / 255.0
        let blue = Double(rgb & 0x0000FF) / 255.0
        
        self.init(red: red, green: green, blue: blue)
    }
}

struct PresetView: View {
    
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    @Binding var presetButtons: [PresetButton]
    @State private var annotationText: String = ""
    @State private var isAddButtonPopupVisible = false
    @State private var newButtonName = ""
    @State private var newButtonBeep = false
    @State private var newButtonColor = "#0000FF"
    @State private var newButtonDescription = ""
    
    @State private var isDeleteSheetVisible = false // Track whether delete sheet is visible
    @State private var selectedPresetIDs = Set<UUID>() // Track selected preset IDs for deletion
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 40)
    
    @Environment(\.scenePhase) private var scenePhase
    let saveAction: () -> Void
    @StateObject private var store = PresetButtonStore()

    var body: some View {
        VStack(alignment: .leading) {
            Text("Preset").font(customFontTextBold)
            LazyVGrid(columns: [
                GridItem(.flexible(minimum: 100, maximum: .infinity)),
                GridItem(.flexible(minimum: 100, maximum: .infinity))
            ], spacing: 10) {
                ForEach(presetButtons) { button in
                    Button(action: {
                        bluetoothModel.markUpdates(annotationText: button.name, beep: button.beep)
                        annotationText = ""
                    }) {
                        Text(button.name)
                            .font(customFontText)
                            .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20))
                            .background(Color(hex: button.color).opacity(0.5))
                            .cornerRadius(5)
                    }
                }
            }
            .padding(.bottom, 10)
            
            Button(action: {
                // Show popup to add new preset button
                isAddButtonPopupVisible = true
            }) {
                Text("Add Preset")
                    .font(customFontText)
                    .foregroundColor(.blue)
            }
            
            Button(action: {
                            // Show delete sheet
                            isDeleteSheetVisible = true
                        }) {
                            Text("Delete Preset")
                                .font(customFontText)
                                .foregroundColor(.red)
                        }
                    }
                    .padding(.bottom, 20)
                    .onChange(of: scenePhase) {
                        if scenePhase == .inactive { saveAction() }
                    }.sheet(isPresented: $isAddButtonPopupVisible) {
                                    // Popup view to add new preset button
                                    VStack (alignment: .leading) {
                                        Text("Add Presets")
                                            .font(customFontTextBold)
                                            .padding()
                                        
                                        HStack {
                                            Text("Name").font(customFontText)
                                            TextField("Name", text: $newButtonName).font(customFontText)
                                                .padding(.bottom)
                                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                        }
                                        
                                        VStack{
                                            HStack {
                                                Text("Beep").font(customFontText)
                                                Toggle("Beep", isOn: $newButtonBeep).labelsHidden()
                                                    .padding(.bottom)
                                                Spacer()
                                            }
                                        }
                                        HStack {
                                            Text("Color (hex)").font(customFontText)
                                            TextField("Color (hex)", text: $newButtonColor)
                                                .padding(.bottom)
                                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                        }
                                        
                                        HStack {
                                            Text("Description").font(customFontText)
                                            TextField("Description", text: $newButtonDescription)
                                                .padding(.bottom)
                                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                        }
                                        
                                        
                                        Button(action: {
                                            // Add new preset button
                                            let newButton = PresetButton(name: newButtonName, color: newButtonColor, description: newButtonDescription, beep: newButtonBeep)
                                            presetButtons.append(newButton)
                                            isAddButtonPopupVisible = false
                                            // Clear input fields
                                            newButtonName = ""
                                            newButtonBeep = false
                                            newButtonColor = "#0000FF"
                                            newButtonDescription = ""
                                        }) {
                                            Text("Save")
                                                .font(customFontText).fontWeight(.bold)
                                                .padding()
                                                .frame(maxWidth: .infinity)
//                                                .background(Color.blue)
//                                                .opacity(0.8)
                                                .foregroundColor(.blue)
                                                .cornerRadius(10)
                                        }
                                    }.environment(\.colorScheme, .light)
                                    .presentationDetents([.medium])
                                    .padding()
                                    .background(Color.white)
                                    .cornerRadius(10)
                                }
                    .sheet(isPresented: $isDeleteSheetVisible) {
                        // Sheet for deleting presets
                        VStack {
                            Text("Delete Presets")
                                .font(customFontTextBold)
                                .padding(.bottom)
                            
                            ScrollView {
                                ForEach(presetButtons) { button in
                                    HStack {
                                        Text(button.name).font(customFontText)
                                        Spacer()
                                        // Checkbox for selecting preset
                                        Image(systemName: selectedPresetIDs.contains(button.id) ? "checkmark.square" : "square")
                                            .resizable()
                                            .frame(width: 20, height: 20)
                                            .onTapGesture {
                                                // Toggle selection
                                                if selectedPresetIDs.contains(button.id) {
                                                    selectedPresetIDs.remove(button.id)
                                                } else {
                                                    selectedPresetIDs.insert(button.id)
                                                }
                                            }
                                    }
                                    .padding()
                                    .onTapGesture {
                                        // Toggle selection on tap
                                        if selectedPresetIDs.contains(button.id) {
                                            selectedPresetIDs.remove(button.id)
                                        } else {
                                            selectedPresetIDs.insert(button.id)
                                        }
                                    }
                                }
                            }
                            
                            // Delete button
                            Button(action: {
                                // Remove selected presets from presetButtons
                                presetButtons.removeAll { button in
                                    selectedPresetIDs.contains(button.id)
                                }
                                // Dismiss the sheet
                                isDeleteSheetVisible = false
                            }) {
                                Text("Delete Selected")
                                    .font(customFontText).fontWeight(.bold)
                                    .padding()
                                    .frame(maxWidth: .infinity)
//                                    .background(Color.red)
//                                    .opacity(0.8)
                                    .foregroundColor(.red)
                                    .cornerRadius(10)
                            }
                        }
                        .presentationDetents([.medium])
                        .padding()
                        .background(Color.white)
                        .cornerRadius(10)
                    }
    }
}


////////////// WITHOUT DELETE FUNCTIONALITY
//struct PresetView: View {
//    
//    @EnvironmentObject var bluetoothModel: BluetoothModel
//    
//    @Binding var presetButtons: [PresetButton]
//    @State private var annotationText: String = ""
//    @State private var isAddButtonPopupVisible = false
//    @State private var newButtonName = ""
//    @State private var newButtonBeep = false
//    @State private var newButtonColor = "#0000FF"
//    @State private var newButtonDescription = ""
//    
//    let customFontTitle = Font.custom("Futura-Bold", size: 25) // Define a custom font
//    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
//    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
//    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 40) // Define a custom font
//    
//    @Environment(\.scenePhase) private var scenePhase
//    let saveAction: ()->Void
//
//    var body: some View {
//        VStack (alignment: .leading) {
//            Text("Preset").font(customFontTextBold)
//            LazyVGrid(columns: [
//                            GridItem(.flexible(minimum: 100, maximum: .infinity)), // Flexible column to fit multiple buttons
//                            GridItem(.flexible(minimum: 100, maximum: .infinity)) // Add more columns if needed
//                        ], spacing: 10) { // Add spacing between buttons
//                            ForEach(presetButtons) { button in
//                                Button(action: {
//                                    bluetoothModel.markUpdates(annotationText: button.name, beep: button.beep)
//                                    // clear annotation text
//                                    annotationText = ""
//                                }) {
//                                    Text(button.name)
//                                        .font(customFontText)
//                                        .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20))
//                                        .background(Color(hex: button.color).opacity(0.5)) // Convert hex string to Color
//                                        .cornerRadius(5)
//                                }
//                            }
//                        }
//                        .padding(.bottom, 10) // Add vertical padding
//                        
//                Button(action: {
//                    // Show popup to add new preset button
//                    isAddButtonPopupVisible = true
//                }) {
//                    Text("Add Preset")
//                        .font(customFontText)
//                        .foregroundColor(.blue)
//                }
//            
//        }.padding(.bottom, 20)
//        .onChange(of: scenePhase) {
//            if scenePhase == .inactive { saveAction() }
//        }
//        .sheet(isPresented: $isAddButtonPopupVisible) {
//            // Popup view to add new preset button
//            VStack {
//                TextField("Name", text: $newButtonName)
//                    .padding()
//                    .textFieldStyle(RoundedBorderTextFieldStyle())
//                Toggle("Beep", isOn: $newButtonBeep)
//                    .padding()
//                TextField("Color (hex)", text: $newButtonColor)
//                    .padding()
//                    .textFieldStyle(RoundedBorderTextFieldStyle())
//                TextField("Description", text: $newButtonDescription)
//                    .padding()
//                    .textFieldStyle(RoundedBorderTextFieldStyle())
//                Button(action: {
//                    // Add new preset button
//                    let newButton = PresetButton(name: newButtonName, color: newButtonColor, description: newButtonDescription, beep: newButtonBeep)
//                    presetButtons.append(newButton)
//                    isAddButtonPopupVisible = false
//                    // Clear input fields
//                    newButtonName = ""
//                    newButtonBeep = false
//                    newButtonColor = "#0000FF"
//                    newButtonDescription = ""
//                }) {
//                    Text("Save")
//                        .padding()
//                        .frame(maxWidth: .infinity)
//                        .background(Color.blue)
//                        .foregroundColor(.white)
//                        .cornerRadius(10)
//                }
//            }.environment(\.colorScheme, .light)
//            .padding()
//            .background(Color.white)
//            .cornerRadius(10)
//        }
//    }
//}



//
//import SwiftUI
//import Foundation
//
//struct PresetView: View {
//    
//    @EnvironmentObject var bluetoothModel: BluetoothModel
//
//    @Binding var presetButtons: [PresetButton]
//    @State private var annotationText: String = ""
//
//    let customFontTitle = Font.custom("Futura-Bold", size: 25) // Define a custom font
//    let customFontText = Font.custom("AvenirNext-Regular", size: 18) // Define a custom font
//    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 23) // Define a custom font
//    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 40) // Define a custom font
//    
////    @State private var presetButtons: [PresetButton] = UserDefaults.standard.data(forKey: "presetButtons").flatMap { try? JSONDecoder().decode([PresetButton].self, from: $0) } ?? []
//    
//    @Environment(\.scenePhase) private var scenePhase
//    let saveAction: ()->Void
//
//    @State private var isPopupPresented = false
//    
//    
//    var body: some View {
//        VStack (alignment: .leading) {
//            Text("Preset").font(customFontTextBold)
//            Button("Add preset") {
//                // Show the pop-up for adding a new schedule
////                selectedIndex = nil
//                isPopupPresented.toggle()
//                print("popup toggled")
//            }.foregroundColor(.blue)
//            
//            // Unified view for editing/creating schedules
//            PresetPopupView(
//                isPresented: $isPopupPresented,
//                onSave: { newPresetButton in
//                    presetButtons.append(newPresetButton)
//                    isPopupPresented = false
//                }
//            )
//            .opacity(isPopupPresented ? 1 : 0) // Optionally, fade out when not presented
//            .animation(.easeInOut, value: 1)
//            
//            HStack {
//                ForEach(presetButtons) { button in
//                    Button(action: {
//                        bluetoothModel.markUpdates(annotationText: button.name, beep: button.beep)
//                        // clear annotation text
//                        annotationText = ""
//                    }) {
//                        Text(button.name)
//                            .font(customFontText)
//                            .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20))
//                            .background(Color(button.color).opacity(0.5))
//                            .cornerRadius(5)
//                    }
//                    Spacer()
//                }
//            }
//        }.padding(.bottom, 20)
//        .onChange(of: scenePhase) {
//            if scenePhase == .inactive { saveAction() }
//        }
////        VStack (alignment: .leading) {
////
////            Text("Preset").font(customFontTextBold)
////            
////            HStack {
////                Button(action: {
////                    bluetoothModel.markUpdates(annotationText: "Native", beep: false)
////                    // clear annotation text
////                    annotationText = ""
////                }) {
////                    Text("Native")
////                        .font(customFontText)
////                        .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20)) // Adjusted padding for thinner buttons
////                        .background(Color.green.opacity(0.5))
////                        .cornerRadius(5)
////                }
////                
////                Spacer()
////                
////                Button(action: {
////                    bluetoothModel.markUpdates(annotationText: "Invasive", beep: false)
////                    // clear annotation text
////                    annotationText = ""
////                }) {
////                    Text("Invasive")
////                        .font(customFontText)
////                        .padding(EdgeInsets(top: 5, leading: 20, bottom: 5, trailing: 20)) // Adjusted padding for thinner buttons
////                        .background(Color.red.opacity(0.5))
////                        .cornerRadius(5)
////                }
////                Spacer()
////            }
////
////        }.padding(.bottom, 20)
//    }
//}
//
//struct PresetPopupView: View {
//    @Binding var isPresented: Bool
//    var onSave: (PresetButton) -> Void
//    var newButton = PresetButton
//    
//    init(isPresented: Binding<Bool>, onSave: @escaping (PresetButton) -> Void) {
//            self._isPresented = isPresented
//            self.onSave = onSave
//        }
//    
//    var body: some View {
//        VStack {
//            Text(selectedIndex != nil ? "Edit Schedule" : "Add Schedule")
//                .font(.title)
//                .padding()
//            
//            // Checkboxes for days
//            ForEach(DaysOfWeek.allCases, id: \.self) { day in
//                Button(action: {
//                    toggleDay(day)
//                }) {
//                    HStack {
//                        Image(systemName: editedSchedule.isDaySelected(day) ? "checkmark.square" : "square")
//                            .resizable()
//                            .frame(width: 20, height: 20)
//                        Text(day.rawValue.prefix(3))
//                    }
//                    .padding(.vertical, 5)
//                }
//            }
//            
//            // Time picker
//            HStack {
//                Text("Start Time:")
//                TimePicker(selectedHour: $editedSchedule.startHour, selectedMinute: $editedSchedule.startMinute)
//            }
//            
//            HStack {
//                Text("End Time:")
//                TimePicker(selectedHour: $editedSchedule.stopHour, selectedMinute: $editedSchedule.stopMinute)
//            }
//            
//            
//            Button("Delete") {
//                onDelete?() // Call delete closure if provided
//                isPresented.toggle()
//
//            }
//            .foregroundColor(.red) // Make delete button red
//            .padding()
//            
//            Button("Save") {
//                onSave(editedSchedule)
//                isPresented.toggle()
//                isPresented.toggle()
//            }
//            .padding()
//            
//            Button("Cancel") {
//                isPresented.toggle()
//            }
//            .padding()
//        }
//        .onChange(of: selectedIndex) {
//            // Update editedSchedule when selectedIndex changes
//            if selectedIndex != nil {
//                editedSchedule = originalSchedule
//            } else {
//                editedSchedule = ScheduleConfig()
//            }
//        }
//        .onAppear {
//            // Perform any setup if needed when the view appears
//            if selectedIndex != nil {
//                // If editing an existing schedule, update editedSchedule with the original schedule
//                editedSchedule = originalSchedule
//                print("og schedule")
//            } else {
//                // If adding a new schedule, update editedSchedule with an empty ScheduleConfig
//                editedSchedule = ScheduleConfig()
//                print("new schedule")
//            }
//        }
//        .frame(width: 300, height: 700)
//        .background(Color.white)
//        .cornerRadius(10)
//        .overlay(
//            RoundedRectangle(cornerRadius: 10)
//                .stroke(Color.gray, lineWidth: 2)
//        )
//        .shadow(radius: 5)
//        .padding()
//        .padding(.bottom, isPresented ? 80 : 0)
//    }
//    
//    private func toggleDay(_ day: DaysOfWeek) {
//        editedSchedule.toggleDay(day)
//    }
//}
//
//
////#Preview {
////    PresetView()
////}
