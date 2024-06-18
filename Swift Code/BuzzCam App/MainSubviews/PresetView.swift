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

extension Color {
    public func toHex(alpha: Bool = false) -> String {
        // Get the components of the color
        let uiColor = UIColor(self)
        var red: CGFloat = 0
        var green: CGFloat = 0
        var blue: CGFloat = 0
        var alphaValue: CGFloat = 0
        uiColor.getRed(&red, green: &green, blue: &blue, alpha: &alphaValue)
        
        // Multiply the components by 255 and convert them to hexadecimal strings
        let r = Int(red * 255)
        let g = Int(green * 255)
        let b = Int(blue * 255)
        let a = Int(alphaValue * 255)
        
        // Construct the hexadecimal string
        let hexString: String
        if alpha {
            hexString = String(format: "%02X%02X%02X%02X", r, g, b, a)
        } else {
            hexString = String(format: "%02X%02X%02X", r, g, b)
        }
        
        return hexString
    }
}




struct PresetView: View {
    
    @EnvironmentObject var bluetoothModel: BluetoothModel
    
    @Binding var presetButtons: [PresetButton]
    @State private var annotationText: String = ""
    @State private var isAddButtonPopupVisible = false
    @State private var newButtonName = ""
    @State private var newButtonBeep = false
    @State private var newButtonColor = Color.red
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
                            .frame(minWidth: 80)
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
                                            Text("Color").font(customFontText)
                                            ColorPicker("", selection: $newButtonColor).labelsHidden()
                                            Spacer()
                                        }
                                        
                                        HStack {
                                            Text("Description").font(customFontText)
                                            TextField("Description", text: $newButtonDescription)
                                                .padding(.bottom)
                                                .textFieldStyle(RoundedBorderTextFieldStyle())
                                        }
                                        
                                        
                                        Button(action: {
                                            // Add new preset button
                                            let newButton = PresetButton(name: newButtonName, color: newButtonColor.toHex(alpha: false), description: newButtonDescription, beep: newButtonBeep)
                                            presetButtons.append(newButton)
                                            isAddButtonPopupVisible = false
                                            // Clear input fields
                                            newButtonName = ""
                                            newButtonBeep = false
                                            newButtonColor = Color.red
                                            newButtonDescription = ""
                                        }) {
                                            Text("Save")
                                                .font(customFontText).fontWeight(.bold)
                                                .padding()
                                                .frame(maxWidth: .infinity)
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


//
////#Preview {
////    PresetView()
////}
