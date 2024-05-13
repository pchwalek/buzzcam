//
//  SchedulesView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 12/12/23.
//

import SwiftUI
import Combine

struct SchedulesView: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel
    @State private var isExpanded = false
    @State private var cancellables: Set<AnyCancellable> = Set()
    @State private var schedules: [ScheduleConfig] = []
    @State private var selectedSchedule: ScheduleConfig?
    @State private var selectedIndex: Int?
    @State private var isPopupPresented = false
    @State private var enableFreeRunMode = false
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextSmall = Font.custom("AvenirNext-Regular", size: 12)

    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Schedules")
                    .font(customFontTextBoldLarge)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(
                GeometryReader { proxy in
                        Image("IMG_4587 (6)")
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: proxy.size.width, height: proxy.size.height)
                            .clipped()
                            .opacity(0.7)
                            .allowsHitTesting(false) // Prevents the image from capturing taps
                            .contentShape(Rectangle()) // Set content shape to Rectangle to allow tap gesture
                    }).onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                    if !isExpanded {
                        isPopupPresented = false
                    }
                }
            }
            
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    VStack (alignment: .leading) {
                        HStack {
                            Text("Free run mode")
                                .font(customFontTextBoldSmall)
                                .padding(.horizontal)
                            
                            Toggle("", isOn: $enableFreeRunMode)
                                .labelsHidden()
                                .onChange(of: enableFreeRunMode) {
                                    // Call your function when the toggle is changed
                                    bluetoothModel.enableFreeRunMode(enableFreeRunMode: enableFreeRunMode)
                                }
                        }
                        
                        Text("Free run mode bypasses any schedules and runs continuously")
                            .font(customFontText).padding(.bottom, 10).padding(.horizontal)
                    }
                    
                    VStack(alignment: .leading) {
                        Button("Add Schedule") {
                            // Show the pop-up for adding a new schedule
                            selectedIndex = nil
                            isPopupPresented.toggle()
                            print("add")
                        }.font(customFontText).padding()
                            .background(Color(white: 0.8))
                            .cornerRadius(5)
                            
                            
                            
                        
                        ForEach(schedules.indices, id: \.self) { index in
                            VStack {
                                HStack {
                                    Text("Schedule #\(index)").font(customFontTextBoldSmall)
                                    Spacer()
                                    Button("Edit") {
                                        // Show the pop-up for editing the schedule
                                        selectedIndex = index
                                        print("index: \(index)")
                                        isPopupPresented.toggle()
                                        print("edit")
                                    }.font(customFontText).foregroundColor(.blue)
                                }
                                Text("Days: \(selectedDaysString(schedule: schedules[index]))").font(customFontText)
                                Text("Time: \(selectedTimeString(schedule: schedules[index]))").font(customFontText)
                            }
                        }
                    }
                    .padding()
                    .frame(
                        minWidth: 0,
                        maxWidth: .infinity,
                        alignment: .leading)
                    .background(Color(white: 0.98))
                    .cornerRadius(10)
                }
                .padding()
            }
            if !isPopupPresented {
                HStack {
                    Spacer()
                    VStack (alignment: .center){
                        Text("Made with ❤️ by the Responsive Environments Lab").font(customFontTextSmall).foregroundColor(.black)
                        Text("Contributors: Patrick Chwalek, Isamar Zhu").font(customFontTextSmall).foregroundColor(.black)
                    }
                    Spacer()
                }.padding(.top, 20)

            }
            // Unified view for editing/creating schedules
            SchedulePopupView(
                isPresented: $isPopupPresented,
                selectedIndex: selectedIndex,
                schedule: selectedIndex != nil ? schedules[selectedIndex!] : nil,
                onSave: { newSchedule in
                    if let index = selectedIndex {
                        schedules[index] = newSchedule
                    } else {
                        schedules.append(newSchedule)
                    }
                    bluetoothModel.sendSchedules(schedules)
                    isPopupPresented = false
                },
                onDelete: {
                        if let index = selectedIndex {
                            if index < schedules.count {
                                schedules.remove(at: index)
                                bluetoothModel.sendSchedules(schedules)
                                
                                // Adjust selected index if it's out of bounds after deletion
                                if selectedIndex! >= schedules.count {
                                    selectedIndex = nil
                                }
                            }
                        }
                    }
            )
            .opacity(isPopupPresented ? 1 : 0) // Optionally, fade out when not presented
            .animation(.easeInOut, value: 1)
        
        }
        .onAppear {
            // Initialize schedules when the view appears
            schedules = bluetoothModel.configPacketData_Schedule?.scheduleConfig ?? []
            
            // Add an observer to monitor changes to configPacketData_Audio
            bluetoothModel.$configPacketData_Audio
                .sink { configPacketData_Audio in
                    self.updateFreeRunMode(configPacketData_Audio)
                }
                .store(in: &cancellables)
            
            // Trigger the initial update
            self.updateFreeRunMode(bluetoothModel.configPacketData_Audio)

        }
        .frame(maxWidth: .infinity)
        .background(Color(white:0.90))
    }
    
    private func selectedDaysString(schedule: ScheduleConfig) -> String {
        let selectedDays = ["S", "M", "T", "W", "Th", "F", "Sa"]
            .filter { day in
                switch day {
                case "S": return schedule.sunday
                case "M": return schedule.monday
                case "T": return schedule.tuesday
                case "W": return schedule.wednesday
                case "Th": return schedule.thursday
                case "F": return schedule.friday
                case "Sa": return schedule.saturday
                default: return false
                }
            }
        return selectedDays.joined(separator: ", ")
    }
    
    private func selectedTimeString(schedule: ScheduleConfig) -> String {
        return String(format: "%02d:%02d - %02d:%02d",
                      schedule.startHour, schedule.startMinute,
                      schedule.stopHour, schedule.stopMinute)
    }

    
    private func updateFreeRunMode(_ configPacketData_Audio: ConfigPacketData_Audio?) {
        enableFreeRunMode = configPacketData_Audio?.freeRunMode ?? false
    }

}

struct SchedulePopupView: View {
    @Binding var isPresented: Bool
    @State private var editedSchedule: ScheduleConfig
    let originalSchedule: ScheduleConfig
    let selectedIndex: Int?
    var onSave: (ScheduleConfig) -> Void
    var onDelete: (() -> Void)? // Closure for delete action
    
    let customFontTitle = Font.custom("Futura-Bold", size: 25)
    let customFontText = Font.custom("AvenirNext-Regular", size: 18)
    let customFontTextBold = Font.custom("AvenirNext-DemiBold", size: 20)
    let customFontTextBoldLarge = Font.custom("AvenirNext-DemiBold", size: 25)
    let customFontTextBoldSmall = Font.custom("AvenirNext-DemiBold", size: 18)
    
    
    init(isPresented: Binding<Bool>, selectedIndex: Int?, schedule: ScheduleConfig? = nil, onSave: @escaping (ScheduleConfig) -> Void, onDelete: (() -> Void)? = nil) {
            self._isPresented = isPresented
            self.originalSchedule = schedule ?? ScheduleConfig()
            self._editedSchedule = State(initialValue: selectedIndex != nil ? (schedule ?? ScheduleConfig()) : ScheduleConfig())
            self.selectedIndex = selectedIndex
            self.onSave = onSave
            self.onDelete = onDelete
        }
    
    var body: some View {
        VStack {
            Text(selectedIndex != nil ? "Edit Schedule" : "Add Schedule")
                .font(customFontTextBoldLarge)
                .padding(.top)
            
            // Checkboxes for days
            ForEach(DaysOfWeek.allCases, id: \.self) { day in
                Button(action: {
                    toggleDay(day)
                }) {
                    HStack {
                        Image(systemName: editedSchedule.isDaySelected(day) ? "checkmark.square" : "square")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(day.rawValue.prefix(3)).font(customFontText)
                    }
                    .padding(.vertical, 5)
                }
            }
            
            // Time picker
            HStack {
                Text("Start").font(customFontText).padding()
                TimePicker(selectedHour: $editedSchedule.startHour, selectedMinute: $editedSchedule.startMinute)
            }
            
            HStack {
                Text("End").font(customFontText).padding()
                TimePicker(selectedHour: $editedSchedule.stopHour, selectedMinute: $editedSchedule.stopMinute)
            }
            
            
            Button("Delete") {
                onDelete?() // Call delete closure if provided
                isPresented.toggle()

            }.font(customFontText)
            .foregroundColor(.red) // Make delete button red
            .padding()
            
            Button("Save") {
                onSave(editedSchedule)
                isPresented.toggle()
                isPresented.toggle()
            }.font(customFontText)
                .foregroundColor(.blue)
            .padding()
            
            Button("Cancel") {
                isPresented.toggle()
            }.font(customFontText)
            .padding()
        }
        .onChange(of: selectedIndex) {
            // Update editedSchedule when selectedIndex changes
            if selectedIndex != nil {
                editedSchedule = originalSchedule
            } else {
                editedSchedule = ScheduleConfig()
            }
        }
        .onAppear {
            // Perform any setup if needed when the view appears
            if selectedIndex != nil {
                // If editing an existing schedule, update editedSchedule with the original schedule
                editedSchedule = originalSchedule
                print("og schedule")
            } else {
                // If adding a new schedule, update editedSchedule with an empty ScheduleConfig
                editedSchedule = ScheduleConfig()
                print("new schedule")
            }
        }
        .frame(width: 300, height: 600)
        .background(Color.white)
        .cornerRadius(10)
        .overlay(
            RoundedRectangle(cornerRadius: 10)
                .stroke(Color.gray, lineWidth: 2)
        )
        .shadow(radius: 5)
        .padding()
        .padding(.bottom, isPresented ? 80 : 0)
    }
    
    private func toggleDay(_ day: DaysOfWeek) {
        editedSchedule.toggleDay(day)
    }
}

struct TimePicker: View {
    @Binding var selectedHour: UInt32
    @Binding var selectedMinute: UInt32
    
    var body: some View {
        HStack {
            Picker("", selection: $selectedHour) {
                ForEach(0..<24, id: \.self) { hour in
                    Text(String(format: "%02d", hour))// Format display for 00:00
                        .tag(UInt32(hour))
                        .foregroundColor(.black)
                }
            }
            .pickerStyle(WheelPickerStyle())
            .frame(width: 80, height: 40)
            
            Text(":")
            
            Picker("", selection: $selectedMinute) {
                ForEach(0..<60, id: \.self) { minute in
                    Text(String(format: "%02d", minute))
                        .tag(UInt32(minute))
                        .foregroundColor(.black)
                }
            }
            .pickerStyle(WheelPickerStyle())
            .frame(width: 80, height: 40)

        }
        .environment(\.colorScheme, .light)
        .padding()
    }
}

struct WheelPicker: View {
    let range: Range<Int>
    @Binding var selection: Int
    let label: String
    
    var body: some View {
        VStack {
            Text(label)
            Picker("", selection: $selection) {
                ForEach(range, id: \.self) { value in
                    Text("\(value)").tag(value)
                }
            }
            .pickerStyle(WheelPickerStyle())
            .frame(width: 80, height: 100)
        }
    }
}

extension ScheduleConfig {
    func isDaySelected(_ day: DaysOfWeek) -> Bool {
        switch day {
        case .sunday: return sunday
        case .monday: return monday
        case .tuesday: return tuesday
        case .wednesday: return wednesday
        case .thursday: return thursday
        case .friday: return friday
        case .saturday: return saturday
        }
    }
    
    mutating func toggleDay(_ day: DaysOfWeek) {
        switch day {
        case .sunday: sunday.toggle()
        case .monday: monday.toggle()
        case .tuesday: tuesday.toggle()
        case .wednesday: wednesday.toggle()
        case .thursday: thursday.toggle()
        case .friday: friday.toggle()
        case .saturday: saturday.toggle()
        }
    }
}

enum DaysOfWeek: String, CaseIterable, Identifiable {
    case sunday = "Sun"
    case monday = "Mon"
    case tuesday = "Tue"
    case wednesday = "Wed"
    case thursday = "Thu"
    case friday = "Fri"
    case saturday = "Sat"
    
    var id: String { self.rawValue }
}

#Preview {
    SchedulesView()
}
