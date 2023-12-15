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
    @State private var schedules: [ScheduleConfig] = []
    @State private var selectedSchedule: ScheduleConfig?
    @State private var selectedIndex: Int?
    @State private var isPopupPresented = false
    
    
    
    var body: some View {
        VStack (alignment: .leading) {
            HStack {
                Spacer()
                Text("Schedules")
                    .font(.title)
                    .padding()
                
                Image(systemName: "chevron.down")
                    .rotationEffect(.degrees(isExpanded ? 180 : 0))
                Spacer()
            }.background(Color(white:0.75)).onTapGesture {
                withAnimation {
                    isExpanded.toggle()
                }
            }
            
            if isExpanded {
                VStack (alignment: .leading, spacing: 20) {
                    VStack(alignment: .leading) {
                        Button("Add Schedule") {
                            // Show the pop-up for adding a new schedule
                            selectedIndex = nil
                            isPopupPresented.toggle()
                            print("add")
                        }
                        
                        ForEach(schedules.indices, id: \.self) { index in
                            VStack {
                                HStack {
                                    Text("Schedule #\(index)").fontWeight(.bold)
                                    Spacer()
                                    Button("Edit") {
                                        // Show the pop-up for editing the schedule
                                        selectedIndex = index
                                        print("index: \(index)")
                                        isPopupPresented.toggle()
                                        print("edit")
                                    }
                                }
                                Text("Days: \(selectedDaysString(schedule: schedules[index]))")
                                Text("Time: \(selectedTimeString(schedule: schedules[index]))")
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
            
            // Unified view for editing/creating schedules
            SchedulePopupView(
                isPresented: $isPopupPresented,
                selectedIndex: selectedIndex,
                //                    schedules: schedules,
                schedule: selectedIndex != nil ? schedules[selectedIndex!] : nil,
                onSave: { newSchedule in
                    if let index = selectedIndex {
                        schedules[index] = newSchedule
                    } else {
                        schedules.append(newSchedule)
                    }
                    bluetoothModel.sendSchedules(schedules)
                    isPopupPresented = false
                }
            )
            .opacity(isPopupPresented ? 1 : 0) // Optionally, fade out when not presented
            .animation(.easeInOut, value: 1)
        }
        .onAppear {
            // Initialize schedules when the view appears
            schedules = bluetoothModel.configPacketData_Schedule?.scheduleConfig ?? []
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
        return "\(schedule.startHour):\(schedule.startMinute) - \(schedule.stopHour):\(schedule.stopMinute)"
    }
}

struct SchedulePopupView: View {
    @Binding var isPresented: Bool
    @State private var editedSchedule: ScheduleConfig
    let originalSchedule: ScheduleConfig
    let selectedIndex: Int?
    var onSave: (ScheduleConfig) -> Void
    
    init(isPresented: Binding<Bool>, selectedIndex: Int?, schedule: ScheduleConfig? = nil, onSave: @escaping (ScheduleConfig) -> Void) {
        self._isPresented = isPresented
        self.originalSchedule = schedule ?? ScheduleConfig()
        self._editedSchedule = State(initialValue: selectedIndex != nil ? (schedule ?? ScheduleConfig()) : ScheduleConfig())
        self.selectedIndex = selectedIndex
        self.onSave = onSave
    }
    
    var body: some View {
        VStack {
            Text(selectedIndex != nil ? "Edit Schedule" : "Add Schedule")
                .font(.title)
                .padding()
            
            // Checkboxes for days
            ForEach(DaysOfWeek.allCases, id: \.self) { day in
                Button(action: {
                    toggleDay(day)
                }) {
                    HStack {
                        Image(systemName: editedSchedule.isDaySelected(day) ? "checkmark.square" : "square")
                            .resizable()
                            .frame(width: 20, height: 20)
                        Text(day.rawValue.prefix(3))
                    }
                    .padding(.vertical, 5)
                }
            }
            
            // Time picker
            HStack {
                Text("Start Time:")
                TimePicker(selectedHour: $editedSchedule.startHour, selectedMinute: $editedSchedule.startMinute)
            }
            
            HStack {
                Text("End Time:")
                TimePicker(selectedHour: $editedSchedule.stopHour, selectedMinute: $editedSchedule.stopMinute)
            }
            
            Button("Save") {
                onSave(editedSchedule)
                isPresented.toggle()
                isPresented.toggle()
            }
            .padding()
            
            Button("Cancel") {
                isPresented.toggle()
            }
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
                    Text("\(hour)")
                        .tag(UInt32(hour))
                }
            }
            .frame(width: 50)
            .clipped()
            
            Text(":")
            
            Picker("", selection: $selectedMinute) {
                ForEach(0..<60, id: \.self) { minute in
                    Text("\(minute)")
                        .tag(UInt32(minute))
                }
            }
            .frame(width: 50)
            .clipped()
        }
        .pickerStyle(WheelPickerStyle())
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
