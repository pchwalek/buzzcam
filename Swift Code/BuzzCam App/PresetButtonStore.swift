//
//  PresetButtonStore.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/3/24.
//

import Foundation
import SwiftUI

struct PresetButton: Identifiable, Codable {
    let id = UUID()
    var name: String
    var color: String
    var description: String
    var beep: Bool
}

@MainActor
class PresetButtonStore: ObservableObject {    
    @Published var presetButtons: [PresetButton] = [] {
            didSet {
                saveIfNeeded()
            }
        }

        private func saveIfNeeded() {
            Task {
                do {
                    try await save(presetButtons: presetButtons)
                } catch {
                    fatalError(error.localizedDescription)
                }
            }
        }
    
    private static func fileURL() throws -> URL {
            try FileManager.default.url(for: .documentDirectory,
                                        in: .userDomainMask,
                                        appropriateFor: nil,
                                        create: false)
            .appendingPathComponent("presetbuttons.data")
        }
    
    func load() async throws {
        let task = Task<[PresetButton], Error> {
            let fileURL = try Self.fileURL()
            guard let data = try? Data(contentsOf: fileURL) else {
                return []
            }
            let presetButtons = try JSONDecoder().decode([PresetButton].self, from: data)
            return presetButtons
        }
        let presetButtons = try await task.value
        self.presetButtons = presetButtons
    }
    
    func save(presetButtons: [PresetButton]) async throws {
            let task = Task {
                let data = try JSONEncoder().encode(presetButtons)
                let outfile = try Self.fileURL()
                try data.write(to: outfile)
            }
            _ = try await task.value
        }
}

