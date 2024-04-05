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
    var isSelectedForDeletion = false // Step 1: Property to track selection for deletion
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

    func load() async {
        do {
            let fileURL = try Self.fileURL()
            guard let data = try? Data(contentsOf: fileURL) else {
                // File doesn't exist or couldn't be read
                print("File not found or couldn't be read.")
                return
            }
            let presetButtons = try JSONDecoder().decode([PresetButton].self, from: data)
            self.presetButtons = presetButtons
        } catch {
            print("Error loading data:", error)
        }
    }

    func save(presetButtons: [PresetButton]) async throws {
        let task = Task {
            let data = try JSONEncoder().encode(presetButtons)
            let outfile = try Self.fileURL()
            try data.write(to: outfile)
        }
        _ = try await task.value
    }
    
    func deletePresetButtons(withIDs ids: Set<UUID>) {
        presetButtons.removeAll { ids.contains($0.id) }
    }
}
