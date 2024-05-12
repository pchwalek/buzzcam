//
//  CameraView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/15/24.
//

import SwiftUI

struct CameraViewTab: View {
    @EnvironmentObject var bluetoothModel: BluetoothModel

    var body: some View {
        // source: https://developer.apple.com/tutorials/sample-apps/capturingphotos-captureandsave
        CameraApp()
    }
}


