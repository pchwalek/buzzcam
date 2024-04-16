//
//  CameraApp.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/16/24.
//

/*
See the License.txt file for this sample’s licensing information.
*/

import SwiftUI

//@main
struct CameraApp: View {

//    init() {
//        UINavigationBar.applyCustomAppearance()
//    }
    
//    var body: some Scene {
//        WindowGroup {
//            CameraView()
//        }
//    }
    
    var body: some View {
        VStack {
            CameraView()
        }.padding(.bottom, 100)
    }
}

//
//fileprivate extension UINavigationBar {
//    
//    static func applyCustomAppearance() {
//        let appearance = UINavigationBarAppearance()
//        appearance.backgroundEffect = UIBlurEffect(style: .systemUltraThinMaterial)
//        UINavigationBar.appearance().standardAppearance = appearance
//        UINavigationBar.appearance().compactAppearance = appearance
//        UINavigationBar.appearance().scrollEdgeAppearance = appearance
//    }
//}
