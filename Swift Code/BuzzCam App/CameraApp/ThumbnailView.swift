//
//  ThumbnailView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/16/24.
//

/*
See the License.txt file for this sample’s licensing information.
*/

import SwiftUI
import CoreLocation

struct ThumbnailView: View {
    var image: Image?
    
    var body: some View {
        ZStack {
            Color.white
            if let image = image {
                image
                    .resizable()
                    .scaledToFill()
            }
        }
        .frame(width: 41, height: 41)
        .cornerRadius(11)
    }
}

struct ThumbnailView_Previews: PreviewProvider {
    static let previewImage = Image(systemName: "photo.fill")
    static var previews: some View {
        ThumbnailView(image: previewImage)
    }
}
