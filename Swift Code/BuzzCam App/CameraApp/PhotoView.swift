//
//  PhotoView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/16/24.
//

/*
See the License.txt file for this sample’s licensing information.
*/

import SwiftUI
import Photos
import Foundation
import CoreLocation

struct PhotoView: View {
    var asset: PhotoAsset
    var cache: CachedImageManager?
    @State private var image: Image?
    @State private var imageRequestID: PHImageRequestID?
    @Environment(\.dismiss) var dismiss
    private let imageSize = CGSize(width: 1024, height: 1024)

    // Create a DateFormatter instance to format the date
    let dateFormatter: DateFormatter = {
        let formatter = DateFormatter()
        formatter.dateFormat = "MMM dd, yyyy   HH:mm:ss" // Date format with time in military format (00:00:00)
        return formatter
    }()
    
    var body: some View {
        Group {
            if let image = image {
                image
                    .resizable()
                    .scaledToFit()
                    .accessibilityLabel(asset.accessibilityLabel)
            } else {
                ProgressView()
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .ignoresSafeArea()
        .background(Color.secondary)
        .navigationTitle("Photo")
        .navigationBarTitleDisplayMode(.inline)
        .overlay(alignment: .bottom) {
            buttonsView()
                .offset(x: 0, y: -50)
        }
        .task {
            guard image == nil, let cache = cache else { return }
            imageRequestID = await cache.requestImage(for: asset, targetSize: imageSize) { result in
                Task {
                    if let result = result {
                        self.image = result.image
                    }
                }
            }
        }
    }
    
    private func buttonsView() -> some View {
        VStack {
            HStack(spacing: 60) {
                
                Button {
                    Task {
                        await asset.setIsFavorite(!asset.isFavorite)
                    }
                } label: {
                    Label("Favorite", systemImage: asset.isFavorite ? "heart.fill" : "heart")
                        .font(.system(size: 24))
                }
                
                Button {
                    Task {
                        await asset.delete()
                        await MainActor.run {
                            dismiss()
                        }
                    }
                } label: {
                    Label("Delete", systemImage: "trash")
                        .font(.system(size: 24))
                }
            }
            .buttonStyle(.plain)
            .labelStyle(.iconOnly)
        
            // Format the foundation date into a string
            let dateString = dateFormatter.string(from: asset.creationDate)
            
            // Display the formatted date string as a SwiftUI view
            Text(dateString).padding()
            
//            let latitude = asset.location.coordinate.latitude;
//            let longitude = asset.location.coordinate.longitude;
//            Text("Location: \(latitude) \(longitude)");
            

            
        }.padding(EdgeInsets(top: 20, leading: 30, bottom: 20, trailing: 30))
            .foregroundColor(.white)
            .background(Color.secondary.colorInvert())
            .cornerRadius(15)
    }
}
