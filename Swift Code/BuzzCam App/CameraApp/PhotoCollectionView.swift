//
//  PhotoCollectionView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/16/24.
//

/*
See the License.txt file for this sample’s licensing information.
*/

import SwiftUI
import os.log
import CoreLocation
import Photos

struct PhotoCollectionView: View {
    @ObservedObject var photoCollection : PhotoCollection
    
    @Environment(\.displayScale) private var displayScale
        
    private static let itemSpacing = 12.0
    private static let itemCornerRadius = 15.0
    private static let itemSize = CGSize(width: 90, height: 90)
    
    private var imageSize: CGSize {
        return CGSize(width: Self.itemSize.width * min(displayScale, 2), height: Self.itemSize.height * min(displayScale, 2))
    }
    
    private let columns = [
        GridItem(.adaptive(minimum: itemSize.width, maximum: itemSize.height), spacing: itemSpacing)
    ]
    
    var body: some View {
        ScrollView {
            LazyVGrid(columns: columns, spacing: Self.itemSpacing) {
                ForEach(photoCollection.photoAssets) { asset in
                    NavigationLink {
                        PhotoView(asset: asset, cache: photoCollection.cache)
                    } label: {
                        photoItemView(asset: asset)
                    }
                    .buttonStyle(.borderless)
                    .accessibilityLabel(asset.accessibilityLabel)
                }
            }
            .padding([.vertical], Self.itemSpacing)
        }
        .navigationTitle(photoCollection.albumName ?? "Gallery")
        .navigationBarTitleDisplayMode(.inline)
        .statusBar(hidden: false)
    }
    
    private func photoItemView(asset: PhotoAsset) -> some View {
        PhotoItemView(asset: asset, cache: photoCollection.cache, imageSize: imageSize)
            .frame(width: Self.itemSize.width, height: Self.itemSize.height)
            .clipped()
            .cornerRadius(Self.itemCornerRadius)
            .overlay(alignment: .bottomLeading) {
                if asset.isFavorite {
                    Image(systemName: "heart.fill")
                        .foregroundColor(.white)
                        .shadow(color: .black.opacity(0.3), radius: 5, x: 0, y: 1)
                        .font(.callout)
                        .offset(x: 4, y: -4)
                }
            }
            .onAppear {
                Task {
                    await photoCollection.cache.startCaching(for: [asset], targetSize: imageSize)
                }
            }
            .onDisappear {
                Task {
                    await photoCollection.cache.stopCaching(for: [asset], targetSize: imageSize)
                }
            }
    }
    
    
//    func filterAssetsBySource() -> [PhotoAsset] {
//        var filteredAssets = [PhotoAsset]()
//
//        for photoAsset in photoCollection.photoAssets {
//            let asset = photoAsset.phAsset
//            let options = PHImageRequestOptions()
//            options.isSynchronous = true
//
//            PHImageManager.default().requestImageData(for: asset!, options: options) { (imageData, _, _, _) in
//                guard let imageData = imageData else { return }
//                guard let imageSource = CGImageSourceCreateWithData(imageData as CFData, nil) else { return }
//                guard let metadata = CGImageSourceCopyPropertiesAtIndex(imageSource, 0, nil) as? [String: Any] else { return }
//
//                if let exifData = metadata[kCGImagePropertyExifDictionary as String] as? [String: Any] {
//                    let fileSource = exifData[kCGImagePropertyHeight as String] as? String ?? "oops"
//                    print("fileSource: ", fileSource)
//                    if fileSource.contains("Buzz") || fileSource.contains("BuzzCam App") || fileSource.contains("com.izzy.BuzzCam") {
//                        filteredAssets.append(photoAsset)
//                    }
//                }
//            }
//        }
//
//        return filteredAssets
//    }


    

    
//    // Filter assets by the image source from EXIF metadata
//    func filterAssetsBySource() -> [PhotoAsset] {
//        var filteredAssets = [PhotoAsset]()
//        
//        let options = PHContentEditingInputRequestOptions()
//        options.canHandleAdjustmentData = {(adjustmentData: PHAdjustmentData) -> Bool in
//            return adjustmentData.formatIdentifier == "edu.mit.BuzzCam"
//        }
//        
//        
//        for photoAsset in photoCollection.photoAssets {
//            let asset = photoAsset.phAsset
//            asset!.requestContentEditingInput(with: options) { (input, _) in
//                guard let input = input else { return }
//                guard let url = input.fullSizeImageURL else { return }
//                let imageSource = CGImageSourceCreateWithURL(url as CFURL, nil)
//                let metadata = CGImageSourceCopyPropertiesAtIndex(imageSource!, 0, nil) as? [CFString: Any]
//                if let exifData = metadata?[kCGImagePropertyExifDictionary] as? [CFString: Any] {
//                    let fileSource = exifData[kCGImagePropertyExifFileSource] as? CFString ?? "oops" as CFString
//                    let fileSourceString = fileSource.CFStringCreateString
//                    print("fileSource: ", fileSource)
//                    if (fileSource.contains("Buzz") || fileSource.contains("BuzzCam App") || fileSource.contains("com.izzy.BuzzCam")) {
//                        filteredAssets.append(photoAsset);
//                    }
//                }
//                
//            }
//            
//        }
        
//        for index in 0..<photoCollection.photoAssets.count {
//            let asset = photoCollection.photoAssets[index]
//            if let imageSource = getImageSourceFromEXIFMetadata(for: asset) {
//                if imageSource.contains(source) {
//                    filteredAssets.append(asset)
//                }
//            }
//        }
        
//        return filteredAssets
//    }
//    
//    // Function to extract image source from EXIF metadata
//    private func getImageSourceFromEXIFMetadata(for asset: PhotoAsset) -> String? {
//        guard let resources = PHAssetResource.assetResources(for: asset.phAsset!).first else {
//            return nil
//        }
//        
//        let imageSourceKey = "kCGImagePropertyExifFileSource" // EXIF key for image source
//        
//        if let data = try? Data(contentsOf: resources.assetResourceURL),
//           let imageSource = UIImage(data: data)?.value(forKey: imageSourceKey) as? String {
//            return imageSource
//        }
//        
//        return nil
//    }

}


//
//struct PhotoCollectionView: View {
//    @ObservedObject var photoCollection : PhotoCollection
//    
//    @Environment(\.displayScale) private var displayScale
//        
//    private static let itemSpacing = 12.0
//    private static let itemCornerRadius = 15.0
//    private static let itemSize = CGSize(width: 90, height: 90)
//    
//    private var imageSize: CGSize {
//        return CGSize(width: Self.itemSize.width * min(displayScale, 2), height: Self.itemSize.height * min(displayScale, 2))
//    }
//    
//    private let columns = [
//        GridItem(.adaptive(minimum: itemSize.width, maximum: itemSize.height), spacing: itemSpacing)
//    ]
//    
//    var body: some View {
//        ScrollView {
//            LazyVGrid(columns: columns, spacing: Self.itemSpacing) {
//                ForEach(filterAssetsBySource(source: "Buzz")) { asset in
//                    NavigationLink {
//                        PhotoView(asset: asset, cache: photoCollection.cache)
//                    } label: {
//                        photoItemView(asset: asset)
//                    }
//                    .buttonStyle(.borderless)
//                    .accessibilityLabel(asset.accessibilityLabel)
//                }
//            }
//            .padding([.vertical], Self.itemSpacing)
//        }
//        .navigationTitle(photoCollection.albumName ?? "Gallery")
//        .navigationBarTitleDisplayMode(.inline)
//        .statusBar(hidden: false)
//    }
//    
//    private func photoItemView(asset: PhotoAsset) -> some View {
//        PhotoItemView(asset: asset, cache: photoCollection.cache, imageSize: imageSize)
//            .frame(width: Self.itemSize.width, height: Self.itemSize.height)
//            .clipped()
//            .cornerRadius(Self.itemCornerRadius)
//            .overlay(alignment: .bottomLeading) {
//                if asset.isFavorite {
//                    Image(systemName: "heart.fill")
//                        .foregroundColor(.white)
//                        .shadow(color: .black.opacity(0.3), radius: 5, x: 0, y: 1)
//                        .font(.callout)
//                        .offset(x: 4, y: -4)
//                }
//            }
//            .onAppear {
//                Task {
//                    await photoCollection.cache.startCaching(for: [asset], targetSize: imageSize)
//                }
//            }
//            .onDisappear {
//                Task {
//                    await photoCollection.cache.stopCaching(for: [asset], targetSize: imageSize)
//                }
//            }
//    }
//    
//    
//    
//    // Filter assets by the image source from EXIF metadata
//    func filterAssetsBySource(source: String) -> [PhotoAsset] {
//        var filteredAssets = [PhotoAsset]()
//        
//        for index in 0..<photoCollection.photoAssets.count {
//            let asset = photoCollection.photoAssets[index]
//            if let imageSource = getImageSourceFromEXIFMetadata(for: asset) {
//                if imageSource.contains(source) {
//                    filteredAssets.append(asset)
//                }
//            }
//        }
//        
//        return filteredAssets
//    }
//    
//    // Function to extract image source from EXIF metadata
//    private func getImageSourceFromEXIFMetadata(for asset: PhotoAsset) -> String? {
//        guard let resources = PHAssetResource.assetResources(for: asset.phAsset!).first else {
//            return nil
//        }
//        
//        let imageSourceKey = "kCGImagePropertyExifFileSource" // EXIF key for image source
//        
//        if let data = try? Data(contentsOf: resources.value(forKey: "fileURL") as! URL),
//           let imageSource = UIImage(data: data)?.value(forKey: imageSourceKey) as? String {
//            return imageSource
//        }
//        
//        return nil
//    }
//
//}
