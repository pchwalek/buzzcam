//
//  VideoPlayerView.swift
//  BuzzCam App
//
//  Created by Responsive Environments on 4/5/24.
//

import SwiftUI
import AVKit

struct PlayerView: UIViewRepresentable {
    func updateUIView(_ uiView: UIView, context: UIViewRepresentableContext<PlayerView>) {
    }
    
    func makeUIView(context: Context) -> UIView {
        return PlayerUIView(frame: .zero)
    }
}

class PlayerUIView: UIView {
    private let playerLayer = AVPlayerLayer()
    
    override init(frame: CGRect) {
        super.init(frame: frame)
        
        if let fileURL = Bundle.main.url(forResource: "patagonia_vid_1", withExtension: "mp4") {
            let player = AVPlayer(url: fileURL)
            player.actionAtItemEnd = .none
            player.play()
            
            playerLayer.player = player
            playerLayer.videoGravity = .resizeAspectFill
            
            NotificationCenter.default.addObserver(self,
                                                   selector: #selector(playerItemDidReachEnd(notification:)),
                                                   name: .AVPlayerItemDidPlayToEndTime,
                                                   object: player.currentItem)

            layer.addSublayer(playerLayer)
        }
        else {
            print("video not found")
        }
        
    }
    
    @objc func playerItemDidReachEnd(notification: Notification) {
        if let playerItem = notification.object as? AVPlayerItem {
            playerItem.seek(to: .zero, completionHandler: nil)
        }
    }
    
    required init?(coder: NSCoder) {
        fatalError("init(coder:) has not been implemented")
    }
    
    override func layoutSubviews() {
        super.layoutSubviews()
        playerLayer.frame = bounds
    }
}
