import SwiftUI

// View for testing popup, not actually used in app

struct TestView: View {
    @State private var isPopupPresented = false

    var body: some View {
        ZStack {
            // Your main content
            VStack {
                Text("Main Content")
                Button("Show Popup") {
                    isPopupPresented.toggle()
                }
            }
            .padding()

            // Pop-up window
            if isPopupPresented {
                PopupView(dismissPopup: $isPopupPresented)
            }
        }
    }
}

struct PopupView: View {
    @Binding var dismissPopup: Bool

    var body: some View {
        VStack {
            Text("Popup Window")
                .font(.title)
                .padding()

            // Add controls for editing ScheduleConfig properties
            // Use appropriate SwiftUI components for editing (e.g., sliders, text fields)

            Button("Save") {
                // Handle save action
                dismissPopup.toggle()
            }
            .padding()

            Button("Cancel") {
                // Handle cancel action
                dismissPopup.toggle()
            }
            .padding()
        }
        .frame(width: 300, height: 200) // Adjust the size as needed
        .background(Color.white)
        .cornerRadius(10)
        .overlay(
            RoundedRectangle(cornerRadius: 10)
                .stroke(Color.gray, lineWidth: 2)
        )
        .shadow(radius: 5)
        .padding()
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
