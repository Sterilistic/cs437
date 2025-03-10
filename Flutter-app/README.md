# flutter_blue_app

A Flutter application that uses the `flutter_bluetooth_serial` package.

## Table of Contents
- [Overview](#overview)
- [Setting Up Flutter](#setting-up-flutter)
  - [Installing Flutter on macOS](#installing-flutter-on-macos)
  - [Installing Flutter on Windows](#installing-flutter-on-windows)
- [Running the Application](#running-the-application)
- [Building the APK](#building-the-apk)
- [Setting Up Android SDK (macOS)](#setting-up-android-sdk-macos)
- [Setting Up Android SDK (Windows)](#setting-up-android-sdk-windows)
- [Getting Started](#getting-started)
- [Additional Resources](#additional-resources)

## Overview

This project is a starting point for a Flutter application that integrates Bluetooth functionality using the `flutter_bluetooth_serial` package.

## Setting Up Flutter

### Installing Flutter on macOS

1. **Download Flutter**
   ```bash
   curl -O https://storage.googleapis.com/flutter_infra_release/releases/stable/macos/flutter_macos_3.10.6-stable.zip
   ```
   Extract the downloaded zip and move it to the desired location:
   ```bash
   unzip flutter_macos_3.10.6-stable.zip
   mv flutter ~/development/flutter
   ```

2. **Update Environment Variables**
   Add Flutter to the PATH:
   ```bash
   export PATH="$PATH:$HOME/development/flutter/bin"
   ```
   To make this change permanent, add the line above to `~/.zshrc` or `~/.bashrc`, depending on your shell.

3. **Install Dependencies**
   ```bash
   brew install cocoapods
   flutter doctor
   ```
   Follow any additional installation steps suggested by `flutter doctor`.

### Installing Flutter on Windows

1. **Download Flutter**
   - Download the Flutter SDK from [Flutter Official Site](https://flutter.dev/docs/get-started/install/windows)
   - Extract it to `C:\src\flutter`

2. **Update Environment Variables**
   - Open **Edit environment variables** from the Start menu.
   - Add `C:\src\flutter\bin` to the **Path** variable.

3. **Install Dependencies**
   - Run:
     ```cmd
     flutter doctor
     ```
   - Install any missing dependencies reported by `flutter doctor`.

## Running the Application

1. **Add the Bluetooth Package Dependency**  
   In your project directory, run:
   ```bash
   flutter pub add flutter_bluetooth_serial:^0.4.0
   ```

2. **Get Dependencies**  
   Fetch the package:
   ```bash
   flutter pub get
   ```

3. **Create Necessary Files**  
   If needed, generate missing files:
   ```bash
   flutter create .
   ```

4. **Run the App**  
   Launch the application:
   ```bash
   flutter run
   ```

## Building the APK

1. **Set Up the Android SDK**  
   Follow the setup instructions below for your operating system.

2. **Clean and Build**  
   Once the SDK is configured, run:
   ```bash
   flutter clean
   flutter build apk --release
   ```

## Setting Up Android SDK (macOS)

1. **Install Android Command-Line Tools**
   ```bash
   brew install android-commandlinetools
   ```

2. **Create the Android SDK Directory**
   ```bash
   mkdir -p ~/Library/Android/sdk
   ```

3. **Install Essential SDK Packages**
   ```bash
   sdkmanager --sdk_root=$HOME/Library/Android/sdk "platform-tools" "platforms;android-34" "build-tools;34.0.0" "cmdline-tools;latest"
   ```

4. **Configure Environment Variables**
   Add these to `~/.zshrc` or `~/.bashrc`:
   ```bash
   export ANDROID_HOME=$HOME/Library/Android/sdk
   export PATH=$ANDROID_HOME/platform-tools:$PATH
   ```

5. **Verify Installation**
   ```bash
   flutter doctor
   ```

## Setting Up Android SDK (Windows)

1. **Install Android Command-Line Tools**
   - Download from [Android Developer](https://developer.android.com/studio#command-tools)
   - Extract it to `C:\Android\cmdline-tools\latest`

2. **Install SDK Packages**
   ```cmd
   cd C:\Android\cmdline-tools\latest\bin
   sdkmanager "platform-tools" "platforms;android-34" "build-tools;34.0.0" "cmdline-tools;latest"
   ```

3. **Configure Environment Variables**
   - Add `C:\Android\platform-tools` and `C:\Android\cmdline-tools\latest\bin` to **Path**.

4. **Verify Installation**
   ```cmd
   flutter doctor
   ```

## Getting Started

This project is a great starting point for your Flutter application. If you're new to Flutter, check out these resources:

- [Lab: Write your first Flutter app](https://docs.flutter.dev/get-started/codelab)
- [Cookbook: Useful Flutter samples](https://docs.flutter.dev/cookbook)

## Additional Resources

For more help with Flutter development, visit the [Flutter documentation](https://docs.flutter.dev/), which provides tutorials, samples, and a full API reference.

