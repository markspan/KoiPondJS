plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "io.github.markspan.koipond"
    compileSdk = 35

    defaultConfig {
        applicationId = "io.github.markspan.koipond"
        minSdk = 26 // Hardware-accelerated wallpaper canvas (lockHardwareCanvas).
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }

    signingConfigs {
        // A fixed debug key, committed on purpose: every CI build is signed
        // with the same key, so a new APK installs over the previous one.
        // It is a debug key with the standard public password, not a secret.
        getByName("debug") {
            storeFile = file("../debug.keystore")
            storePassword = "android"
            keyAlias = "androiddebugkey"
            keyPassword = "android"
        }
    }

    buildTypes {
        getByName("release") {
            isMinifyEnabled = false
            signingConfig = signingConfigs.getByName("debug")
        }
    }

    testOptions {
        unitTests.isIncludeAndroidResources = true
        unitTests.all { test ->
            test.testLogging {
                events("failed")
                exceptionFormat = org.gradle.api.tasks.testing.logging.TestExceptionFormat.FULL
            }
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

kotlin {
    jvmToolchain(17)
}

dependencies {
    // Rendering tests run Android's real Skia graphics on the JVM.
    testImplementation("junit:junit:4.13.2")
    testImplementation("org.robolectric:robolectric:4.14.1")
}
