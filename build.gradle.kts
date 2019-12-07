import edu.wpi.first.toolchain.NativePlatforms

plugins {
    id("java")
    id("edu.wpi.first.GradleRIO")  version "2019.4.1"
}

group = "com.github.MittyRobotics"
version = "1.0-SNAPSHOT"

repositories {
    jcenter()
    maven(url = "https://jitpack.io")
}


dependencies {
    // WPILib
    wpi.deps.wpilib().forEach { compile(it) }
    wpi.deps.vendor.java().forEach { compile(it) }
    wpi.deps.vendor.jni(NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.vendor.jni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

    // https://mvnrepository.com/artifact/org.jfree/jfreechart
    compile(group = "org.jfree", name = "jfreechart", version = "1.5.0")
}

