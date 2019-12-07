
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.frc.FRCJavaArtifact
import edu.wpi.first.gradlerio.frc.FRCPlugin
import edu.wpi.first.gradlerio.frc.RoboRIO
import edu.wpi.first.gradlerio.wpi.dependencies.WPIVendorDepsExtension
import edu.wpi.first.toolchain.NativePlatforms
import jaci.gradle.deploy.artifact.ArtifactsExtension
import jaci.gradle.deploy.artifact.FileTreeArtifact
import jaci.gradle.deploy.target.TargetsExtension
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
