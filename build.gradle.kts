/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import edu.wpi.first.toolchain.NativePlatforms

plugins {
    id("java")
    id("edu.wpi.first.GradleRIO") version "2020.3.2"
    id("maven")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "0.1.0"

allprojects {
    apply(plugin = "java")
    apply(plugin = "edu.wpi.first.GradleRIO")
    apply(plugin = "maven")

    java {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    repositories {
        mavenLocal()
        jcenter()
        maven(url = "https://jitpack.io")
    }

    dependencies {
        // WPILib
        wpi.deps.wpilib().forEach { compile(it) }
        wpi.deps.wpilibJni(NativePlatforms.roborio).forEach { nativeZip(it) }
        wpi.deps.wpilibJni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

        wpi.deps.vendor.java().forEach { compile(it) }
        wpi.deps.vendor.jni(NativePlatforms.roborio).forEach { nativeZip(it) }
        wpi.deps.vendor.jni(NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

        compile(group = "edu.wpi.first.wpilibNewCommands", name = "wpilibNewCommands-java", version = "2020.3.2")
        compile(group = "com.revrobotics.frc", name = "SparkMax-java", version = "1.5.2")
        // https://mvnrepository.com/artifact/org.jfree/jfreechart
        compile(group = "org.jfree", name = "jfreechart", version = "1.5.0")
        compile(group = "org.apache.commons", name = "commons-collections4", version = "4.1")
    }
}

publishing {
    repositories {
        mavenLocal()
    }
}