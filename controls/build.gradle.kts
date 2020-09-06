/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
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

plugins {
    id("java")
    id("maven")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "0.1.0"

repositories {
    jcenter()
    mavenLocal()
    maven(url = "https://jitpack.io")
}

dependencies {
    compile(project(":datatypes"))
    wpi.deps.wpilib().forEach { compile(it) }
    wpi.deps.wpilibJni(edu.wpi.first.toolchain.NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.wpilibJni(edu.wpi.first.toolchain.NativePlatforms.desktop).forEach { nativeDesktopZip(it) }

    wpi.deps.vendor.java().forEach { compile(it) }
    wpi.deps.vendor.jni(edu.wpi.first.toolchain.NativePlatforms.roborio).forEach { nativeZip(it) }
    wpi.deps.vendor.jni(edu.wpi.first.toolchain.NativePlatforms.desktop).forEach { nativeDesktopZip(it) }


}

publishing {
    publications {
        create<MavenPublication>("maven") {
            from(components["java"])
        }
    }
}
