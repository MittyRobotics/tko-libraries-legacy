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
    id("cpp")
    id("edu.wpi.first.GradleVsCode") version "0.8.0"
    id("edu.wpi.first.GradleJni") version "0.4.1"
    id("org.openjfx.javafxplugin") version "0.0.8"
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "0.1.0"

allprojects {
    apply(plugin = "java")
    apply(plugin = "edu.wpi.first.GradleRIO")
    apply(plugin = "maven")
    apply(plugin = "org.openjfx.javafxplugin")

    java {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    javafx {
        version = "11"
        modules("javafx.controls")
    }

    repositories {
        mavenLocal()
        mavenCentral()
        maven(url = "https://jitpack.io")
        maven(url = "http://www.revrobotics.com/content/sw/max/sdk/maven/")
        maven(url = "http://devsite.ctr-electronics.com/maven/release/")
    }

    dependencies {

        compile(group = "org.jfree", name = "jfreechart", version = "1.5.0")
        compile(group = "org.apache.commons", name = "commons-collections4", version = "4.1")
        compile(group = "org.ejml", name = "ejml-all", version = "0.38")
        compile(group = "org.jblas", name = "jblas", version = "1.2.4")
        compile(group = "org.la4j", name = "la4j", version = "0.6.0")
        compile(group = "net.java.dev.jna", name = "jna", version = "5.5.0")
        compile(group = "org.apache.commons", name = "commons-math3", version = "3.6.1")

    }
}

publishing {
    repositories {
        mavenLocal()
    }
}