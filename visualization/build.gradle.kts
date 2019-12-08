plugins {
    id("java")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "1.0-SNAPSHOT"

repositories {
    jcenter()
    mavenLocal()
    maven (url = "https://jitpack.io")
}

dependencies {
    implementation("com.github.MittyRobotics:visualization:1.0-SNAPSHOT")
}

publishing {
    repositories {
        mavenLocal()
    }
}