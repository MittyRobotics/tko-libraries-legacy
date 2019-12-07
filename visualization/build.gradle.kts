plugins {
    id("java")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "1.0-SNAPSHOT"

repositories {
    jcenter()
    mavenLocal()
}

dependencies {
    implementation("com.github.MittyRobotics:visualization:1.0-SNAPSHOT")
}

publishing {
    repositories {
        mavenLocal()
    }
}