plugins {
    id("java")
    id("edu.wpi.first.GradleRIO")
    id("maven")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "1.0-SNAPSHOT"

repositories {
    jcenter()
    mavenLocal()
}

dependencies {
    implementation("com.github.MittyRobotics:motion-profiling:1.0-SNAPSHOT")
    compile(project(":datatypes"))
}

