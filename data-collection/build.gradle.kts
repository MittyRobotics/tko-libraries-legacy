plugins {
    id("java")
    id("edu.wpi.first.GradleRIO")
    id("maven")
    `maven-publish`
}

group = "com.github.MittyRobotics"
version = "0.1.0"

repositories {
    jcenter()
    mavenLocal()
}

dependencies {
    compile(project(":datatypes"))
    compile(project(":motion-profiling"))
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            from(components["java"])
        }
    }
}