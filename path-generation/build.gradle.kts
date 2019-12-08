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
    implementation("com.github.MittyRobotics:path-generation:1.0-SNAPSHOT")
    compile(project(":datatypes"))
}

publishing {
    repositories {
        mavenLocal()
    }
}