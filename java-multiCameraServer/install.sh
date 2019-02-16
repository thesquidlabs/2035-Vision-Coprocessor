#!/bin/sh
./gradlew build
cp build/libs/java-multiCameraServer-all.jar runCamera /home/pi
