This repository will be used to rewrite the Swerve code in Java.
It consists of a Swerve directory (SwerveBotv2-Imported being the latest version) and a motorcontrol directory. Each of these is a separate WPILib project---the Swerve directories are used for controlling the Swerve Drive and subsystems, while motorcontrol is a general-purpose project for testing motors, motor controllers, etc.

SwerveBotv2-Imported is the ONLY working Swerve directory. It is the SwerveBotv2 directory, ported to work with the 2026 version of WPILib.

The configuration JSON files can be configured and downloaded from https://broncbotz.org/YAGSL-Example/.

**NOTE: Replace some values in config JSON files with actual values (Swerve module canbus id, grip coefficient, etc.). These files are located in ./src/main/deploy/swerve/
