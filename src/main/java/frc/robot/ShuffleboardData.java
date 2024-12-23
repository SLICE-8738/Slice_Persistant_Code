// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.drivetrain.Drivetrain;

/** Contains and runs all code needed to display all necessary information on Shuffleboard.*/
public class ShuffleboardData {

    private final ShuffleboardTab /*driverTab,*/ debugTab, swerveTab, autoTab;

    public ShuffleboardData(Drivetrain drivetrain, AutoSelector autoSelector) {

        //driverTab = Shuffleboard.getTab("Driver Tab");
        debugTab = Shuffleboard.getTab("Debug Tab");
        swerveTab = Shuffleboard.getTab("Swerve Tab");
        autoTab = Shuffleboard.getTab("Autonomous");

        //Displays the current velocity in meters per second of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Left Front Velocity", () -> drivetrain.getModuleStates()[0].speedMetersPerSecond).
        withPosition(0, 0).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Right Front Velocity", () -> drivetrain.getModuleStates()[1].speedMetersPerSecond).
        withPosition(7, 0).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Right Back Velocity", () -> drivetrain.getModuleStates()[2].speedMetersPerSecond).
        withPosition(7, 3).
        withSize(2, 1);
        //Displays the current velocity in meters per second of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Left Back Velocity", () -> drivetrain.getModuleStates()[3].speedMetersPerSecond).
        withPosition(0, 3).
        withSize(2, 1);
        
        //Displays the current CANCoder angle in degrees with no offset of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Left Front CANCoder Angle", () -> drivetrain.getAbsoluteAngles()[0]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 1).
        withSize(2, 1);
        //Displays the current CANCoder angle in degrees with no offset of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Right Front CANCoder Angle", () -> drivetrain.getAbsoluteAngles()[1]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0,"Max", 360)).
        withPosition(7, 1).
        withSize(2, 1);
        //Displays the current CANCoder angle in degrees with no offset of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Right Back CANCoder Angle", () -> drivetrain.getAbsoluteAngles()[2]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(7, 2).
        withSize(2, 1);
        //Displays the current CANCoder angle in degrees with no offset of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Left Back CANCoder Angle", () -> drivetrain.getAbsoluteAngles()[3]).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 2).
        withSize(2, 1);
        
        //Displays the current integrated encoder angle in degrees of the left front swerve module on Shuffleboard
        swerveTab.addDouble("Left Front Integrated Angle", () -> drivetrain.getModuleStates()[0].angle.getDegrees()).
        withPosition(2, 0).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the right front swerve module on Shuffleboard
        swerveTab.addDouble("Right Front Integrated Angle", () -> drivetrain.getModuleStates()[1].angle.getDegrees()).
        withPosition(5, 0).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the right back swerve module on Shuffleboard
        swerveTab.addDouble("Right Back Integrated Angle", () -> drivetrain.getModuleStates()[2].angle.getDegrees()).
        withPosition(5, 3).
        withSize(2, 1);
        //Displays the current integrated encoder angle in degrees of the left back swerve module on Shuffleboard
        swerveTab.addDouble("Left Back Integrated Angle", () -> drivetrain.getModuleStates()[3].angle.getDegrees()).
        withPosition(2, 3).
        withSize(2, 1);
        
        //Displays the current heading of the robot in degrees on Shuffleboard
        debugTab.addDouble("Drivetrain Heading", () -> drivetrain.getHeading().getDegrees()).
        withWidget(BuiltInWidgets.kDial).
        withProperties(Map.of("Min", 0, "Max", 360)).
        withPosition(0, 0).
        withSize(2, 1);
                
        //Displays the current position of the robot on the field on Shuffleboard
        debugTab.add(drivetrain.m_field2d).
        withPosition(4, 2).
        withSize(3, 2);

        // //Adds a tuner for the drive motor PID gains to Shuffleboard
        // ShuffleboardTuner.create(
        //     (values) -> {
          
        //       drivetrain.setDrivePID(values[0], values[1], values[2]);
          
        //     },
        //     new String[] {"kP", "kI", "kD"},
        //     "Drive Motor PID");

        // //Adds a tuner for the angle motor PID gains to Shuffleboard
        // ShuffleboardTuner.create(
        //     (values) -> {

        //         drivetrain.setAnglePIDF(values[0], values[1], values[2], values[3]);

        //     },
        //     new String[] {"kP", "kI", "kD", "kFF"},
        //     "Angle Motor PIDF");

        // //Adds a tuner for the maximum velocities to Shuffleboard
        // ShuffleboardTuner.create(
        //     (values) -> {

        //         drivetrain.maxLinearVelocity = values[0];
        //         drivetrain.maxAngularVelocity = values[1];

        //     },
        //     new String[] {"Max Linear", "Max Angular"},
        //     "Drivetrain Max Velocities");

        debugTab.add("SysID Routine", drivetrain.sysIDChooser);

        //Adds the sendable chooser for the desired autonomous mode onto Shuffleboard
        autoTab.add("Auto Mode", autoSelector.modeChooser).withPosition(2, 0).withSize(2, 1);
        //Adds the sendable chooser for the robot starting position onto Shuffleboard
        autoTab.add("Starting Position", autoSelector.startingPositionChooser).withPosition(5, 0).withSize(2, 1);

        //Displays the autonomous mode selected on the sendable chooser on Shuffleboard
        autoTab.addString("Selected Auto Mode", autoSelector::getMode).
        withPosition(2, 1).
        withSize(2, 1);
        //Displays the robot starting position selected on the sendable chooser on Shuffleboard
        autoTab.addString("Selected Starting Position", autoSelector::getStartingPosition).
        withPosition(5, 1).
        withSize(2, 1);

        //Displays the X offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
        autoTab.addDouble("Initial Auto Pose X Offset", () -> autoSelector.initialAutoPoseXOffset).
        withPosition(1, 2).
        withSize(2, 1);
        //Displays the Y offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
        autoTab.addDouble("Initial Auto Pose Y Offset", () -> autoSelector.initialAutoPoseYOffset).
        withPosition(6, 2).
        withSize(2, 1);
        //Displays the rotational offset of the robot from the inital pose of the selected autonomous routine on Shuffleboard
        autoTab.addDouble("Initial Auto Pose Rotation Offset", () -> autoSelector.initialAutoPoseRotationOffset).
        withPosition(3, 2).
        withSize(3, 1);

    }

}