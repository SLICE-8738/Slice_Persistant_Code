// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.slicelibs.config.CTREConfigs;
import frc.slicelibs.config.REVConfigs;
import frc.slicelibs.config.SwerveModuleConstants;

import com.studica.frc.AHRS.NavXComType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode ADVANTAGE_KIT_MODE = Mode.SIM;
  public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();
  public static final REVConfigs REV_CONFIGS = new REVConfigs();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final double driveExponent = 1.0;
    public static final double driveExponentPercent = 1;

    public static final double turnExponent = 1.0;
    public static final double turnExponentPercent = 1;

  }

  public final class kDrivetrain {

    /* Gyro */
    public static final NavXComType NAVX_PORT = NavXComType.kUSB1;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /* Swerve Physics */
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.729); // TODO: Find track width and wheel base length
    public static final double WHEEL_BASE = Units.inchesToMeters(18.299);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.95);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double MASS = 65.3173; // TODO: Find mass (kg)
    public static final double MOMENT_OF_INERTIA = 7; // TODO: Find MOI (kg*m^2)
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.7; // (Vex Griplocks)

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0), // Front left module 
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Front right module
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0), // Back right module
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)); // Back left module

    /* Motor Gearing */
    public static final double DRIVE_GEAR_RATIO = (5.14 / 1.0); // 5.14:1
    public static final double ANGLE_GEAR_RATIO = (25.0 / 1.0); // 25:1

    /* Swerve Voltage Compensation */
    public static final double MAX_VOLTAGE = 12.0;

    /* Swerve Current Limiting */
    public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT = true;
    public static final int DRIVE_SUPPLY_CURRENT_LIMIT = 40;
    public static final int DRIVE_SUPPLY_CURRENT_LOWER_LIMIT = 65;
    public static final double DRIVE_SUPPLY_CURRENT_LOWER_TIME = 0.1;

    public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = true;
    public static final double DRIVE_STATOR_CURRENT_LIMIT = 65;

    public static final int ANGLE_CURRENT_LIMIT = 20;

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Status Frame Rates/Periods */
    // TODO: Tune signal frequencies/status frame periods
    public static final int DRIVE_DEFAULT_FREQUENCY_HZ = 22;
    public static final int DRIVE_POSITION_FREQUENCY_HZ = 100;
    public static final int ANGLE_VELOCITY_PERIOD_MS = 1500;
    public static final int ANGLE_POSITION_PERIOD_MS = 300;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.12; // TODO: Tune drive motor PID gains
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    /* Angle Motor PID Values */
    public static final double ANGLE_KP = 0.01;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.001;
    public static final double ANGLE_KFF = 0.0;

    // TODO: Find drive and angle motor feedforward gains from characterization
    /* Drive Motor Feedforward Values */
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 2.4103;
    public static final double DRIVE_KA = 0.0;

    /* Angle Motor Feedforward Values */
    public static final double ANGLE_KS = 0.0;
    public static final double ANGLE_KV = 2.0244;
    public static final double ANGLE_KA = 0.0;

    /* Drive Motor Conversion Factors */
    public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
    public static final double ANGLE_POSITION_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
    public static final double ANGLE_VELOCITY_CONVERSION_FACTOR = ANGLE_POSITION_CONVERSION_FACTOR / 60.0;

    /* Swerve Profiling Values */
    // TODO: Find maximum velocities
    public static final double MAX_LINEAR_VELOCITY = 4.5; // meters per second
    public static final double MAX_ANGULAR_VELOCITY = 7; // radians per second

    /* PathPlanner Values */
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3.5, 2.5, Math.PI * 2, Math.PI * 2);
    public static final double TRANSLATION_KP = 4.5;
    public static final double ROTATION_KP = 1.0;

    /* Motor Idle Modes */
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;
    public static final NeutralModeValue DRIVE_IDLE_MODE = NeutralModeValue.Brake;

    /* Motor Inverts */
    public static final InvertedValue DRIVE_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final boolean ANGLE_INVERT = true;

    /* Absolute Angle Encoder Invert */
    public static final boolean ABSOLUTE_ENCODER_INVERT = false; //TODO: Determine whether to invert

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public final class Mod0 {
      public static final int DRIVE_MOTOR_ID = 4;
      public static final int ANGLE_MOTOR_ID = 8;
      public static final int ABSOLUTE_ENCODER_ID = 23;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(28.48);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
        ABSOLUTE_ENCODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public final class Mod1 {
      public static final int DRIVE_MOTOR_ID = 1;
      public static final int ANGLE_MOTOR_ID = 5;
      public static final int ABSOLUTE_ENCODER_ID_ID = 22;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(315.79);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
        ABSOLUTE_ENCODER_ID_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 2 */
    public final class Mod2 {
      public static final int DRIVE_MOTOR_ID = 2;
      public static final int ANGLE_MOTOR_ID = 30;
      public static final int ABSOLUTE_ENCODER_ID = 21;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(210.15);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
        ABSOLUTE_ENCODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 3 */
    public final class Mod3 {
      public static final int DRIVE_MOTOR_ID = 3;
      public static final int ANGLE_MOTOR_ID = 7;
      public static final int ABSOLUTE_ENCODER_ID = 20;
      public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(183.16);
      public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
        ABSOLUTE_ENCODER_ID, ANGLE_OFFSET);
    }

  }

  public final class kLEDs {
    public static final int LED_PWM_PORT = 1;
    public static final int LED_LENGTH = 300;
  }
  
}



