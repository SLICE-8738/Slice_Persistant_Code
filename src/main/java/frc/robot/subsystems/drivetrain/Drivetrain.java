/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import frc.robot.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import com.pathplanner.lib.util.PathPlannerLogging;

import com.studica.frc.AHRS;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule[] swerveMods;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
  private final SwerveDrivePoseEstimator m_odometry;
  private final AHRS m_gyro;
  public final Field2d m_field2d;

  private Rotation2d fieldOrientedOffset;

  private Rotation2d simHeading = new Rotation2d();

  public double maxLinearVelocity = 4.5;
  public double maxAngularVelocity = 7;

  private final SysIdRoutine sysIDDriveRoutine;
  public final SendableChooser<Command> sysIDChooser;

  /** Creates a new Drivetrain. */
  public Drivetrain(SwerveModuleIO mod0IO, SwerveModuleIO mod1IO, SwerveModuleIO mod2IO, SwerveModuleIO mod3IO) {

    swerveMods = new SwerveModule[] {
      new SwerveModule(mod0IO, 0),
      new SwerveModule(mod1IO, 1),
      new SwerveModule(mod2IO, 2),
      new SwerveModule(mod3IO, 3)
    };

    m_gyro = new AHRS(Constants.kDrivetrain.NAVX_PORT);

    Timer.delay(1.0);
    resetModulesToAbsolute();
    resetHeading();

    m_field2d = new Field2d();

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d);

    m_odometry = new SwerveDrivePoseEstimator(
      Constants.kDrivetrain.kSwerveKinematics, 
      getHeading(), 
      getModulePositions(), 
      LimelightHelpers.getBotPose2d_wpiBlue("limelight"),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.1, 0.1, 9999999));

    fieldOrientedOffset = new Rotation2d();

    PathPlannerLogging.setLogActivePathCallback(
      (path) -> {
        Logger.recordOutput("Odometry/Trajectory", path.toArray(new Pose2d[path.size()]));
        m_field2d.getObject("Trajectory").setPoses(path);
      }
    );
    PathPlannerLogging.setLogTargetPoseCallback(
      (pose) -> {
        Logger.recordOutput("Odometry/Trajectory Setpoint", pose);
      }
    );

    sysIDDriveRoutine = new SysIdRoutine(
      new Config(), 
      new Mechanism(
        (voltage) -> {
          for (SwerveModule mod : swerveMods) {
            mod.runCharacterization(voltage.in(Units.Volts));
          }
        },
        null,
        this));

    sysIDChooser = new SendableChooser<Command>();

    sysIDChooser.setDefaultOption("Quasistatic Forward", sysIDDriveRoutine.quasistatic(Direction.kForward)
      .beforeStarting(SignalLogger::start).andThen(SignalLogger::stop));
    sysIDChooser.addOption("Quasistatic Reverse", sysIDDriveRoutine.quasistatic(Direction.kReverse)
      .beforeStarting(SignalLogger::start).andThen(SignalLogger::stop));
    sysIDChooser.addOption("Dynamic Forward", sysIDDriveRoutine.quasistatic(Direction.kForward)
      .beforeStarting(SignalLogger::start).andThen(SignalLogger::stop));
    sysIDChooser.addOption("Dynamic Reverse", sysIDDriveRoutine.quasistatic(Direction.kReverse)
      .beforeStarting(SignalLogger::start).andThen(SignalLogger::stop));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    for (SwerveModule mod : swerveMods) {

      mod.updateInputs();

    }

    updateOdometry();
    m_field2d.setRobotPose(getPose());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drives the robot at either given field-relative X, Y, and rotational
   * velocities or given
   * robot-relative forward, sideways, and rotational velocities.
   * 
   * <p>
   * If using robot-relative velocities, the X component of the Translation2d
   * object should be the forward velocity
   * and the Y component should be the sideways velocity.
   * 
   * @param transform       A Transform2d object representing either the desired
   *                        field-relative velocities in meters/second for the
   *                        robot to move at along the X and Y axes of the
   *                        field(forwards/backwards from driver POV), or the
   *                        desired robot-relative forward
   *                        and sideways velocities in meters/second for the robot
   *                        to move at, as well as the desired velocity in
   *                        radians/second for the
   *                        robot to rotate at.
   * 
   * @param isOpenLoop      Whether the accordingly generated states for the given
   *                        velocities should be set using open loop control for
   *                        the drive motors
   *                        of the swerve modules.
   * @param isFieldRelative Whether the given velocities are relative to the field
   *                        or not.
   */
  public void drive(Transform2d transform, boolean isOpenLoop, boolean isFieldRelative) {

    Rotation2d rotationWithOffset = getHeading().minus(fieldOrientedOffset);
    if (rotationWithOffset.getDegrees() > 360) {
      rotationWithOffset.minus(Rotation2d.fromDegrees(360));
    }
    if (rotationWithOffset.getDegrees() < 0) {
      rotationWithOffset.plus(Rotation2d.fromDegrees(360));
    }

    ChassisSpeeds speeds = new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians());

    speeds.toRobotRelativeSpeeds(rotationWithOffset);
    speeds.discretize(0.02);

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    runSetpoints(states, isOpenLoop);

  }

  /**
   * Updates the drivetrain odometry object to the robot's current position on the
   * field.
   * 
   * @return The new updated pose of the robot.
   */
  public Pose2d updateOdometry() {

    m_odometry.update(getHeading(), getModulePositions());

    if (!DriverStation.isAutonomousEnabled()) {

      LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");

      if (estimate.tagCount >= 2) {

        Translation3d aprilTagPosition = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-shooter").getTranslation();

        if (Math.hypot(aprilTagPosition.getX(), aprilTagPosition.getZ()) <= 4.5) {
        
          m_odometry.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
        
        }

      }

    }

    return m_odometry.getEstimatedPosition();

  }

  @AutoLogOutput(key = "Odometry/Field Position")
  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {

    return m_odometry.getEstimatedPosition();

  }

  public boolean atAllianceWing() {
    Alliance alliance = DriverStation.getAlliance().get();
    double x = m_odometry.getEstimatedPosition().getX();

    if (alliance == Alliance.Blue) {
      return x < 5.87248;
    } else {
      return x > 16.54 - 5.87248;
    }
  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules.
   * 
   * @return The current positions of all drivetrain swerve modules.
   */
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(SwerveModule mod : swerveMods) {

      positions[mod.moduleNumber] = mod.getPosition();

    }

    return positions;

  }

  @AutoLogOutput(key = "Swerve/Actual Module States")
  /**
   * Obtains and returns the current states of all drivetrain swerve modules.
   * 
   * @return The current states of all drivetrain swerve modules.
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      states[mod.moduleNumber] = mod.getState();

    }

    return states;

  }

  @AutoLogOutput(key = "Swerve/Target Module States")
  /**
   * Obtains and returns the target states that the drivetrain swerve modules have
   * been set to.
   * 
   * @return The target states that the drivetrain swerve modules have been set
   *         to.
   */
  public SwerveModuleState[] getTargetStates() {

    SwerveModuleState[] targetStates = new SwerveModuleState[4];

    for(SwerveModule mod : swerveMods) {

      targetStates[mod.moduleNumber] = mod.getTargetState();

    }

    return targetStates;

  }

  /**
   * Obtains and returns the current absolute angle readings
   * in degrees from the absolute encoders of all swerve modules 
   * without offsets.
   * 
   * @return The current absolute angle readings in degrees from the CANCoders
   *         of all swerve modules without offsets.
   */
  public double[] getAbsoluteAngles() {

    double[] angles = new double[4];

    for(SwerveModule mod : swerveMods) {

      angles[mod.moduleNumber] = mod.getAbsoluteAngle().getDegrees();

    }

    return angles;

  }

  /**
   * Sets the positions of the integrated angle motor
   * encoders of all swerve modules to the absolute position
   * readings of the CANCoders with their offsets being taken
   * into account.
   */
  public void resetModulesToAbsolute() {

    for(SwerveModule mod : swerveMods) {

      mod.resetToAbsolute();

    }

  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    m_odometry.resetPosition(getHeading(), getModulePositions(), position);

  }

  /** 
   * Resets the position of the odometry object using a specified rotation
   * while keeping the translation the same.
   * 
   * @param rotation The desired rotation to reset the odometry of the robot to.
   */
  public void resetRotation(Rotation2d rotation) {

    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), rotation));

  }

  /**
   * Resets the rotation of the odometry object to the rotation received from
   * the limelight.
   */
  public void resetToAprilTagRotation() {

    resetRotation(LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation());

  }

  public void resetFieldOrientedHeading() {

    fieldOrientedOffset = getHeading().minus(Rotation2d.fromDegrees(180));
    resetRotation(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 180 : 0));

  }

  public void reverseFieldOrientedHeading() {

    fieldOrientedOffset = getHeading();
    resetRotation(Rotation2d.fromDegrees(DriverStation.getAlliance().get() == Alliance.Blue? 0 : 180));

  }

  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the
   * gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getHeading() {

    if (RobotBase.isReal()) {
      return Rotation2d.fromDegrees(Constants.kDrivetrain.INVERT_GYRO? 
        MathUtil.inputModulus(-(m_gyro.getYaw() - 180), 0 , 360) 
          : MathUtil.inputModulus(m_gyro.getYaw() - 180, 0, 360));
    }
    else {
      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (SwerveModule mod : swerveMods) {
        
        moduleDeltas[mod.moduleNumber] = 
          new SwerveModulePosition(
            modulePositions[mod.moduleNumber].distanceMeters -
              lastModulePositions[mod.moduleNumber].distanceMeters,
            modulePositions[mod.moduleNumber].angle
          );
        lastModulePositions[mod.moduleNumber] = modulePositions[mod.moduleNumber];

      }

      simHeading = simHeading.plus(new Rotation2d(Constants.kDrivetrain.kSwerveKinematics.toTwist2d(moduleDeltas).dtheta));
      return simHeading;
    }

  }

  /**
   * @return The current rotational velocity of the robot as a Rotation2d
   */
  public Rotation2d getRotationalVelocity() {

    return Rotation2d.fromDegrees(Constants.kDrivetrain.INVERT_GYRO? -m_gyro.getRate() : m_gyro.getRate());

  }

  public boolean facingDoubleSub() {
    double degrees = getPose().getRotation().getDegrees();
    return (degrees > 0 && degrees < 45) || (degrees > 315 && degrees < 360) || (degrees > 135 && degrees < 225);
  }

  /**
   * Obtains and returns the current pitch of the robot from -180 to 180 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees.
   */
  public double getPitch() {

    return m_gyro.getPitch();

  }

  /**
   * Obtains and returns the current roll of the robot from -180 to 180 degrees,
   * with an offset of 1.7 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an
   *         offset of 1.7 degrees.
   */
  public double getRoll() {

    return m_gyro.getRoll();

  }

  /**
   * Resets the gyro yaw axis to a heading of 0.
   */
  public void resetHeading() {

    m_gyro.reset();

  }

  /**
   * Calculates and returns the current chassis speeds of the drivetrain using
   * the average forward and sideways velocities of the individual swerve modules
   * and the rotational velocity measured by the gyro.
   * 
   * @return The current chassis speeds of the drivetrain.
   */
  public ChassisSpeeds getChassisSpeeds() {

    return Constants.kDrivetrain.kSwerveKinematics.toChassisSpeeds(getModuleStates());

  }

  /**
   * Runs the drivetrain at the given robot-relative chassis 
   * speeds after they are converted to swerve module states.
   * 
   * @param speeds The desired chassis speeds to run the drivetrain at.
   * 
   */
  public void runChassisSpeeds(ChassisSpeeds speeds) {

    speeds.discretize(0.02);
    runSetpoints(Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(speeds), false);

  }

  /**
   * Runs the drivetrain swerve modules at the given setpoint states.
   * 
   * @param states The desired states for all drivetrain swerve modules to run at.
   * @param isOpenLoop Whether the accordingly generated states for the given
   *                   velocities should be set using open loop control for
   *                   the drive motors of the swerve modules.
   */
  public void runSetpoints(SwerveModuleState[] states, boolean isOpenLoop) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

    for(SwerveModule mod : swerveMods) {

      mod.runSetpoint(states[mod.moduleNumber], isOpenLoop);

    }

  }

  /**
   * Sets the drive and angle motors of all swerve modules to given drive and
   * angle motor
   * percent outputs.
   * 
   * @param drivePercentOutput The percent output between -1 and 1 to set all
   *                           drive motors to.
   * @param anglePercentOutput The percent output between -1 and 1 to set all
   *                           angle motors to.
   */
  public void runDutyCycle(double drivePercentOutput, double anglePercentOutput) {

    for(SwerveModule mod : swerveMods) {

      mod.runDutyCycle(drivePercentOutput, anglePercentOutput);

    }

  }

  public double[] driveOutputCurents(){

    double[] currents = new double[4];

    for (SwerveModule mod : swerveMods) {

      currents[mod.moduleNumber] = mod.getDriveOutputCurrent();

    }

    return currents;
  }

  public Command getSysIDDriveRoutine() {

    return sysIDChooser.getSelected();

  }

} 