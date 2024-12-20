// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterLimelight;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.util.config.JoystickFilterConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Maintains driver control of robot translation, but automatically turns to face the speaker
 * with no PID for higher speed.
 */
public class AlignWithSpeaker1Command extends Command {

  private final Drivetrain m_drivetrain;

  private GenericHID m_driverController;
  private PolarJoystickFilter translationFilter;

  private final boolean m_isOpenLoop;
  private final boolean m_isFieldRelative;

  private double error;

  /** Creates a new AlignWithSpeaker1Command for teleop. */
  public AlignWithSpeaker1Command(Drivetrain drivetrain, GenericHID driverController, boolean isOpenLoop,
      boolean isFieldRelative) {

    this(drivetrain, isOpenLoop, isFieldRelative);

    m_driverController = driverController;

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(
        0.07,
        0.85,
        Constants.OperatorConstants.driveExponent,
        Constants.OperatorConstants.driveExponentPercent));

  }

  /** Creates a new AlignWithSpeaker1Command for autonomous. */
  public AlignWithSpeaker1Command(Drivetrain drivetrain, boolean isOpenLoop, boolean isFieldRelative) {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_isOpenLoop = isOpenLoop;
    m_isFieldRelative = isFieldRelative;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("Running");
    m_drivetrain.runDutyCycle(0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationX = 0;
    double translationY = 0;

    if (m_driverController != null) {
    
      double[] translation = translationFilter.filter(m_driverController.getRawAxis(1), m_driverController.getRawAxis(0));

      translationX = translation[0] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;
      translationY = translation[1] * Constants.kDrivetrain.MAX_LINEAR_VELOCITY;

    }

    // find the angle to speaker;
    //Translation2d directionToSpeaker = ShooterLimelight.getTargetDetected()? ShooterLimelight.getSpeakerPosition() : m_drivetrain.getSpeakerPosition();
    Translation2d directionToSpeaker = m_drivetrain.getSpeakerPosition();
    Rotation2d targetAngle = directionToSpeaker.getAngle();
    double targetDegrees = targetAngle.getDegrees() % 360;
    if (targetDegrees < 0) {
      targetDegrees += 360;
    }

    // Run PID Controller
    //double currentAngle = ShooterLimelight.getTable().getTargetDetected()? ShooterLimelight.getTable().getLastBotPoseBlue().getRotation().getDegrees() : m_drivetrain.getPose().getRotation().getDegrees();
    double currentAngle = m_drivetrain.getPose().getRotation().getDegrees();
    error = (targetDegrees - currentAngle);
    double turnAmount = (error > 0 && error < 180) ? Constants.kShooter.SPEAKER_ALIGN_INITIAL_SPEED : -Constants.kShooter.SPEAKER_ALIGN_INITIAL_SPEED;

    m_drivetrain.drive(
        new Transform2d(new Translation2d(translationX, translationY), Rotation2d.fromDegrees(turnAmount)),
        m_isOpenLoop,
        m_isFieldRelative);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.drive(
      new Transform2d(), 
      m_isOpenLoop,
      m_isFieldRelative);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(error) <= 10;

  }

}