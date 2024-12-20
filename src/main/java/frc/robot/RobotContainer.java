// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Intake.StoreNote.StoreNoteSequence;
import frc.robot.commands.LEDs.SignalStoreNote;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.RealSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SimSwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.testing.routines.DrivetrainTest;
import frc.robot.testing.routines.FlywheelTest;
import frc.robot.testing.routines.IntakeTest;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final PS4Controller driverController = Button.controller1;
  private final GenericHID operatorController = Button.controller2;

  // ==========================
  // Subsystems
  // ==========================

  public final Drivetrain m_drivetrain;
  public final Shooter m_shooter;
  public final Intake m_intake;
  public final Indexer m_indexer;
  public final ShooterLimelight m_shooterLimelight;
  public final IntakeLimelight m_intakeLimelight;
  public final LEDs m_leds;

  public final AutoSelector m_autoSelector;
  public final ShuffleboardData m_shuffleboardData;

  // ==========================
  // Commands
  // ==========================

  /* Drivetrain */
  public final SwerveDriveCommand m_swerveDriveOpenLoop;
  public final SwerveDriveCommand m_swerveDriveClosedLoop;
  public final RunDutyCycleCommand m_setDrivePercentOutput;
  public final ResetFieldOrientedHeading m_resetFieldOrientedHeading;
  // public final Command m_pathfindToSource;
  public final Command m_pathfindToAmp;
  public final Command m_pathfindToStage;
  public final Command m_sysIDDriveRoutine;
  public final AlignWithSpeaker2Command m_alignWithSpeaker;
  // public final SequentialCommandGroup m_alignWithSpeaker;

  /* Shooter */
  public final ManualShooterCommand m_manualShooter;
  public final ResetAlternateAngleCommand m_resetAlternateAngle;
  public final StowShooterCommand m_stow;
  public final ToAmpPositionCommand m_toAmpAngle;
  public final AlignWithAmpCommand m_alignAmp;
  public final ToClimbPositionCommand m_ToClimbPositionCommand;
  public final ShootCommand m_shoot;
  public final ClimbLockCommand m_lockClimber;
  public final SubwooferShotCommand m_subwooferShotCommand;
  public final PassNoteCommand m_pass;

  /* Intake */
  public final RunIntakeCommand m_runIntakeIn;
  public final RunIntakeCommand m_runIntakeOut;
  public final StoreNoteSequence m_storeNote;
  public final ReverseWhileNoteStoredCommand m_reverseWhileNoteStored;

  /* Indexer */
  public final ManualIndexerCommand m_manualIndexer;
  public final AlignWithNoteCommand m_alignNote;

  public final RecordFFDataCommand m_ffData;

  /* LEDs */
  public final SignalStoreNote m_signalStoreNote;

  /* Tests */
  public final FlywheelTest m_flywheelTest;
  public final DrivetrainTest m_drivetrainTest;
  public final IntakeTest m_intakeTest;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // ==========================
    // Subsystems
    // ==========================

    switch (Constants.ADVANTAGE_KIT_MODE) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          m_drivetrain =
            new Drivetrain(
                new RealSwerveModuleIO(Constants.kDrivetrain.Mod0.CONSTANTS),
                new RealSwerveModuleIO(Constants.kDrivetrain.Mod0.CONSTANTS),
                new RealSwerveModuleIO(Constants.kDrivetrain.Mod0.CONSTANTS),
                new RealSwerveModuleIO(Constants.kDrivetrain.Mod0.CONSTANTS));
          break;
        case SIM:
          m_drivetrain =
            new Drivetrain(
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO(),
              new SimSwerveModuleIO());
          break;
        default:
          m_drivetrain =
            new Drivetrain(
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {});
          break;
    }

    m_shooter = new Shooter();
    m_intake = new Intake();
    m_indexer = new Indexer();
    m_shooterLimelight = new ShooterLimelight();
    m_intakeLimelight = new IntakeLimelight();
    m_leds = new LEDs();

    m_autoSelector = new AutoSelector(m_drivetrain, m_shooter, m_intake, m_indexer);
    m_shuffleboardData = new ShuffleboardData(m_drivetrain, m_shooter,
        m_intake/* , m_indexer */, m_autoSelector);

    // ==========================
    // Commands
    // ==========================

    /* Drivetrain */
    m_swerveDriveOpenLoop = new SwerveDriveCommand(m_drivetrain, driverController, true);
    m_swerveDriveClosedLoop = new SwerveDriveCommand(m_drivetrain, driverController,
        false);
    m_setDrivePercentOutput = new RunDutyCycleCommand(m_drivetrain, 0.10, 0);
    m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
    // Command m_pathfindToSource = AutoBuilder
    //     .pathfindToPose(new Pose2d(1.32, 1.32, Rotation2d.fromDegrees(-120)), Constants.kDrivetrain.PATH_CONSTRAINTS);
    m_pathfindToAmp = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Align With Amp"),
      Constants.kDrivetrain.PATH_CONSTRAINTS);
    m_pathfindToStage = AutoBuilder.pathfindToPoseFlipped(new Pose2d(4.85, 4.09, Rotation2d.fromDegrees(180)), Constants.kDrivetrain.PATH_CONSTRAINTS);
    m_sysIDDriveRoutine = new DeferredCommand(m_drivetrain::getSysIDDriveRoutine, Set.of(m_drivetrain));
    m_alignWithSpeaker = new AlignWithSpeaker2Command(m_drivetrain, driverController, false, true);
    /*SequentialCommandGroup m_alignWithSpeaker = new SequentialCommandGroup(
        new AlignWithSpeaker1Command(m_drivetrain, driverController, false, true),
        new AlignWithSpeaker2Command(m_drivetrain, driverController, false, true));*/

    /* Shooter */
    m_manualShooter = new ManualShooterCommand(m_shooter, m_drivetrain, m_indexer, operatorController);
    m_resetAlternateAngle = new ResetAlternateAngleCommand(m_shooter);
    m_stow = new StowShooterCommand(m_shooter);
    m_toAmpAngle = new ToAmpPositionCommand(m_shooter, operatorController);
    m_alignAmp = new AlignWithAmpCommand(m_drivetrain, driverController, false, true);
    m_ToClimbPositionCommand = new ToClimbPositionCommand(m_shooter);
    m_shoot = new ShootCommand(m_shooter, m_indexer, m_drivetrain, driverController);
    m_lockClimber = new ClimbLockCommand(m_shooter, operatorController);
    m_subwooferShotCommand = new SubwooferShotCommand(m_shooter, m_indexer, m_drivetrain, driverController);
    m_pass = new PassNoteCommand(m_shooter, m_indexer);

    /* Intake */
    m_runIntakeIn = new RunIntakeCommand(m_intake, 0.5);
    m_runIntakeOut = new RunIntakeCommand(m_intake, -0.5);
    m_storeNote = new StoreNoteSequence(m_indexer, m_intake);
    m_reverseWhileNoteStored = new ReverseWhileNoteStoredCommand(m_intake, m_indexer, operatorController);

    /* Indexer */
    m_manualIndexer = new ManualIndexerCommand(m_indexer, operatorController);
    m_alignNote = new AlignWithNoteCommand(m_drivetrain, m_indexer);

    m_ffData = new RecordFFDataCommand(m_shooter);

    /* LEDs */
    m_signalStoreNote = new SignalStoreNote(m_leds, m_indexer, m_intake);

    /* Tests */
    m_flywheelTest = new FlywheelTest(m_shooter);
    m_drivetrainTest = new DrivetrainTest(m_drivetrain);
    m_intakeTest = new IntakeTest(m_intake);

    // Configure the trigger bindings
    configureDriveBindings();

    m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    m_indexer.setDefaultCommand(m_manualIndexer);
    m_shooter.setDefaultCommand(m_manualShooter);
    m_intake.setDefaultCommand(m_reverseWhileNoteStored);
    m_leds.setDefaultCommand(m_signalStoreNote);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDebugBindings() {

    // /* Drivetrain Bindings */
    // Button.square.whileTrue(m_setDrivePercentOutput);
    // Button.triangle.onTrue(m_resetFieldOrientedHeading);
    // Button.controlPadRight1.whileTrue(m_pathfindToSource);
    // Button.controlPadLeft1.whileTrue(m_pathfindToAmp);
    // Button.rightBumper1.and(Button.triangle).whileTrue(m_driveQuasistaicForward);
    // Button.rightBumper1.and(Button.square).whileTrue(m_driveQuasistaicReverse);
    // Button.rightBumper1.and(Button.cross).whileTrue(m_driveDynamicForward);
    // Button.rightBumper1.and(Button.circle).whileTrue(m_driveDynamicReverse);
    // Button.leftBumper1.onTrue(m_resetToAprilTagPose);
    // Button.circle.whileTrue(m_alignWithSpeaker);

    // /* Shooter Bindings */
    // Button.x.toggleOnTrue(m_prepareShooter);
    // Button.leftBumper2.onTrue(m_resetAlternateAngle);
    // Button.rightBumper2.toggleOnTrue(m_toAmpAngle);
    // Button.leftTrigger2.toggleOnTrue(m_nudgeIndexer);
    // Button.rightTrigger2.onTrue(m_ToClimbPositionCommand);

    // /* Intake Bindings */
    // Button.b.toggleOnTrue(m_runIntakeIn);
    // Button.a.whileTrue(m_runIntakeOut);

    // /* Indexer Bindings */
    // Button.y.whileTrue(m_runIndexerUp);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriveBindings() {

    // ================
    // Driver Controls
    // ================

    /* Drivetrain */
    Button.rightTrigger1.whileTrue(m_shoot);
    Button.leftTrigger1.or(Button.rightBumper2).onTrue(m_stow);
    Button.cross1.toggleOnTrue(m_toAmpAngle);
    Button.triangle1.onTrue(m_resetFieldOrientedHeading);
    Button.square1.whileTrue(m_pathfindToStage);
    Button.leftBumper1.whileTrue(m_alignNote);
    Button.controlPadLeft1.toggleOnTrue(m_sysIDDriveRoutine);

    /* Intake */
    Button.controlPadUp1.onTrue(new InstantCommand(() -> Constants.kIntake.INTAKE_SPEED += 0.1));
    Button.controlPadDown1.onTrue(new InstantCommand(() -> Constants.kIntake.INTAKE_SPEED -= 0.1));
    Button.controlPadRight1.onTrue(m_ffData);
    Button.leftBumper1.or(Button.cross2).onTrue(m_storeNote);

    // ==================
    // Operator Controls
    // ==================

    Button.leftTrigger2.onTrue(m_lockClimber);
    Button.rightTrigger2.onTrue(m_ToClimbPositionCommand);
    Button.triangle2.onTrue(m_runIntakeOut);
    Button.square2.onTrue(m_resetAlternateAngle);
    Button.leftStickClick2.onTrue(m_ffData);
    Button.rightBumper2.whileTrue(m_subwooferShotCommand);
    Button.leftBumper2.onTrue(m_pass);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_autoSelector.getAutoRoutine();

  }

}
