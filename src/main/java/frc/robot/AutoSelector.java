// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.Drivetrain.AlignWithNoteCommand;
import frc.robot.commands.Intake.StoreNote.StoreNoteSequence;
import frc.robot.commands.Shooter.EjectNoteCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinFlywheelsCommand;
import frc.robot.commands.Shooter.SubwooferShotCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * This class primarily manages the creation and updating of the autonomous mode
 * and starting position {@link SendableChooser sendable choosers} on Shuffleboard.
 */
public class AutoSelector {

    public enum StartingPosition {

        PLACEHOLDER("Placeholder");

        public final String value;

        StartingPosition(String value) {

            this.value = value;

        }

    }

    public enum Mode {

        CHOREO_TEST_PATH("Choreo Test Path", false),
        TEST_PATH("Test Path", false),
        CHOREO_TEST_AUTO("Choreo Test Auto", false);

        public final String value;
        public final boolean useStartingPosition;

        Mode(String value, boolean useStartingPosition) {

            this.value = value;
            this.useStartingPosition = useStartingPosition;

        }

    }

    private StartingPosition storedStartingPosition = StartingPosition.PLACEHOLDER;
    private Mode storedMode = Mode.TEST_PATH;

    public final SendableChooser<StartingPosition> startingPositionChooser;
    public final SendableChooser<Mode> modeChooser;

    private Optional<PathPlannerAuto> autoRoutine = Optional.empty();

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;
    private final Shooter m_shooter;
    private final Intake m_intake;
    private final Indexer m_indexer;

    public AutoSelector(Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {

        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_intake = intake;
        m_indexer = indexer;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Placeholder", StartingPosition.PLACEHOLDER);

        startingPositionChooser.onChange((position) -> updateAutoRoutine(position, storedMode));

        modeChooser = new SendableChooser<Mode>();

        modeChooser.setDefaultOption(Mode.CHOREO_TEST_PATH.value, Mode.CHOREO_TEST_PATH);
        modeChooser.addOption(Mode.TEST_PATH.value, Mode.TEST_PATH);
        modeChooser.addOption(Mode.CHOREO_TEST_AUTO.value, Mode.CHOREO_TEST_AUTO);

        modeChooser.onChange((mode) -> updateAutoRoutine(storedStartingPosition, mode));

        AutoBuilder.configure(
            m_drivetrain::getPose,
            m_drivetrain::resetOdometry,
            m_drivetrain::getChassisSpeeds,
            m_drivetrain::runChassisSpeeds,
            new PPHolonomicDriveController(
                new PIDConstants(Constants.kDrivetrain.TRANSLATION_KP),
                new PIDConstants(Constants.kDrivetrain.ROTATION_KP)),
            new RobotConfig(
                Constants.kDrivetrain.MASS,
                Constants.kDrivetrain.MOMENT_OF_INERTIA,
                new ModuleConfig(
                    Constants.kDrivetrain.WHEEL_DIAMETER / 2,
                    Constants.kDrivetrain.MAX_LINEAR_VELOCITY,
                    Constants.kDrivetrain.WHEEL_COEFFICIENT_OF_FRICTION,
                    DCMotor.getKrakenX60(1).withReduction(Constants.kDrivetrain.DRIVE_GEAR_RATIO),
                    Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LIMIT,
                    1),
                Constants.kDrivetrain.TRACK_WIDTH,
                Constants.kDrivetrain.WHEEL_BASE
                ),
            () -> DriverStation.getAlliance().get() == Alliance.Red,
            m_drivetrain);

        NamedCommands.registerCommand("Store Note", new StoreNoteSequence(m_indexer, m_intake));
        NamedCommands.registerCommand("Shoot Note", new ShootCommand(m_shooter, m_indexer, m_drivetrain));
        NamedCommands.registerCommand("Shoot Note Subwoofer", new SubwooferShotCommand(m_shooter, m_indexer, m_drivetrain));
        NamedCommands.registerCommand("Spin Flywheels", new SpinFlywheelsCommand(m_shooter, 3500));
        NamedCommands.registerCommand("Note Align", new AlignWithNoteCommand(m_drivetrain, m_indexer).withTimeout(5));
        NamedCommands.registerCommand("Eject Note", new EjectNoteCommand(m_shooter, m_indexer));

    }

    private void updateAutoRoutine(StartingPosition position, Mode mode) {

        storedStartingPosition = position;
        storedMode = mode;

        try {

            System.out.println("Auto selection changed, updating creator; Starting Position: " + position.value
                + ", Mode: " + mode.value);
            autoRoutine = Optional.of(new PathPlannerAuto(mode.useStartingPosition? position.value + " " + mode.value : mode.value));

        }
        catch (Exception e) {

            DriverStation.reportError(e.getMessage(), true);            
            autoRoutine = Optional.empty();

        }

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d currentPose = m_drivetrain.getPose();

        try {

            initialAutoPose = autoRoutine.get().getStartingPose();

        }
        catch (Exception e) {

            DriverStation.reportError("Selected auto routine '" + (storedMode.useStartingPosition? 
                storedStartingPosition.value + " " + storedMode.value : storedMode.value)+ "' does not exist", false);

        }

        if (currentPose != null && initialAutoPose != null) {

            Transform2d offset = initialAutoPose.minus(currentPose);

            initialAutoPoseXOffset = offset.getX();
            initialAutoPoseYOffset = offset.getY();
            initialAutoPoseRotationOffset = offset.getRotation().getDegrees();

        }

    }

    public Command getAutoRoutine() {

        return autoRoutine.get();

    }

    public String getStartingPosition() {

        return startingPositionChooser.getSelected().value;

    }

    public String getMode() {

        return modeChooser.getSelected().value;

    }

}