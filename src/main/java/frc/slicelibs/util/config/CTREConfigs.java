package frc.slicelibs.util.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public final TalonFXConfiguration flywheelFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {
        /** Swerve Drive Motor Configuration */

            /* Motor Invert and Neutral Mode */
            var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
            driveMotorOutput.Inverted = Constants.kDrivetrain.DRIVE_INVERT;
            driveMotorOutput.NeutralMode = Constants.kDrivetrain.DRIVE_IDLE_MODE;

            /* Current Limiting */
            var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
            driveCurrentLimits.SupplyCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT;
            driveCurrentLimits.SupplyCurrentLimit = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LIMIT;
            driveCurrentLimits.SupplyCurrentLowerLimit = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LOWER_LIMIT;
            driveCurrentLimits.SupplyCurrentLowerTime = Constants.kDrivetrain.DRIVE_SUPPLY_CURRENT_LOWER_TIME;

            driveCurrentLimits.StatorCurrentLimitEnable = Constants.kDrivetrain.DRIVE_ENABLE_STATOR_CURRENT_LIMIT;
            driveCurrentLimits.StatorCurrentLimit = Constants.kDrivetrain.DRIVE_STATOR_CURRENT_LIMIT;

            /* PID */
            var driveSlot0 = swerveDriveFXConfig.Slot0;
            driveSlot0.kP = Constants.kDrivetrain.DRIVE_KP;
            driveSlot0.kI = Constants.kDrivetrain.DRIVE_KI;
            driveSlot0.kD = Constants.kDrivetrain.DRIVE_KD;

            /* Open and Closed Loop Ramping */
            swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;
            swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kDrivetrain.OPEN_LOOP_RAMP;

            swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;
            swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kDrivetrain.CLOSED_LOOP_RAMP;
    }
}