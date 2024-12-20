// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;

import frc.robot.Constants;
import frc.slicelibs.util.config.SwerveModuleConstants;
import frc.slicelibs.util.math.Conversions;

public class RealSwerveModuleIO implements SwerveModuleIO {
    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final SparkMax angleMotor;

    private final AnalogEncoder angleEncoder;
    private final RelativeEncoder integratedAngleEncoder;

    private final SimpleMotorFeedforward driveFeedforward;
    private final SparkClosedLoopController anglePID;

    /* Drive Motor Control Requests */
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);

    /* Drive Motor Status Signals */
    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<AngularVelocity> driveVelocitySignal;
    private final StatusSignal<Voltage> driveAppliedVoltsSignal;
    private final StatusSignal<Current> driveCurrentSignal;

    public RealSwerveModuleIO(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.swerveDriveFXConfig);
        driveMotor.getConfigurator().setPosition(0);
        driveFeedforward = new SimpleMotorFeedforward(
            Constants.kDrivetrain.DRIVE_KS, Constants.kDrivetrain.DRIVE_KV, Constants.kDrivetrain.DRIVE_KA);

        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleMotor.configure(Constants.REV_CONFIGS.angleSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        integratedAngleEncoder = angleMotor.getEncoder();
        resetToAbsolute();
        anglePID = angleMotor.getClosedLoopController();
        angleMotor.setCANTimeout(200);

        /* Absolute Encoder Config */
        angleEncoder = new AnalogEncoder(moduleConstants.absoluteEncoderID);
        angleEncoder.setInverted(Constants.kDrivetrain.ABSOLUTE_ENCODER_INVERT);

        /* Drive Motor Status Signals */
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAppliedVoltsSignal = driveMotor.getMotorVoltage();
        driveCurrentSignal = driveMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kDrivetrain.DRIVE_POSITION_FREQUENCY_HZ, drivePositionSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kDrivetrain.DRIVE_DEFAULT_FREQUENCY_HZ,
            driveVelocitySignal,
            driveAppliedVoltsSignal,
            driveCurrentSignal);
        //driveMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltsSignal,
            driveCurrentSignal);

        inputs.drivePositionMeters =
            Conversions.talonToMeters(drivePositionSignal.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
        inputs.driveVelocityMetersPerSec =
            Conversions.talonToMPS(driveVelocitySignal.getValueAsDouble(), Constants.kDrivetrain.WHEEL_CIRCUMFERENCE, Constants.kDrivetrain.DRIVE_GEAR_RATIO);
        inputs.driveAppliedVolts = driveAppliedVoltsSignal.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrentSignal.getValueAsDouble();

        inputs.absoluteAnglePosition =
            Rotation2d.fromDegrees(angleEncoder.get());
        inputs.integratedAnglePosition =
            Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        inputs.angleVelocityDegreesPerSec =
            integratedAngleEncoder.getVelocity();
        inputs.angleAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleCurrentAmps = angleMotor.getOutputCurrent();
    }

    @Override
    public void setDriveDutyCycle(double percentOutput) {
        driveDutyCycleRequest.Output = percentOutput;
        driveMotor.setControl(driveDutyCycleRequest);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveVoltageRequest.Output = volts;
        driveMotor.setControl(driveVoltageRequest);
    }

    @Override
    public void setDriveVelocity(double velocity) {
        driveVelocityRequest.Velocity = velocity;
        driveVelocityRequest.FeedForward = driveFeedforward.calculate(Units.MetersPerSecond.of(velocity)).in(Units.Volts);
        driveMotor.setControl(driveVelocityRequest);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    @Override
    public void setAnglePosition(double position) {
        anglePID.setReference(position, ControlType.kPosition);
    }

    @Override
    public void resetToAbsolute() {
        integratedAngleEncoder.setPosition(angleEncoder.get() - angleOffset.getDegrees());
    }
}
