// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

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

import frc.robot.Constants;
import frc.slicelibs.config.SwerveModuleConstants;

public class RealSwerveModuleIO implements SwerveModuleIO {
    private final Rotation2d angleOffset;

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SimpleMotorFeedforward driveFeedforward;
    private final SparkClosedLoopController drivePID;

    private final SparkMax angleMotor;
    private final RelativeEncoder integratedAngleEncoder;
    private final SparkClosedLoopController anglePID;

    private final CANcoder angleEncoder;
    private final StatusSignal<Angle> angleEncoderSignal;

    public RealSwerveModuleIO(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Drive Motor Config */
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotor.configure(Constants.REV_CONFIGS.driveSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getClosedLoopController();
        driveMotor.setCANTimeout(200);
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
        angleEncoder = new CANcoder(moduleConstants.absoluteEncoderID);
        angleEncoderSignal = angleEncoder.getAbsolutePosition();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        angleEncoderSignal.refresh();

        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.absoluteAnglePosition =
            Rotation2d.fromRotations(angleEncoderSignal.getValueAsDouble());
        inputs.integratedAnglePosition =
            Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        inputs.angleVelocityDegreesPerSec =
            integratedAngleEncoder.getVelocity();
        inputs.angleAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleCurrentAmps = angleMotor.getOutputCurrent();
    }

    @Override
    public void runDriveDutyCycle(double percentOutput) {
        driveMotor.set(percentOutput);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setDriveVelocity(double velocity) {
        drivePID.setReference(
            velocity, 
            ControlType.kVelocity, 
            0, 
            driveFeedforward.calculate(velocity));
    }

    @Override
    public void runAngleDutyCycle(double percentOutput) {
        angleMotor.set(percentOutput);
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
        integratedAngleEncoder.setPosition(angleEncoderSignal.getValue().in(Units.Degrees) - angleOffset.getDegrees());
    }
}
