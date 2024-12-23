// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PositionalSubsystem extends SubsystemBase {
    private SparkMax[] motors;
    private RelativeEncoder[] encoders;
    private SparkClosedLoopController[] pids;
    double targetReference;
    ControlType currentControlType;

    public PositionalSubsystem(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor) {
        if (ids.length != inverted.length) throw new IllegalArgumentException("ids and inverted must be the same length");
        
        motors = new SparkMax[ids.length];
        encoders = new RelativeEncoder[ids.length];
        pids = new SparkClosedLoopController[ids.length];

        for (int i = 0; i < ids.length; i++) {
            motors[i] = new SparkMax(ids[i], MotorType.kBrushless);
            motors[i].configure(
                Constants.REV_CONFIGS.defaultPositionSparkMaxConfig
                    .inverted(inverted[i])
                    .apply(new ClosedLoopConfig()
                        .p(kP)
                        .i(kI)
                        .d(kD))
                    .apply(new EncoderConfig()
                        .positionConversionFactor(positionConversionFactor)
                        .velocityConversionFactor(velocityConversionFactor)), 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
            encoders[i] = motors[i].getEncoder();
            pids[i] = motors[i].getClosedLoopController();
        }

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        for (SparkMax motor : motors) {
            motor.set(speed);
        }
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        for (SparkClosedLoopController pid : pids) {
            pid.setReference(velocity, ControlType.kVelocity);
        }

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        for (SparkClosedLoopController pid : pids) {
            pid.setReference(position, ControlType.kPosition);
        }

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        for (SparkMax motor : motors) {
            motor.setVoltage(voltage);
        }
        currentControlType = ControlType.kVoltage;
    }

    public void setEncoderPosition(double position) {
        for (RelativeEncoder encoder: encoders) {
            encoder.setPosition(position);
        }
    }

    public double getVelocity() {
        double velocity = 0;
        for (RelativeEncoder encoder : encoders) {
            velocity += encoder.getVelocity();
        }
        return velocity / encoders.length;
    }

    public double getPosition() {
        double position = 0;
        for (RelativeEncoder encoder : encoders) {
            position += encoder.getPosition();
        }
        return position / encoders.length;
    }

    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }
  
}
