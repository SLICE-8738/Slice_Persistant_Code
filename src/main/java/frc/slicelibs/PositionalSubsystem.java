// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionalSubsystem extends SubsystemBase {
    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;
    private SparkPIDController[] pids;
    double targetReference;
    ControlType currentControlType;

    public PositionalSubsystem(int[] ids, boolean[] inverted, double kP, double kI, double kD, double positionConversionFactor, double velocityConversionFactor) {
        if (ids.length != inverted.length) throw new IllegalArgumentException("ids and inverted must be the same length");
        
        motors = new CANSparkMax[ids.length];
        encoders = new RelativeEncoder[ids.length];
        pids = new SparkPIDController[ids.length];

        for (int i = 0; i < ids.length; i++) {
            motors[i] = new CANSparkMax(ids[i], CANSparkMax.MotorType.kBrushless);
            motors[i].setInverted(inverted[i]);
            encoders[i] = motors[i].getEncoder();
            pids[i] = motors[i].getPIDController();
            pids[i].setP(kP);
            pids[i].setI(kI);
            pids[i].setD(kD);
            encoders[i].setPositionConversionFactor(positionConversionFactor);
            encoders[i].setVelocityConversionFactor(velocityConversionFactor);
        }

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    public void set(double speed) {
        for (CANSparkMax motor : motors) {
            motor.set(speed);
        }
        currentControlType = ControlType.kDutyCycle;
    }

    public void setVelocity(double velocity) {
        for (SparkPIDController pid : pids) {
            pid.setReference(velocity, ControlType.kVelocity);
        }

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    public void setPosition(double position) {
        for (SparkPIDController pid : pids) {
            pid.setReference(position, ControlType.kPosition);
        }

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    public void setVoltage(double voltage) {
        for (CANSparkMax motor : motors) {
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
