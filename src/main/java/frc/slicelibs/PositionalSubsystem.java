// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is an extension of WPILibs SubsystemBase that implements integrated functionality for positional and velocity based PID control.
 * It it intended to be used as a subclass, and the superclass can add additional functionality like sensors or additional motors.
 */
public class PositionalSubsystem extends SubsystemBase {
    private CANSparkMax[] motors;
    private RelativeEncoder[] encoders;
    private SparkPIDController[] pids;
    double targetReference;
    ControlType currentControlType;

    /**
     * Construct a new Positional Subsystem
     * @param ids an array of CAN ids for every positionally controlled motor in the subsystem. All motors should rotate together, if additional motors should spin seperately, add them in the superclass
     * @param inverted an array of booleans indicating whether the motor of the corresponding CAN id is inverted. This should be the same length as ids.
     * @param kP the p gain of the subsystem
     * @param kI the i gain of the subsystem
     * @param kD the d gain of the subsystem
     * @param positionConversionFactor the number that, when multiplied by the encoder's rotations, gives the position of the subsystem in proper units
     * @param velocityConversionFactor the number that, when multiplied by the encoder's rpm, gives the velocity of the subsystem in proper units
     */
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

    /**
     * Directly sets the speed of all positional motors via duty cycle control
     * @param speed a value from -1 to 1
     */
    public void set(double speed) {
        for (CANSparkMax motor : motors) {
            motor.set(speed);
        }
        currentControlType = ControlType.kDutyCycle;
    }

    /**
     * Sets the target speed of the subsystem, which it will approach using its internal PID controller.
     * @param velocity a value in the units defined by the velocityConversionFactor
     */
    public void setVelocity(double velocity) {
        for (SparkPIDController pid : pids) {
            pid.setReference(velocity, ControlType.kVelocity);
        }

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    /**
     * Sets the target psotiion of the subsystem, which it will approach using its internal PID controller.
     * @param velocity a value in the units defined by the positionConversionFactor
     */
    public void setPosition(double position) {
        for (SparkPIDController pid : pids) {
            pid.setReference(position, ControlType.kPosition);
        }

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    /**
     * Directly sets the voltage being supplied to all positional motors
     * @param voltage volts supplied to each motors (this should generally be between -12 and 12)
     */
    public void setVoltage(double voltage) {
        for (CANSparkMax motor : motors) {
            motor.setVoltage(voltage);
        }
        currentControlType = ControlType.kVoltage;
    }

    /**
     * Changes the encoders of all positional motors to read that they are at the given position
     * @param position a value in the units defined by the positionConversionFactor
     */
    public void setEncoderPosition(double position) {
        for (RelativeEncoder encoder: encoders) {
            encoder.setPosition(position);
        }
    }

    /**
     * Returns the average velocity reading across all positional motors.
     * @return the subsystems's velocity, in the units defined by the velocityConversionFactor
     */
    public double getVelocity() {
        double velocity = 0;
        for (RelativeEncoder encoder : encoders) {
            velocity += encoder.getVelocity();
        }
        return velocity / encoders.length;
    }

    /**
     * Returns the average position reading across all positional motors.
     * @return the subsystems's position, in the units defined by the positionConversionFactor
     */
    public double getPosition() {
        double position = 0;
        for (RelativeEncoder encoder : encoders) {
            position += encoder.getPosition();
        }
        return position / encoders.length;
    }

    /**
     * Returns whether the subsystem's position or velocity (depending on which control type was used last) is close to the target value
     * @param threshold the maximum acceptable error, in the units defined by either positionConversionFactor or velocityConversionFactor
     * @return true if the subsystem is near its target, false if otherwise (or if the subsystem was last controlled using set() or setVoltage())
     */
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
