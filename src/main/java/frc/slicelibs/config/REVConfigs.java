package frc.slicelibs.config;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class REVConfigs {

    public final SparkMaxConfig defaultVelocitySparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig defaultPositionSparkMaxConfig = new SparkMaxConfig();
    public final SparkMaxConfig angleSparkMaxConfig = new SparkMaxConfig();

    public REVConfigs() {
        /* Default Velocity Motor Configuration */

            /* Motor Invert and Idle Mode */
            defaultVelocitySparkMaxConfig.inverted(false);
            defaultVelocitySparkMaxConfig.idleMode(IdleMode.kBrake);

            /* Current Limiting */
            defaultVelocitySparkMaxConfig.smartCurrentLimit(30);

            /* Open and Closed Loop Ramping */
            defaultVelocitySparkMaxConfig.openLoopRampRate(0);
            defaultVelocitySparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            defaultVelocitySparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(200);
            defaultVelocitySparkMaxConfig.signals.primaryEncoderPositionPeriodMs(1000);

            /* Voltage Compensation */
            defaultVelocitySparkMaxConfig.voltageCompensation(12);

        /* Default Position Motor Configuration */

            /* Motor Invert and Idle Mode */
            defaultPositionSparkMaxConfig.inverted(false);
            defaultPositionSparkMaxConfig.idleMode(IdleMode.kBrake);

            /* Current Limiting */
            defaultPositionSparkMaxConfig.smartCurrentLimit(20);

            /* Open and Closed Loop Ramping */
            defaultPositionSparkMaxConfig.openLoopRampRate(0);
            defaultPositionSparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            defaultPositionSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(1500);
            defaultPositionSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(300);

            /* Voltage Compensation */
            defaultPositionSparkMaxConfig.voltageCompensation(12);

        /* Swerve Module Angle Motor Configuration */

            /* Motor Invert and Idle Mode */
            angleSparkMaxConfig.inverted(Constants.kDrivetrain.ANGLE_INVERT);
            angleSparkMaxConfig.idleMode(Constants.kDrivetrain.ANGLE_IDLE_MODE);

            /* Current Limiting */
            angleSparkMaxConfig.smartCurrentLimit(Constants.kDrivetrain.ANGLE_CURRENT_LIMIT);

            /* Open and Closed Loop Ramping */
            angleSparkMaxConfig.openLoopRampRate(0);
            angleSparkMaxConfig.closedLoopRampRate(0);

            /* Status Frame Periods */
            angleSparkMaxConfig.signals.primaryEncoderVelocityPeriodMs(Constants.kDrivetrain.ANGLE_VELOCITY_PERIOD_MS);
            angleSparkMaxConfig.signals.primaryEncoderPositionPeriodMs(Constants.kDrivetrain.ANGLE_POSITION_PERIOD_MS);

            /* Voltage Compensation */
            angleSparkMaxConfig.voltageCompensation(12);

            /* Conversion Factors */
            angleSparkMaxConfig.encoder.positionConversionFactor(Constants.kDrivetrain.ANGLE_POSITION_CONVERSION_FACTOR);
            angleSparkMaxConfig.encoder.velocityConversionFactor(Constants.kDrivetrain.ANGLE_VELOCITY_CONVERSION_FACTOR);

            /* PID */
            angleSparkMaxConfig.closedLoop.p(Constants.kDrivetrain.DRIVE_KP);
            angleSparkMaxConfig.closedLoop.i(Constants.kDrivetrain.DRIVE_KI);
            angleSparkMaxConfig.closedLoop.d(Constants.kDrivetrain.DRIVE_KD);
    }

}
