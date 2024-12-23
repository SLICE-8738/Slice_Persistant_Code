package frc.slicelibs.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public final CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        /* Swerve CANcoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.kDrivetrain.ABSOLUTE_ENCODER_INVERT;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    }
}