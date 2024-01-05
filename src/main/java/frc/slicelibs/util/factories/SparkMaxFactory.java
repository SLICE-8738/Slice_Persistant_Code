package frc.slicelibs.util.factories;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.slicelibs.util.config.REVConfigs;
import frc.slicelibs.util.config.REVConfigs.SparkMaxConfiguration;

import java.util.ArrayList;

public class SparkMaxFactory {

    private static final ArrayList<CANSparkMax> sparkMaxes = new ArrayList<CANSparkMax>();

    /**
     * Creates a CANSparkMax object with a default configuration optimized for less significant motors.
     */
    public static CANSparkMax createDefaultSparkMax(int id) {

        return createSparkMax(id, REVConfigs.defaultSparkMaxConfig);

    }

    private static void handleREVLibError(int id, REVLibError error, String message) {

        if (error != REVLibError.kOk) {
            DriverStation.reportError(
                    "Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }

    }

    /** Creates a CANSparkMax object with specifiable configuration settings. */
    public static CANSparkMax createSparkMax(int id, SparkMaxConfiguration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
        handleREVLibError(id, sparkMax.setCANTimeout(200), "set timeout");

        sparkMax.restoreFactoryDefaults();

        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, config.statusFrame0PeriodMs), "set status0 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, config.statusFrame1PeriodMs), "set status1 rate");
        handleREVLibError(id, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, config.statusFrame2PeriodMs), "set status2 rate");

        sparkMax.clearFaults();

        handleREVLibError(id, sparkMax.setIdleMode(config.idleMode), "set idle");
        sparkMax.setInverted(config.inverted);
        handleREVLibError(id, sparkMax.setOpenLoopRampRate(config.openLoopRampRate), "set open loop ramp");
        handleREVLibError(id, sparkMax.setClosedLoopRampRate(config.closedLoopRampRate), "set closed loop ramp");

        if (config.enableVoltageCompensation) {
            handleREVLibError(id, sparkMax.enableVoltageCompensation(config.nominalVoltage), "voltage compensation");
        } 
        else {
            handleREVLibError(id, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        handleREVLibError(id, sparkMax.setSmartCurrentLimit(config.currentLimit), "current limit");

        sparkMaxes.add(sparkMax);

        return sparkMax;

    }

    /** Burns the configurations of all motors constructed in the class to flash. */
    public static void flashAll() {

        for(int i = 0; i < sparkMaxes.size(); i++) {

            sparkMaxes.get(i).burnFlash();
            
        }

    }

}
