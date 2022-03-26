package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

    private final double FEEDER_GEAR_RATIO_MULTIPLIER = 1;

    // Feeder PID constants
    private final double KP = 0.0001;
    private final double KI = 0.000001;
    private final double KD = 0.0009;

    private final int FEED_RPM_STOPPED = 0;
    private final int FEED_RPM_SHOOT = 4000; // how fast the feeder should be running when we are shooting

    // Subsystems internal data
    private CANSparkMax feedMotor;
    private RelativeEncoder feedEncoder;
    private SparkMaxPIDController feedPID;

    private DigitalInput entrySensor;
    private DigitalInput exitSensor;

    // network table entries for Shuffleboard
    private NetworkTableEntry feederRPMEntry;
    private NetworkTableEntry entrySensorEntry;
    private NetworkTableEntry exitSensorEntry;

    public FeederSubsystem(int feedMotorCanId, int entrySensorDio, int exitSensorDio) {

        feedMotor = new CANSparkMax(feedMotorCanId, MotorType.kBrushless);
        feedMotor.setIdleMode(IdleMode.kBrake);

        feedPID = feedMotor.getPIDController();
        feedPID.setP(KP);
        feedPID.setI(KI);
        feedPID.setD(KD);

        feedEncoder = feedMotor.getEncoder();
        feedEncoder.setVelocityConversionFactor(FEEDER_GEAR_RATIO_MULTIPLIER);

        // Sensors for Feeder
        entrySensor = new DigitalInput(entrySensorDio);
        exitSensor = new DigitalInput(exitSensorDio);

        initTelemetry();
    }

    /**
     * Perform background processing for feeder system.
     */
    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void start() {
        feedPID.setReference(FEED_RPM_SHOOT, ControlType.kVelocity);
    }

    public void stop() {
        feedPID.setReference(FEED_RPM_STOPPED, ControlType.kVelocity);
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------
    private void initTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Feeder");

        feederRPMEntry = tab.add("Feeder RPM", 0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        entrySensorEntry = tab.add("Entry Sensor", false)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();

        exitSensorEntry = tab.add("Exit Sensor", false)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();
    }

    private void updateTelemetry() {
        feederRPMEntry.setNumber(feedMotor.getEncoder().getVelocity());
        entrySensorEntry.forceSetBoolean(entrySensor.get());
        exitSensorEntry.forceSetBoolean(exitSensor.get());
    }
}