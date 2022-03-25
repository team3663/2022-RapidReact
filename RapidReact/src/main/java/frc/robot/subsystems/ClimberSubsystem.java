package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.utils.MathUtils.*;

public class ClimberSubsystem extends SubsystemBase {

    /**
     * Defined hook positions.
     * 
     * Grab - Opened to grab the next bar
     * Release - Opened to release the current bar
     * Locked - Locked to hold onto a bar
     * Unknown - Hook is moving between defined positions
     */
    public enum HookPosition {
        Grab,
        Release,
        Locked,
        Unknown
    }

    // Constants
    private static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 80;
    private static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.0;
    private static final double ELEVATOR_MAX_POSITION = 17;
    private static final double ELEVATOR_MIN_POSITION = 0;
    private static final double ELEVATOR_MAX_OFFSET = 1.0;
    private static final double kElevatorP = 0.000153;
    private static final double kElevatorI = 0.000000;
    private static final double kElevatorD = 0.000003;
    private static final double kElevatorIz = 0.000000;
    private static final double kElevatorFF = 0.00029625;
    private static final double kElevatorMaxOutput = 1.000000;
    private static final double kElevatorMinOutput = -0.1;

    private static final int WINDMILL_MOTOR_CURRENT_LIMIT = 80;
    private static final double WINDMILL_POSITION_CONVERSION_FACTOR = 1.0;
    private static final double kWindmillP = 0.000153;
    private static final double kWindmillI = 0.000000;
    private static final double kWindmillD = 0.000003;
    private static final double kWindmillIz = 0.000000;
    private static final double kWindmillFF = 0.00029625;
    private static final double kWindmillMaxOutput = 1.0;
    private static final double kWindmillMinOutput = -1.0;

    private static final int HOOK_MOTOR_CURRENT_LIMIT = 15;
    private static final double HOOK_POSITION_CONVERSION_FACTOR = 1.0;
    private static final double kHookP = 0.1;
    private static final double kHookI = 1e-4;
    private static final double kHookD = 1;
    private static final double kHookIz = 0;
    private static final double kHookFF = 0;
    private static final double kHookMaxOutput = 1.0;
    private static final double kHookMinOutput = -1.0;

    // Motors and associated encoders
    private CANSparkMax elevatorMotor;
    private CANSparkMax windmillMotor1;
    private CANSparkMax windmillMotor2;
    private CANSparkMax redHookMotor;
    private CANSparkMax blueHookMotor;

    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPID;

    private RelativeEncoder windmillEncoder;
    private SparkMaxPIDController windmillPID;

    private RelativeEncoder redHookEncoder;
    private SparkMaxPIDController redHookPID;

    private RelativeEncoder blueHookEncoder;
    private SparkMaxPIDController blueHookPID;

    private double elevatorTargetPosition = 0.0;
    private double elevatorCurrentPosition = 0.0;

    // Network table entries for Shuffleboard
    private NetworkTableEntry elevatorTargetPositionEntry;
    private NetworkTableEntry elevatorCurrentPositionEntry;  

    /**
     * Initilalize the climber subsystem
     * 
     * Elevator - One Neo  gearbox ???, Winch drum diameter ???
     * Windmill - 2 Neo with 100:1 gearboxes, chain 20-tooth drive gear, 72-tooth driven gear
     * Hooks - Neo 550, ??? gear reduction
     *
     */
    public ClimberSubsystem(int elevatorMotorCanId, int windmillMotor1CanId, int windmillMotor2CanId, int redHookMotorCanId, int blueHookMotorCanId) {

        // Setup the elevator motor controllers
        elevatorMotor = new CANSparkMax(windmillMotor1CanId, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevatorPID = elevatorMotor.getPIDController();
        elevatorPID.setP(kElevatorP);
        elevatorPID.setI(kElevatorI);
        elevatorPID.setD(kElevatorD);
        elevatorPID.setIZone(kElevatorIz);
        elevatorPID.setFF(kElevatorFF);
        elevatorPID.setOutputRange(kElevatorMinOutput, kElevatorMaxOutput);

        // Setup the motor controllers for the windmill
        windmillMotor1 = new CANSparkMax(windmillMotor1CanId, MotorType.kBrushless);
        windmillMotor1.restoreFactoryDefaults();
        windmillMotor1.setInverted(true);
        windmillMotor1.setIdleMode(IdleMode.kBrake);
        windmillMotor1.setSmartCurrentLimit(WINDMILL_MOTOR_CURRENT_LIMIT);
        windmillEncoder = windmillMotor1.getEncoder();

        windmillMotor2 = new CANSparkMax(windmillMotor2CanId, MotorType.kBrushless);
        windmillMotor2.restoreFactoryDefaults();
        windmillMotor2.setIdleMode(IdleMode.kBrake);
        windmillMotor2.setSmartCurrentLimit(WINDMILL_MOTOR_CURRENT_LIMIT);
        windmillMotor2.follow(windmillMotor1, true);

        windmillEncoder = windmillMotor1.getEncoder();
        windmillEncoder.setPositionConversionFactor(WINDMILL_POSITION_CONVERSION_FACTOR);
        windmillPID = windmillMotor1.getPIDController();
        windmillPID.setP(kWindmillP);
        windmillPID.setI(kWindmillI);
        windmillPID.setD(kWindmillD);
        windmillPID.setIZone(kWindmillIz);
        windmillPID.setFF(kWindmillFF);
        windmillPID.setOutputRange(kWindmillMinOutput, kWindmillMaxOutput);

        // Setup the motor controllers for the hooks
        redHookMotor = new CANSparkMax(redHookMotorCanId, MotorType.kBrushless);
        redHookMotor.restoreFactoryDefaults();
        redHookMotor.setIdleMode(IdleMode.kBrake);
        redHookMotor.setSmartCurrentLimit(HOOK_MOTOR_CURRENT_LIMIT);
        redHookEncoder = redHookMotor.getEncoder();
        redHookEncoder.setPositionConversionFactor(HOOK_POSITION_CONVERSION_FACTOR);
        redHookPID = redHookMotor.getPIDController();
        redHookPID.setP(kHookP);
        redHookPID.setI(kHookI);
        redHookPID.setD(kHookD);
        redHookPID.setIZone(kHookIz);
        redHookPID.setFF(kHookFF);
        redHookPID.setOutputRange(kHookMinOutput, kHookMaxOutput);

        blueHookMotor = new CANSparkMax(blueHookMotorCanId, MotorType.kBrushless);
        blueHookMotor.restoreFactoryDefaults();
        blueHookMotor.setIdleMode(IdleMode.kBrake);
        blueHookMotor.setSmartCurrentLimit(HOOK_MOTOR_CURRENT_LIMIT);
        blueHookEncoder = blueHookMotor.getEncoder();
        blueHookEncoder.setPositionConversionFactor(HOOK_POSITION_CONVERSION_FACTOR);
        blueHookPID = redHookMotor.getPIDController();
        blueHookPID.setP(kHookP);
        blueHookPID.setI(kHookI);
        blueHookPID.setD(kHookD);
        blueHookPID.setIZone(kHookIz);
        blueHookPID.setFF(kHookFF);
        blueHookPID.setOutputRange(kHookMinOutput, kHookMaxOutput);

        initTelemetry();
    }

    @Override
    public void periodic() {
        home();

        // Read the current positions back from our various encoders
        elevatorCurrentPosition = elevatorEncoder.getPosition();

        updateTelemetry();
    }

    private void home() {
        homeElevator();
        homeWindmill();
        homeRedHook();
        homeBlueHook();
    }

    // ---------------------------------------------------------------------------
    // Elevator control methods
    // ---------------------------------------------------------------------------

    /**
     * Move the elevator to its home position to establish our zero reference
     */
    private void homeElevator() {

    }

    /**
     * Set target position of the elevator.
     * 
     * @param position
     */
    public void setElevatorPosition(double position) {
        elevatorTargetPosition = position;

        if (elevatorTargetPosition > ELEVATOR_MAX_POSITION) {
            elevatorTargetPosition = ELEVATOR_MAX_POSITION;
        } else if (elevatorTargetPosition < ELEVATOR_MIN_POSITION) {
            elevatorTargetPosition = ELEVATOR_MIN_POSITION;
        }

        elevatorPID.setReference(position, ControlType.kPosition);       
    }

    /**
     * Check to see if the elevator has reached the target position.
     * 
     * @return - True if elevator has reached the target position.
     */
    public boolean elevatorAtTarget() {
        return WithinDelta(elevatorCurrentPosition, elevatorTargetPosition, ELEVATOR_MAX_OFFSET);
    }


    // ---------------------------------------------------------------------------
    // Windmill control methods
    // ---------------------------------------------------------------------------

    /**
     * Move the windmill to its home position to establish our zero reference
     */
    private void homeWindmill() {


    }
    /**
     * Set target angle for windmill to rotate to.
     * 
     * @param angle - Target angle.
     */
    public void setWindmillAngle(double angle) {

    }

    /**
     * Check to see if the windwill has reached the current target angle.
     * 
     * @return - True if the windmill has reached the target position.
     */
    public boolean windmillAtTarget() {
        return false;
    }


    // ---------------------------------------------------------------------------
    // Hook control methods
    // ---------------------------------------------------------------------------

    /**
     * Move the red hook to its home position to establish our zero reference
     */
    private void homeRedHook() {

    }

    /**
     * Set the position of the Red climbing hook
     * 
     * @param position
     */
    public void setRedHookPosition(HookPosition position) {

    }

    /**
     * Get the current position of the Red climbing hook
     * @return - HookPosition enum telling what position the hook is currently in.
     */
    public HookPosition getRedHookPosition() {
        return HookPosition.Unknown;
    }

	public boolean redHookAtTarget() {
		return false;
	}


     /**
     * Move the blue hook to its home position to establish our zero reference
     */
    private void homeBlueHook() {

    }   

    /**
     * Set the position of the Blue climbing hook
     * 
     * @param position
     */
    public void setBlueHookPosition(HookPosition position) {

    }

    /**
     * Get the current position of the Blue climbing hook
     * @return - HookPosition enum telling what position the hook is currently in.
     */
    public HookPosition getBlueHookPosition() {
        return HookPosition.Unknown;
    }

	public boolean blueHookAtTarget() {
		return false;
	}

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    private void initTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");

        // Elevator Data
        elevatorTargetPositionEntry = tab.add("Target Position", 0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        elevatorCurrentPositionEntry = tab.add("Current Position", 0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();

        // Windmill Data

        // Hook Data

    }

    private void updateTelemetry() {

        // Elevator
        elevatorTargetPositionEntry.setNumber(elevatorTargetPosition);
        elevatorCurrentPositionEntry.setNumber(elevatorCurrentPosition);

        // Windmill

        // Hooks
    }
}