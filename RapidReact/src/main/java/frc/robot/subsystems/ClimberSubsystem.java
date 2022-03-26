/*
* Climber hardware description
* 
* Elevator
*   - Motor: 1x Neo
*   - Gearing: 27:1 gearbox
*   - Winch drum: Diameter 1-3/16 inches (3.73 inch circumference)
*   = 0.138 inches of travel per motor revolution (7.237 RPI of travel)
*   - Maximum extension: 17 inches
*
* Windmill
*   - Motors: 2x Neos
*   - Gearing 100:1 gearboxes -> 20-tooth drive sprocket -> 72-tooth driven sprocket
*   - Total reduction 360:1

* Hooks - Neo 550, 27:1 gear reduction to 16-tooth sprocket  -> 72 tooth sprocket
*
*/

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
    private static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;
    private static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 0.138;
    private static final double ELEVATOR_MAX_POSITION = 17;
    private static final double ELEVATOR_MIN_POSITION = 0;
    private static final double ELEVATOR_MAX_OFFSET = 0.125;
    private static final double kElevatorP = 0.000153;
    private static final double kElevatorI = 0.000000;
    private static final double kElevatorD = 0.000003;
    private static final double kElevatorIz = 0.000000;
    private static final double kElevatorFF = 0.00029625;
    private static final double kElevatorMaxOutput = 1.000000;
    private static final double kElevatorMinOutput = -0.1;

    private static final int WINDMILL_MOTOR_CURRENT_LIMIT = 80;
    private static final double WINDMILL_POSITION_CONVERSION_FACTOR = 1.0;
    private static final double WINDMILL_MAX_ANGLE = 360;
    private static final double WINDMILL_MIN_ANGLE = 0;
    private static final double WINDMILL_MAX_OFFSET = 2;
    private static final double kWindmillP = 0.000153;
    private static final double kWindmillI = 0.000000;
    private static final double kWindmillD = 0.000003;
    private static final double kWindmillIz = 0.000000;
    private static final double kWindmillFF = 0.00029625;
    private static final double kWindmillMaxOutput = 1.0;
    private static final double kWindmillMinOutput = -1.0;

    // Motors and associated encoders
    private CANSparkMax elevatorMotor;
    private CANSparkMax windmillMotor1;
    private CANSparkMax windmillMotor2;

    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPID;

    private RelativeEncoder windmillEncoder;
    private SparkMaxPIDController windmillPID;

    private boolean elevatorInitialized = false;
    private double elevatorTargetPosition = 0.0;
    private double elevatorCurrentPosition = 0.0;

    private boolean windmillInitialized = false;
    private double windmillTargetAngle = 0.0;
    private double windmillCurrentAngle = 0.0;

    private Hook redHook;
    private Hook blueHook;

    // Network table entries for Shuffleboard
    private NetworkTableEntry elevatorInitializedEntry;
    private NetworkTableEntry elevatorTargetPositionEntry;
    private NetworkTableEntry elevatorCurrentPositionEntry;
    private NetworkTableEntry elevatorMotorCurrentEntry;

    private NetworkTableEntry windmillInitializedEntry;
    private NetworkTableEntry windmillTargetAngleEntry;
    private NetworkTableEntry windmillCurrentAngleEntry;
    private NetworkTableEntry windmillMotor1CurrentEntry;
    private NetworkTableEntry windmillMotor2CurrentEntry;

    /**
     * Initilalize the climber subsystem
     */
    public ClimberSubsystem(int elevatorMotorCanId, int windmillMotor1CanId, int windmillMotor2CanId,
            int redHookMotorCanId, int blueHookMotorCanId) {

        // Setup the elevator motor controller
        elevatorMotor = new CANSparkMax(windmillMotor1CanId, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(ELEVATOR_MOTOR_CURRENT_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR);
        elevatorEncoder.setPosition(1.0); // The initial encoder position must be greater than zero for homing function to work.
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

        redHook = new Hook(redHookMotorCanId);
        blueHook = new Hook(blueHookMotorCanId);

        initTelemetry();
    }

    @Override
    public void periodic() {

        // Call the subsystem initialization code each component has completed initialization
        // this call becomes essentially a noop
        initialize();

        // Read the current positions back from each of our encoder
        elevatorCurrentPosition = elevatorEncoder.getPosition();
        windmillCurrentAngle = windmillEncoder.getPosition();

        updateTelemetry();
    }

    private void initialize() {
        initElevator();
        initWindmill();
        redHook.init();
        blueHook.init();
    }


    // ---------------------------------------------------------------------------
    // Elevator control methods
    // ---------------------------------------------------------------------------

    /**
     * Move the elevator to its home position to establish our zero reference
     */
    private void initElevator() {

        if (elevatorInitialized) {
            return;
        }

        // If the elevator has not stopped moving yet then let it keep running and return.
        if (elevatorEncoder.getPosition() > 0.0) {
            elevatorEncoder.setPosition(0.0);
            elevatorMotor.set(0.05);
            return;
        }

        // If we got here then the elevator has stopped and the cables are taut so we are at our zero position.
        elevatorInitialized = true;
        elevatorMotor.set(0);
        elevatorEncoder.setPosition(ELEVATOR_MIN_POSITION);
    }

    /**
     * Set target position of the elevator.
     * 
     * @param position - Desired height of the elevator in inches.
     */
    public void setElevatorPosition(double position) {
        elevatorTargetPosition = ClipToRange(position, ELEVATOR_MIN_POSITION, ELEVATOR_MAX_POSITION);
        elevatorPID.setReference(elevatorTargetPosition, ControlType.kPosition);
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
    private void initWindmill() {

        windmillInitialized = true;
    }

    /**
     * Set target angle for windmill to rotate to.
     * 
     * @param angle - Target angle.
     */
    public void setWindmillAngle(double angle) {
        windmillTargetAngle = ClipToRange(angle, WINDMILL_MIN_ANGLE, WINDMILL_MAX_ANGLE);
        windmillPID.setReference(windmillTargetAngle, ControlType.kPosition);
    }

    /**
     * Check to see if the windwill has reached the current target angle.
     * 
     * @return - True if the windmill has reached the target position.
     */
    public boolean windmillAtTarget() {
        return WithinDelta(windmillCurrentAngle, windmillTargetAngle, WINDMILL_MAX_OFFSET);
    }


    // ---------------------------------------------------------------------------
    // Hook control methods
    // ---------------------------------------------------------------------------

    public void setRedHookPosition(HookPosition position) {
        redHook.setPosition(position);
    }

    public HookPosition getRedHookPosition() {
        return redHook.getPosition();
    }

    public boolean redHookAtTarget() {
        return redHook.atTarget();
    }


    public void setBlueHookPosition(HookPosition position) {
        blueHook.setPosition(position);
    }

    public HookPosition getBlueHookPosition() {
        return blueHook.getPosition();
    }

    public boolean blueHookAtTarget() {
        return blueHook.atTarget();
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    private void initTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");

        // Elevator Data
        elevatorInitializedEntry = tab.add("Initialized", 0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        elevatorTargetPositionEntry = tab.add("Target Position", 0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();

        elevatorCurrentPositionEntry = tab.add("Current Position", 0)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        elevatorMotorCurrentEntry = tab.add("Motor Amps", 0)
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();


        // Windmill Data
        windmillInitializedEntry = tab.add("Initialized", 0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        windmillTargetAngleEntry = tab.add("Target Angle", 0)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();

        windmillCurrentAngleEntry = tab.add("Current Angle", 0)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();

        windmillMotor1CurrentEntry = tab.add("Motor1 Amps", 0)
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();

        windmillMotor2CurrentEntry = tab.add("Motor2 Amps", 0)
                .withPosition(4, 1)
                .withSize(1, 1)
                .getEntry();

        // Hook Data

    }

    private void updateTelemetry() {

        // Elevator
        elevatorInitializedEntry.forceSetBoolean(elevatorInitialized);
        elevatorTargetPositionEntry.setNumber(elevatorTargetPosition);
        elevatorCurrentPositionEntry.setNumber(elevatorCurrentPosition);
        elevatorMotorCurrentEntry.setNumber( elevatorMotor.getOutputCurrent());

        // Windmill
        windmillInitializedEntry.forceSetBoolean(windmillInitialized);
        windmillTargetAngleEntry.setNumber(windmillTargetAngle);
        windmillCurrentAngleEntry.setNumber(windmillCurrentAngle);
        windmillMotor1CurrentEntry.setNumber( windmillMotor1.getOutputCurrent());
        windmillMotor2CurrentEntry.setNumber( windmillMotor2.getOutputCurrent());
    
        // Hooks
    }


    /**
     * Nested class that encapsulates the climbing hooks
     */
    private class Hook
    {
        private static final int MOTOR_CURRENT_LIMIT = 15;
        private static final double POSITION_CONVERSION_FACTOR = 1.0;
        private static final double kP = 0.1;
        private static final double kI = 1e-4;
        private static final double kD = 1;
        private static final double kIz = 0;
        private static final double kFF = 0;
        private static final double kMaxOutput = 1.0;
        private static final double kMinOutput = -1.0;

        private CANSparkMax motor;
        private RelativeEncoder encoder;
        private SparkMaxPIDController pidController;
    
        public Hook(int motorCanId) {

            motor = new CANSparkMax(motorCanId, MotorType.kBrushless);
            motor.restoreFactoryDefaults();
            motor.setIdleMode(IdleMode.kBrake);
            motor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
            encoder = motor.getEncoder();
            encoder.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
            pidController = motor.getPIDController();
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);
        }

        /**
         * Move the hook to its home position to establish our zero reference
         */
        private void init() {

        }

        /**
         * Set the position of the climbing hook
         * 
         * @param position
         */
        public void setPosition(HookPosition position) {

        }

        /**
         * Get the current position of the climbing hook
         * 
         * @return - HookPosition enum telling what position the hook is currently in.
         */
        public HookPosition getPosition() {
            return HookPosition.Unknown;
        }

        public boolean atTarget() {
            return false;
        }
    }
}