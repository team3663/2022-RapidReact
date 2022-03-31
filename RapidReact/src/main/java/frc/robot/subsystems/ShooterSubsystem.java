package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.FiringSolution;
import frc.robot.utils.Ranger;

public class ShooterSubsystem extends SubsystemBase {

    private enum MotorState {
        STOPPED,
        IDLE, // running at a constant rpm
        SHOOTING, // rpm changed by command
        STOPPING
    }

    // Subsystem Constants
    private static final int IDLE_CURRENT = 13; 
    private static final int MAX_CURRENT = 60;
    private double highestCurrent = 0;

    private static final double MAX_RPM = 6000;
    private double IDLE_RPM;
    private double IDLE_ANGLE;
    private static final double shooterBeltRatio = 0.66;
    private static final double speedIncrement = 100;
    private static final double speedMarginPercent = 0.02;

    private static final double MAX_HOOD_ANGLE = 85;
    private static final double MIN_HOOD_ANGLE = 67;
    private static final double HOOD_LOWER_LIMIT = 0;
    private static final double HOOD_UPPER_LIMIT = 13.5;
    private static final double ROTATIONS_PER_DEGREE = (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT)
            / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
    private static final double angleIncrement = 1;

    // Shooter PID coefficients constants
    private static final double kShooterP = 0.001;
    private static final double kShooterI = 0.000000;
    private static final double kShooterD = 0.0;
    private static final double kShooterIz = 0.000000;
    private static final double kShooterFF = 0.00028;
    private static final double kShooterMaxOutput = 1.000000;
    private static final double kShooterMinOutput = 0.000000;

    // Hood PID coefficients
    private static final double kHoodP = 0.1;
    private static final double kHoodI = 1e-4;
    private static final double kHoodD = 1;
    private static final double kHoodIz = 0;
    private static final double kHoodFF = 0;
    private static final double kHoodMaxOutput = 1;
    private static final double kHoodMinOutput = -1;

    private Ranger ranger;
    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private RelativeEncoder shooterEncoder;
    private SparkMaxPIDController shooterPidController;

    private CANSparkMax hoodMotor;
    private RelativeEncoder hoodEncoder;
    private SparkMaxPIDController hoodPidController;
    private DigitalInput hoodLimit;

    private MotorState motorState = MotorState.STOPPED;
    private double currentSpeed = 0;
    private double targetSpeed = 0;
    private double speedError;
    private double speedErrorPercent;

    private boolean parkingHood = true;
    private double currentAngle = 0;
    private double targetAngle = 0;

    private double currentRange = 0.0;
    private double currentXOffset = 0;

    public boolean aligned = false;

    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry targetSpeedEntry;
    private NetworkTableEntry shooterEncoderEntry;
    private NetworkTableEntry speedErrorEntry;
    private NetworkTableEntry speedErrorPercentEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry hoodEncoderEntry;
    private NetworkTableEntry hoodLimitSwitchEntry;
    private NetworkTableEntry readyToShootEntry;
    private NetworkTableEntry currentRangeEntry;
    private NetworkTableEntry currentXEntry;
    private NetworkTableEntry highestCurrentEntry;
    private NetworkTableEntry alignedWithHubEntry;

    private NetworkTableEntry shooterMotorCurrentEntry;
    private NetworkTableEntry hoodMotorCurrentEntry;

    /** Creates a new instance of the Shooter subsystem. */
    public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio,
            Ranger ranger) {

        this.ranger = ranger;
        IDLE_RPM = ranger.getFiringSolution("lob").speed;
        IDLE_ANGLE = ranger.getFiringSolution("lob").angle;

        shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
        shooterMotor1.setInverted(true);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        shooterEncoder = shooterMotor1.getEncoder();
        shooterEncoder.setVelocityConversionFactor(shooterBeltRatio);

        shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        shooterMotor2.follow(shooterMotor1, true);

        hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.setSmartCurrentLimit(15);
        hoodEncoder = hoodMotor.getEncoder();
        hoodLimit = new DigitalInput(hoodLimitDio);

        shooterPidController = shooterMotor1.getPIDController();
        shooterPidController.setP(kShooterP);
        shooterPidController.setI(kShooterI);
        shooterPidController.setD(kShooterD);
        shooterPidController.setIZone(kShooterIz);
        shooterPidController.setFF(kShooterFF);
        shooterPidController.setOutputRange(kShooterMinOutput, kShooterMaxOutput);

        hoodPidController = hoodMotor.getPIDController();
        hoodPidController.setP(kHoodP);
        hoodPidController.setI(kHoodI);
        hoodPidController.setD(kHoodD);
        hoodPidController.setIZone(kHoodIz);
        hoodPidController.setFF(kHoodFF);
        hoodPidController.setOutputRange(kHoodMinOutput, kHoodMaxOutput);

        initTelemetry();
    }

    @Override
    public void periodic() {
        currentSpeed = shooterEncoder.getVelocity();
        speedError = currentSpeed - targetSpeed;
        speedErrorPercent = targetSpeed > 0 ? speedError / targetSpeed : 0;

        currentAngle = encoderPositionToAngle(hoodEncoder.getPosition());

        parkHood();

        // If we are in the process of stopping the shooter motors then slowly ramp the
        // target speed down so the momentum of the flywheels doesn't damage the drive belts.
        if (motorState == MotorState.STOPPING) {
            setSpeed(targetSpeed - 20);
        }

        updateTelemetry();
    }

    // The shooter is not avaialble for use until after it finishes parking the hood
    // this method lets you find you find out if that has completed.
    public boolean available() {
        return !parkingHood;
    }

    public boolean ready() {
        return atTargetSpeed();
    }

    // ---------------------------------------------------------------------------
    // Shooter control methods
    // ---------------------------------------------------------------------------

    public void idle() {
        motorState = MotorState.IDLE;

        setSpeed(IDLE_RPM);
        setAngle(IDLE_ANGLE);

        shooterMotor1.setSmartCurrentLimit(IDLE_CURRENT);
        shooterMotor2.setSmartCurrentLimit(IDLE_CURRENT);
    }

    public void shoot() {
      motorState = MotorState.SHOOTING;

      setSpeed(targetSpeed);

      shooterMotor1.setSmartCurrentLimit(MAX_CURRENT);
      shooterMotor2.setSmartCurrentLimit(MAX_CURRENT);
    }

    public void stop() {
        motorState = MotorState.STOPPING;
    }

    public void setRange(double range) {
        currentRange = range;

        FiringSolution solution = ranger.getFiringSolution(currentRange);
        setSpeed(solution.speed);
        setAngle(solution.angle);
    }

    public void setSpeed(double targetSpeed) {
        this.targetSpeed = targetSpeed;

        if (targetSpeed > MAX_RPM) {
            targetSpeed = MAX_RPM;
        } else if (targetSpeed < 0) {
            targetSpeed = 0;
            motorState = MotorState.STOPPED;
        }

        shooterPidController.setReference(targetSpeed, ControlType.kVelocity);
    }

    public void increaseSpeed() {
        targetSpeed += speedIncrement;

        if (motorState == MotorState.SHOOTING) {
            setSpeed(targetSpeed);
        }
    }

    public void decreaseSpeed() {
        targetSpeed -= speedIncrement;

        if (motorState == MotorState.SHOOTING) {
            setSpeed(targetSpeed);
        }
    }

    private boolean atTargetSpeed() {
        double delta = speedMarginPercent * targetSpeed;
        return currentSpeed >= targetSpeed - delta && currentSpeed <= targetSpeed + delta;
    }

    // ---------------------------------------------------------------------------
    // Hood Control methods
    // ---------------------------------------------------------------------------

    public void setAngle(double angle) {

        targetAngle = angle;

        if (targetAngle > MAX_HOOD_ANGLE) {
            targetAngle = MAX_HOOD_ANGLE;
        } else if (targetAngle < MIN_HOOD_ANGLE) {
            targetAngle = MIN_HOOD_ANGLE;
        }

        hoodPidController.setReference(angleToEncoderPosition(targetAngle), ControlType.kPosition);
    }

    public void raiseHood() {
        setAngle(targetAngle - angleIncrement);
    }

    public void lowerHood() {
        setAngle(targetAngle + angleIncrement);
    }

    private double angleToEncoderPosition(double angle) {
        return (MAX_HOOD_ANGLE - angle) * ROTATIONS_PER_DEGREE;
    }

    private double encoderPositionToAngle(double position) {
        return ((HOOD_UPPER_LIMIT - position) / ROTATIONS_PER_DEGREE) + MIN_HOOD_ANGLE;
    }

    /**
     * Lower the hood until it trips the limit switch and then reset the encoder to
     * establish our zero position.  Called by periodic.
     */
    private void parkHood() {

        // If we are not in the process of parking the hood then just bail out now.
        // parkingHood is initialized to true so we will always fall through this
        // check on the first call.
        if (!parkingHood) {
            return;
        }

        // if the hood is not at the limit then set power to start lowering it and return.
        if (hoodMotor.getOutputCurrent() <= 1) {
            hoodMotor.set(-0.05);
            return;
        }

        // Hood has reached limit, clear parking flag, stop motor and zero encoder.
        parkingHood = false;
        hoodMotor.set(0);
        hoodEncoder.setPosition(-0.25);
        setAngle(MAX_HOOD_ANGLE);
    }

    // ---------------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------------

    private void initTelemetry() {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

        // Shooter Data
        currentSpeedEntry = tab.add("Current Speed", 0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        targetSpeedEntry = tab.add("Target Speed", 0)
                .withPosition(1, 2)
                .withSize(1, 1)
                .getEntry();

        speedErrorEntry = tab.add("Error", 0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        speedErrorPercentEntry = tab.add("Error %", 0)
                .withPosition(3, 2)
                .withSize(1, 1)
                .getEntry();

        shooterEncoderEntry = tab.add("Encoder", 0)
                .withPosition(4, 2)
                .withSize(1, 1)
                .getEntry();

        currentRangeEntry = tab.add("Distance", 0)
                .withPosition(5, 2)
                .withSize(1, 1)
                .getEntry();
        currentXEntry = tab.add("X offset", 0)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();

        readyToShootEntry = tab.add("Ready", false)
                .withPosition(6, 2)
                .withSize(1, 1)
                .getEntry();

        alignedWithHubEntry = tab.add("Aligned", false)
                .withPosition(7, 2)
                .withSize(1, 1)
                .getEntry();

        // Hood Data
        currentAngleEntry = tab.add("Current Angle", 0)
                .withPosition(0, 3)
                .withSize(1, 1)
                .getEntry();

        targetAngleEntry = tab.add("Target Angle", 0)
                .withPosition(1, 3)
                .withSize(1, 1)
                .getEntry();

        hoodEncoderEntry = tab.add("Hood Encoder", 0)
                .withPosition(2, 3)
                .withSize(1, 1)
                .getEntry();

        hoodLimitSwitchEntry = tab.add("Hood Limit", false)
                .withPosition(3, 3)
                .withSize(1, 1)
                .getEntry();

        // current data
        highestCurrentEntry = tab.add("Highest Current", 0)
                .withPosition(7, 0)
                .withSize(1, 1)
                .getEntry();

        shooterMotorCurrentEntry = tab.add("shooter motor current", 0)
                .withPosition(8, 0)
                .withSize(1, 1)
                .getEntry();
        hoodMotorCurrentEntry = tab.add("hood motor current", 0)
                .withPosition(9, 0)
                .withSize(1, 1)
                .getEntry();
    }

    private void updateTelemetry() {
        currentSpeedEntry.setNumber(currentSpeed);
        targetSpeedEntry.setNumber(targetSpeed);
        speedErrorEntry.setNumber(speedError);
        speedErrorPercentEntry.setNumber(speedErrorPercent);
        shooterEncoderEntry.setNumber(shooterEncoder.getPosition());
        currentRangeEntry.setNumber(currentRange);
        currentXEntry.setNumber(currentXOffset);
        readyToShootEntry.forceSetBoolean(ready());
        alignedWithHubEntry.setBoolean(aligned);

        currentAngleEntry.setNumber(currentAngle);
        targetAngleEntry.setNumber(targetAngle);
        hoodEncoderEntry.setNumber(hoodEncoder.getPosition());
        hoodLimitSwitchEntry.forceSetBoolean(hoodLimit.get());

        highestCurrentEntry.setNumber(getHighestCurrent());
        shooterMotorCurrentEntry.setValue(shooterMotor1.getOutputCurrent());
        hoodMotorCurrentEntry.setValue(hoodMotor.getOutputCurrent());
    }

    private double getHighestCurrent() {
      double current = hoodMotor.getOutputCurrent();
      if (current > highestCurrent) {
        highestCurrent = current;
      }
      return highestCurrent;
    }
}
