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

  // Subsystem Constants
  private static final double MAX_RPM = 6000;
  private static final double shooterBeltRatio = 0.66;

  private static final double MAX_HOOD_ANGLE = 85;
  private static final double MIN_HOOD_ANGLE = 67;
  private static final double HOOD_LOWER_LIMIT = 0;
  private static final double HOOD_UPPER_LIMIT = 13.5;
  private static final double ROTATIONS_PER_DEGREE = (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT)
      / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
  private static final double speedIncrement = 200;
  private static final double angleIncrement = 1;

  // Shooter PID coefficients
  private static final double kShooterP = 0.0012;
  private static final double kShooterI = 0;
  private static final double kShooterD = 0;
  private static final double kShooterIz = 0;
  private static final double kShooterFF = 0.000015;
  private static final double kShooterMaxOutput = 1;
  private static final double kShooterMinOutput = 0;

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

  private boolean running = false;
  public double currentSpeed = 0;
  public double targetSpeed = 0;

  private boolean parkingHood = false;
  public double currentAngle = 0;
  public double targetAngle = 0;

  private double currentRange = 10.0;

  private NetworkTableEntry currentSpeedEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry shooterEncoderEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry targetAngleEntry;
  private NetworkTableEntry hoodEncoderEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;
  private NetworkTableEntry readyToShootEntry;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio,
      Ranger ranger) {

    this.ranger = ranger;

    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor1.setInverted(true);
    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterEncoder = shooterMotor1.getEncoder();
    shooterEncoder.setPositionConversionFactor(shooterBeltRatio);

    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    shooterMotor2.follow(shooterMotor1, true);

    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodMotor.setIdleMode(IdleMode.kBrake);
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
   
    parkHood();
    initTelemetry();
  }

  @Override
  public void periodic() {
    currentSpeed = (int) Math.round(shooterEncoder.getVelocity());
    currentAngle = hoodMotor.getEncoder().getPosition();

    if (parkingHood) {
      parkHood();
    }

    updateTelemetry();
  }

  public boolean readyToShoot(){
    if(currentAngle == targetAngle && currentSpeed == targetSpeed){
      return true;
    }
    return false;
  }

  // ---------------------------------------------------------------------------
  // Shooter control methods
  // ---------------------------------------------------------------------------

  public void start() {
    System.out.println("Starting shooter");
    running = true;
    setSpeed(targetSpeed);
  }

  public void stop() {
    System.out.println("Stopping shooter");
    running = false;
    setSpeed(0);
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
    }

    shooterPidController.setReference(targetSpeed, ControlType.kVelocity);
  }

  public void increaseSpeed() {
    System.out.println("Increase Shooter speed");
    targetSpeed += speedIncrement;

    if (running) {
      setSpeed(targetSpeed);
    }
  }

  public void decreaseSpeed() {
    System.out.println("Decrease Shooter speed");
    targetSpeed -= speedIncrement;

    if (running) {
      setSpeed(targetSpeed);
    }
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

    hoodPidController.setReference((MAX_HOOD_ANGLE - targetAngle) * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }

  public void raiseHood() {
    setAngle(targetAngle - angleIncrement);
  }

  public void lowerHood() {
    setAngle(targetAngle + angleIncrement);
  }

  private void parkHood() {

    if (!hoodLimit.get()) {
      parkingHood = true;
      hoodMotor.set(-0.05);
      return;
    }

    parkingHood = false;
    hoodMotor.set(0);
    hoodEncoder.setPosition(0);
    targetAngle = MAX_HOOD_ANGLE;
  }

  // ---------------------------------------------------------------------------
  // Telemetry
  // ---------------------------------------------------------------------------

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

    currentSpeedEntry = tab.add("Current Speed", 0)
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();

    targetSpeedEntry = tab.add("Target Speed", 0)
        .withPosition(1, 2)
        .withSize(1, 1)
        .getEntry();

    shooterEncoderEntry = tab.add("Shooter Encoder", 0)
        .withPosition(2, 2)
        .withSize(1, 1)
        .getEntry();

    currentAngleEntry = tab.add("Current Angle", 0)
        .withPosition(3, 2)
        .withSize(1, 1)
        .getEntry();

    targetAngleEntry = tab.add("Target Angle", 0)
        .withPosition(4, 2)
        .withSize(1, 1)
        .getEntry();

    hoodEncoderEntry = tab.add("Hood Encoder", 0)
        .withPosition(5, 2)
        .withSize(1, 1)
        .getEntry();

    hoodLimitSwitchEntry = tab.add("Hood Limit", false)
        .withPosition(6, 2)
        .withSize(1, 1)
        .getEntry();

    readyToShootEntry = tab.add("Ready to Shoot", false)
        .withPosition(7, 2)
        .withSize(1, 1)
        .getEntry();
  }

  private void updateTelemetry() {
    currentSpeedEntry.setNumber(currentSpeed);
    targetSpeedEntry.setNumber(targetSpeed);
    shooterEncoderEntry.setNumber(shooterEncoder.getPosition());

    currentAngleEntry.setNumber(currentAngle);
    targetAngleEntry.setNumber(targetAngle);
    hoodEncoderEntry.setNumber(hoodEncoder.getPosition());
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
    readyToShootEntry.setBoolean(readyToShoot());
  }
}
