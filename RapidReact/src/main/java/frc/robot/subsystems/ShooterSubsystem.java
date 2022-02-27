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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.utils.FiringSolution;
import frc.robot.utils.Ranger;

public class ShooterSubsystem extends SubsystemBase {

  // Subsystem Constants
  private static final double MAX_RPM = 2000.0;
  private static final int MAX_HOOD_ANGLE = 285;
  private static final int MIN_HOOD_ANGLE = 25;
  private static final double ROTATIONS_PER_DEGREE = 5;
  private static final double speedIncrement = 200.0;
  private static final int angleIncrement = 5;

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
  public double currentSpeed = 0.0;
  public double targetSpeed = 500.0;

  public double currentAngle = 0;
  public double targetAngle = 0;

  private double currentRange = 10.0;

  private NetworkTableEntry currentSpeedEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry targetAngleEntry;
  private NetworkTableEntry hoodEncoderEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio,
      Ranger ranger) {

    this.ranger = ranger;

    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    shooterEncoder = shooterMotor1.getEncoder();
    // The motors in the shooter run in opposition to each other by default, invert
    // one of them
    shooterMotor2.follow(shooterMotor1, true);

    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodEncoder = hoodMotor.getEncoder();
    hoodLimit = new DigitalInput(hoodLimitDio);

    shooterPidController = shooterMotor1.getPIDController();
    shooterPidController.setP(0);
    shooterPidController.setI(0);
    shooterPidController.getD(0);

    hoodPidController = hoodMotor.getPIDController();
    hoodPidController.setP(0);
    hoodPidController.setI(0);
    hoodPidController.setD(0);

    initTelemetry();
  }

  @Override
  public void periodic() {
    currentSpeed = (int) Math.round(shooterEncoder.getVelocity());
    currentAngle = hoodMotor.getEncoder().getPosition();

    shooterPidController.setReference(targetSpeed, ControlType.kVelocity);

    updateTelemetry();
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
      targetSpeed = 0.0;
    }
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
    }

    if (targetAngle < MIN_HOOD_ANGLE) {
      targetAngle = MIN_HOOD_ANGLE;
    }

    hoodPidController.setReference(targetAngle * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }

  public void raiseAngle() {
    targetAngle += angleIncrement;
  }

  public void lowerAngle() {
    targetAngle -= angleIncrement;
  }

  public void resetHoodEncoder() {
    hoodMotor.getEncoder().setPosition(MIN_HOOD_ANGLE);
  }


  // ---------------------------------------------------------------------------
  //  Telemetry
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

    currentAngleEntry = tab.add("Current Angle", 0)
        .withPosition(2, 2)
        .withSize(1, 1)
        .getEntry();

    targetAngleEntry = tab.add("Target Angle", 0)
        .withPosition(3, 2)
        .withSize(1, 1)
        .getEntry();

    hoodEncoderEntry = tab.add("Hood Encoder", 0)
        .withPosition(4, 2)
        .withSize(1, 1)
        .getEntry();

    hoodLimitSwitchEntry = tab.add("Hood Limit", 0)
        .withPosition(5, 2)
        .withSize(1, 1)
        .getEntry();
  }

  private void updateTelemetry() {
    currentSpeedEntry.setNumber(currentSpeed);
    targetSpeedEntry.setNumber(targetSpeed);
    currentAngleEntry.setNumber(currentAngle);
    targetAngleEntry.setNumber(targetAngle);
    hoodEncoderEntry.setNumber(hoodEncoder.getPosition());
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
  }
}
