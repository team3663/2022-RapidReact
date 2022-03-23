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

public class ShooterSubsystem extends SubsystemBase {

  private enum MotorState {
    STOPPED,
    RUNNING,
    STOPPING
  }

  // Subsystem Constants
  private static final double MAX_RPM = 6000;
  private static final int MAX_CURRENT = 13;
  private static final double shooterBeltRatio = 0.66;
  private static final double speedIncrement = 50;

  // Hood related constants
  private static final double MAX_HOOD_ANGLE = 85;
  private static final double MIN_HOOD_ANGLE = 67;
  private static final double HOOD_LOWER_LIMIT = 0;
  private static final double HOOD_UPPER_LIMIT = 13.5;
  private static final double ROTATIONS_PER_DEGREE = (HOOD_UPPER_LIMIT - HOOD_LOWER_LIMIT)
      / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE);
  private static final double angleIncrement = 1;

  // Shooter PID coefficients constants
  private static final double kShooterP = 0.000153;
  private static final double kShooterI = 0.000000;
  private static final double kShooterD = 0.000003;
  private static final double kShooterIz = 0.000000;
  private static final double kShooterFF = 0.00029625;
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

  // Live PID coefficients
  private double currentP = kShooterP;
  private double currentI = kShooterI;
  private double currentD = kShooterD;
  private double currentIz = kShooterIz;
  private double currentFF = kShooterFF;
  private double currentMaxOutput = kShooterMaxOutput;
  private double currentMinOutput = kShooterMinOutput;

  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private RelativeEncoder shooterEncoder;
  private SparkMaxPIDController shooterPidController;

  private CANSparkMax hoodMotor;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPidController;
  private DigitalInput hoodLimit;

  private MotorState motorState = MotorState.STOPPED;
  public double currentSpeed = 0;
  public double targetSpeed = 0;

  private boolean parkingHood = false;
  public double currentAngle = 0;
  public double targetAngle = 0;

  // Shuffleboard constants
  private final int kShooterRow = 0;
  private final int kHoodRow = 1;
  private final int kConstantsRow = 2;
  private final int kCurrentRow = 3;

  private NetworkTableEntry currentSpeedEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry shooterEncoderEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry targetAngleEntry;
  private NetworkTableEntry hoodEncoderEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  // Network table entries to display default PID coefficients for reference
  private NetworkTableEntry kPEntry;
  private NetworkTableEntry kIEntry;
  private NetworkTableEntry kDEntry;
  private NetworkTableEntry kIzEntry;
  private NetworkTableEntry kFFEntry;
  private NetworkTableEntry kMaxOutputEntry;
  private NetworkTableEntry kMinOutputEntry;

  // Network table entries for active PID coefficient
  private NetworkTableEntry curPEntry;
  private NetworkTableEntry curIEntry;
  private NetworkTableEntry curDEntry;
  private NetworkTableEntry curIzEntry;
  private NetworkTableEntry curFFEntry;
  private NetworkTableEntry curMaxOutputEntry;
  private NetworkTableEntry curMinOutputEntry;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio) {

    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor1.setInverted(true);
    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor1.setSmartCurrentLimit(MAX_CURRENT);
    shooterEncoder = shooterMotor1.getEncoder();
    shooterEncoder.setVelocityConversionFactor(shooterBeltRatio);

    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    shooterMotor2.follow(shooterMotor1, true);
    shooterMotor2.setSmartCurrentLimit(MAX_CURRENT);

    shooterPidController = shooterMotor1.getPIDController();

    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodEncoder = hoodMotor.getEncoder();
    hoodLimit = new DigitalInput(hoodLimitDio);

    hoodPidController = hoodMotor.getPIDController();
    hoodPidController.setP(kHoodP);
    hoodPidController.setI(kHoodI);
    hoodPidController.setD(kHoodD);
    hoodPidController.setIZone(kHoodIz);
    hoodPidController.setFF(kHoodFF);
    hoodPidController.setOutputRange(kHoodMinOutput, kHoodMaxOutput);

    parkHood();
    initTelemetry();

    updatePIDController();
  }

  @Override
  public void periodic() {
    currentSpeed = shooterEncoder.getVelocity();

    currentAngle = encoderPositionToAngle(hoodEncoder.getPosition());

    if (parkingHood) {
      parkHood();
    }

    // If we are in the process of stopping the shooter motor then slowly ramp the target
    // speed down so the momentum of the flywheels don't damage the belts.
    if (motorState == MotorState.STOPPING)
    {
      setSpeed( targetSpeed - 20);
    }

    updateTelemetry();
  }

  // ---------------------------------------------------------------------------
  // Shooter control methods
  // ---------------------------------------------------------------------------

  public void start() {
    motorState = MotorState.RUNNING;
    setSpeed(targetSpeed);
  }

  public void stop() {
    motorState = MotorState.STOPPING;
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
    System.out.println("Increase Shooter speed");
    targetSpeed += speedIncrement;

    if (motorState == MotorState.RUNNING) {
      setSpeed(targetSpeed);
    }
  }

  public void decreaseSpeed() {
    System.out.println("Decrease Shooter speed");
    targetSpeed -= speedIncrement;

    if (motorState == MotorState.RUNNING) {
      setSpeed(targetSpeed);
    }
  }

  /**
   * Update the cached coefficients values from the network tables and write them
   * out to the motor controller.
   */
  public void updatePIDCoefficients() {
    currentP = curPEntry.getDouble(kShooterP);
    currentI = curIEntry.getDouble(kShooterI);
    currentD = curDEntry.getDouble(kShooterD);
    currentIz = curIzEntry.getDouble(kShooterIz);
    currentFF = curFFEntry.getDouble(kShooterFF);
    currentMaxOutput = curMaxOutputEntry.getDouble(kShooterMaxOutput);
    currentMinOutput = curMinOutputEntry.getDouble(kShooterMinOutput);

    updatePIDController();
  }

  /**
   * Update the PID coefficents in the motor controller from the cached values
   */
  public void updatePIDController() {
    shooterPidController.setP(currentP);
    shooterPidController.setI(currentI);
    shooterPidController.setD(currentD);
    shooterPidController.setIZone(currentIz);
    shooterPidController.setFF(currentFF);
    shooterPidController.setOutputRange(currentMinOutput, currentMaxOutput);

    updateCurrentValues();
  }

  public void dumpPIDCoefficients() {

    System.out.println("--------------------------------------------------------");
    System.out.printf("private static final double kShooterP = %f;\n", currentP);
    System.out.printf("private static final double kShooterI = %f;\n", currentI);
    System.out.printf("private static final double kShooterD = %f;\n", currentD);
    System.out.printf("private static final double kShooterIz = %f;\n", currentIz);
    System.out.printf("private static final double kShooterFF = %f;\n", currentFF);
    System.out.printf("private static final double kShooterMaxOutput = %f;\n", currentMaxOutput);
    System.out.printf("private static final double kShooterMinOutput = %f;\n", currentMinOutput);
    System.out.println("--------------------------------------------------------");
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

  private double angleToEncoderPosition(double angle){
    return (MAX_HOOD_ANGLE - angle) * ROTATIONS_PER_DEGREE;
  }

  private double encoderPositionToAngle(double position){
    return ((HOOD_UPPER_LIMIT - position) / ROTATIONS_PER_DEGREE) + MIN_HOOD_ANGLE;
  }

  /**
   * Lower the hood until it trips the limit switch and then reset the encoder to
   * establish our zero position.
   */
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
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter Tuner");

    // Shooter data values
    currentSpeedEntry = tab.add("Current Speed", 0)
        .withPosition(0, kShooterRow)
        .withSize(1, 1)
        .getEntry();

    targetSpeedEntry = tab.add("Target Speed", 0)
        .withPosition(1, kShooterRow)
        .withSize(1, 1)
        .getEntry();

    shooterEncoderEntry = tab.add("Shooter Encoder", 0)
        .withPosition(2, kShooterRow)
        .withSize(1, 1)
        .getEntry();

    // Hood data values
    currentAngleEntry = tab.add("Current Angle", 0)
        .withPosition(0, kHoodRow)
        .withSize(1, 1)
        .getEntry();

    targetAngleEntry = tab.add("Target Angle", 0)
        .withPosition(1, kHoodRow)
        .withSize(1, 1)
        .getEntry();

    hoodEncoderEntry = tab.add("Hood Encoder", 0)
        .withPosition(2, kHoodRow)
        .withSize(1, 1)
        .getEntry();

    hoodLimitSwitchEntry = tab.add("Hood Limit", false)
        .withPosition(3, kHoodRow)
        .withSize(1, 1)
        .getEntry();

    // Constant value entries
    kPEntry = tab.add("kP", 0)
        .withPosition(0, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kIEntry = tab.add("kI", 0)
        .withPosition(1, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kDEntry = tab.add("kD", 0)
        .withPosition(2, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kIzEntry = tab.add("kIz", 0)
        .withPosition(3, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kFFEntry = tab.add("kF", 0)
        .withPosition(4, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kMaxOutputEntry = tab.add("kMaxOutput", 0)
        .withPosition(5, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    kMinOutputEntry = tab.add("kMinOutput", 0)
        .withPosition(6, kConstantsRow)
        .withSize(1, 1)
        .getEntry();

    // Live value entries
    curPEntry = tab.add("Current P", 0)
        .withPosition(0, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curIEntry = tab.add("Current I", 0)
        .withPosition(1, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curDEntry = tab.add("Current D", 0)
        .withPosition(2, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curIzEntry = tab.add("Current Iz", 0)
        .withPosition(3, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curFFEntry = tab.add("Current FF", 0)
        .withPosition(4, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curMaxOutputEntry = tab.add("Current MaxOutput", 0)
        .withPosition(5, kCurrentRow)
        .withSize(1, 1)
        .getEntry();

    curMinOutputEntry = tab.add("Current MinOutput", 0)
        .withPosition(6, kCurrentRow)
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
    hoodLimitSwitchEntry.forceSetBoolean(hoodLimit.get());
   
    // Update the contant values in the network table even though they never change.
    kPEntry.setDouble(kShooterP);
    kIEntry.setDouble(kShooterI);
    kDEntry.setDouble(kShooterD);
    kIzEntry.setDouble(kShooterIz);
    kFFEntry.setDouble(kShooterFF);
    kMaxOutputEntry.setDouble(kShooterMaxOutput);
    kMinOutputEntry.setDouble(kShooterMinOutput);
    }

  private void updateCurrentValues() {
    curPEntry.setDouble(currentP);
    curIEntry.setDouble(currentI);
    curDEntry.setDouble(currentD);
    curIzEntry.setDouble(currentIz);
    curFFEntry.setDouble(currentFF);
    curMaxOutputEntry.setDouble(currentMaxOutput);
    curMinOutputEntry.setDouble(currentMinOutput);    
  }
}
