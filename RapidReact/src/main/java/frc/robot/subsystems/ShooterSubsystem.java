package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.drivers.Limelight;
import frc.robot.utils.FiringSolution;
import frc.robot.utils.SimpleRanger;

public class ShooterSubsystem extends SubsystemBase {
  private SimpleRanger ranger;

  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private SparkMaxPIDController shooterPidController;
  
  private CANSparkMax hoodMotor;
  private SparkMaxPIDController hoodPidController;
  private DigitalInput hoodLimit; // TODO not used

  private Limelight limelight;

  private boolean running = false;

  // TODO fix numbers
  private final int MAX_HOOD_ANGLE = 285;
  private final int MIN_HOOD_ANGLE = 25;
  private final int MAX_RPM = 500;
  private final double ROTATIONS_PER_DEGREE = 5;
  public final double HOOD_SPEED = 0.05;

  private static final int speedIncrement = 100;
  private static final int angleIncrement = 5;

  public int currentSpeed = 0; // rpm
  public int targetSpeed = 0;
  public double currentAngle = 0; // speed
  public double targetAngle = 0; // degrees

  private NetworkTableEntry currentSpeedEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry targetAngleEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  private int count;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio, SimpleRanger ranger, Limelight limelight) {

    this.ranger = ranger;
    this.limelight = limelight;
    
    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodLimit = new DigitalInput(hoodLimitDio);

    // TODO tune pids
    shooterPidController = shooterMotor1.getPIDController();
    shooterPidController.getP(0);
    shooterPidController.getI(0);
    shooterPidController.getD(0);
    hoodPidController = hoodMotor.getPIDController();
    hoodPidController.getP(0);
    hoodPidController.getI(0);
    hoodPidController.getD(0);

    // The motors in the shooter run in opposition to each other by default, invert one of them
    shooterMotor2.follow(shooterMotor1, true);

    initTelemetry();
  }

  public void start() {
    running = true;
    System.out.print(getHoodLimitswitch().get());
    setSpeed(targetSpeed); // TODO this is setting speed to 0, which does nothing
  }

  public void stop() {
    running = false;
    setSpeed(0);
  }

  public void raiseAngle(String mode) {
    hoodMotor.set(HOOD_SPEED);
    // shooterPidController.setReference(HOOD_SPEED, ControlType.kVoltage);
  }

  public void raiseAngle() {
    targetAngle += angleIncrement;
    if (running) {
      goToAngle(currentAngle);
    }
  }

  public void lowerAngle(String mode) {
    //count += hoodMotor.getEncoder().setPositionConversionFactor(factor).getPosition();
    hoodMotor.set(-HOOD_SPEED);
    System.out.println(count);
  }

  public void lowerAngle() {
    targetAngle -= angleIncrement;
    if (running) {
      goToAngle(currentAngle);
    }
  }

  public Boolean atMaxAngle(){
    if(currentAngle >= MAX_HOOD_ANGLE) return true;
    return false;
  }

  public void increaseSpeed() {
      targetSpeed += speedIncrement;
      if (targetSpeed > MAX_RPM) {
        targetSpeed = MAX_RPM;
      }

      if (running) {
        setSpeed(targetSpeed);
      }
  }

  public void decreaseSpeed() {
    targetSpeed -= speedIncrement;
    if (targetSpeed < 0) {
      targetSpeed = 0;
    }

    if (running) {
      setSpeed(targetSpeed);
    }
  }

  public DigitalInput getHoodLimitswitch(){
    return hoodLimit;
  }

  public double getAngle(){
    currentAngle = hoodMotor.getEncoder().getPosition() / ROTATIONS_PER_DEGREE;
    return currentAngle;
  }

  public double getMinAngle(){
    return MIN_HOOD_ANGLE;
  }

  public double getMaxAngle(){
    return MAX_HOOD_ANGLE;
  }

  public void resetHoodEncoder(){
    hoodMotor.getEncoder().setPosition(MIN_HOOD_ANGLE);
  }

  public void setSpeed(int targetSpeed){
    this.targetSpeed = targetSpeed;
    shooterPidController.setReference(targetSpeed, ControlType.kVelocity);
  }

  public void setAngle(double angle){
    this.currentAngle = angle;
  }

  public void goToAngle(double angle){
    this.currentAngle = angle;

    if(targetAngle > MAX_HOOD_ANGLE){
      targetAngle = MAX_HOOD_ANGLE;
    }
    if(targetAngle < MIN_HOOD_ANGLE){
      targetAngle = MIN_HOOD_ANGLE;
    }

    hoodPidController.setReference(targetAngle * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

    currentSpeedEntry = tab.add("Current Speed", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    targetSpeedEntry = tab.add("Target Speed", 0)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

    currentAngleEntry = tab.add("Current Angle", 0)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();
        
    targetAngleEntry = tab.add("Target Angle", 0)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

    hoodLimitSwitchEntry = tab.add("Hood Limit", 0)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
  }

  private void updateTelemetry() {
    currentSpeedEntry.setNumber(currentSpeed);
    targetSpeedEntry.setNumber(targetSpeed);
    currentAngleEntry.setNumber(currentAngle);
    targetAngleEntry.setNumber(targetAngle);
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
  }

  @Override
  public void periodic() {
    currentAngle = hoodMotor.getEncoder().getPosition();
    currentSpeed = (int) Math.round(shooterMotor2.getEncoder().getVelocity());
    
    double range = limelight.getDistance();
    FiringSolution solution = ranger.getFiringSolution(range);
    setSpeed(solution.speed);
    goToAngle(solution.angle);

    updateTelemetry();
  }
}
