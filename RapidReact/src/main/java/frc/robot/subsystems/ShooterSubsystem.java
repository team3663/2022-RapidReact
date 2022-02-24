package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.drivers.Limelight;
import frc.robot.utils.FiringSolution;
import frc.robot.utils.SimpleRanger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private SimpleRanger ranger;

  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private SparkMaxPIDController shooterPidController;
  
  private CANSparkMax hoodMotor;
  private DigitalInput hoodLimit;
  private SparkMaxPIDController hoodPidController;

  private Limelight limelight;

  private boolean running = false;

  // TODO fix numbers
  private final int MAX_HOOD_ANGLE = 80;
  private final int MIN_HOOD_ANGLE = 25;
  private final int MAX_RPM = 500;
  private final double ROTATIONS_PER_DEGREE = 5;

  private static final int rpmIncrement = 100;
  private static  final int hoodAngleIncrement = 5;

  public int speed = 0; // rpm
  public double angle = 0; // degrees

  private NetworkTableEntry currentSpeedEntry;
  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry targetAngleEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio, SimpleRanger ranger) {

    this.ranger = ranger;
    
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

    limelight = RobotContainer.getLimelight();

    // The motors in the shooter run in opposition to each other by default, invert one of them
    shooterMotor2.follow(shooterMotor1, true);

    initTelemetry();
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
    currentSpeedEntry.setNumber(shooterMotor2.getEncoder().getVelocity());
    targetSpeedEntry.setNumber(speed);
    currentAngleEntry.setNumber(hoodMotor.getEncoder().getPosition());
    targetAngleEntry.setNumber(angle);
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
  }


  public void start() {
    running = true;
    setSpeed(speed);
  }

  public void stop() {
    running = false;
    setSpeed(0);
  }

  public void raiseHood() {
    angle += hoodAngleIncrement;
    if (running) {
      setAngle(angle);
    }
  }

  public void lowerHood() {
    angle -= hoodAngleIncrement;
    if (running) {
      setAngle(angle);
    }
  }

  public void increaseRPM() {
      speed += rpmIncrement;
      if (speed > MAX_RPM) {
        speed = MAX_RPM;
      }

      if (running) {
        setSpeed(speed);
      }
  }

  public void decreaseRPM() {
    speed -= rpmIncrement;
    if (speed < 0) {
      speed = 0;
    }

    if (running) {
      setSpeed(speed);
    }
  }


  public void resetHoodEncoder(){
    hoodMotor.getEncoder().setPosition(MIN_HOOD_ANGLE);
  }

  public void setSpeed(int rpm){
    shooterPidController.setReference(rpm, ControlType.kVelocity);
  }

  public void setAngle(double angle){
    this.angle = angle;

    if(angle > MAX_HOOD_ANGLE){
      angle = MAX_HOOD_ANGLE;
    }
    if(angle < MIN_HOOD_ANGLE){
      angle = MIN_HOOD_ANGLE;
    }
    hoodPidController.setReference(angle * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    
    double range = limelight.getDistance();
    FiringSolution solution = ranger.getFiringSolution(range);
    setSpeed(solution.speed);
    setAngle(solution.angle);

    updateTelemetry();
  }
}
