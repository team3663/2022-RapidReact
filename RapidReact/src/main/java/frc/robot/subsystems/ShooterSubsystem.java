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
import frc.robot.utils.Ranger;

public class ShooterSubsystem extends SubsystemBase {
  private Ranger ranger;
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private SparkMaxPIDController shooterPidController;
  private CANSparkMax hoodMotor;
  private DigitalInput hoodLimit;
  private SparkMaxPIDController hoodPidController;

  private boolean running = false;

  private Limelight limelight;

  // TODO fix numbers
  private final int MAX_HOOD_ANGLE = 80;
  private final int MIN_HOOD_ANGLE = 25;
  private final int MAX_RPM = 500;
  private final double ROTATIONS_PER_DEGREE = 5;

  private static final int rpmIncrement = 100;
  private static  final int hoodAngleIncrement = 5;
  public int rpm = 0;
  public int hoodAngle = 0;

  private NetworkTableEntry shooterRPMEntry;
  private NetworkTableEntry hoodAngleEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitDio, Ranger ranger, Limelight limelight) {

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

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

    shooterRPMEntry = tab.add("Shooter RPM", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    hoodAngleEntry = tab.add("Hood Angle", 0)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

    hoodLimitSwitchEntry = tab.add("Hood Limit", 0)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
  }

  private void updateTelemetry() {
    shooterRPMEntry.setNumber(shooterMotor2.getEncoder().getVelocity());
    hoodAngleEntry.setNumber(hoodMotor.getEncoder().getPosition());
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
  }


  public void start() {
    running = true;
    setRPM(rpm);
  }

  public void stop() {
    running = false;
    setRPM(rpm);
  }

  public void raiseHood() {
    hoodAngle += hoodAngleIncrement;
    if (running) {
      setHoodAngle(hoodAngle);
    }
  }

  public void lowerHood() {
    hoodAngle -= hoodAngleIncrement;
    if (running) {
      setHoodAngle(hoodAngle);
    }
  }

  public void increaseRPM() {
      rpm += rpmIncrement;
      if (rpm > MAX_RPM) {
        rpm = MAX_RPM;
      }

      if (running) {
        setRPM(rpm);
      }
  }

  public void decreaseRPM() {
    rpm -= rpmIncrement;
    if (rpm < 0) {
      rpm = 0;
    }

    if (running) {
      setRPM(rpm);
    }
  }


  public void resetHoodEncoder(){
    hoodMotor.getEncoder().setPosition(MIN_HOOD_ANGLE);
  }

  public void setRPM(int rpm){
    shooterPidController.setReference(rpm, ControlType.kVelocity);
  }

  public void setHoodAngle(double degree){
    if(degree > MAX_HOOD_ANGLE){
      degree = MAX_HOOD_ANGLE;
    }
    if(degree < MIN_HOOD_ANGLE){
      degree = MIN_HOOD_ANGLE;
    }
    hoodPidController.setReference(degree * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    
    double range = limelight.getDistance();
    FiringSolution solution = ranger.getFiringSolution(range);

    setRPM(solution.rpm);
    setHoodAngle(solution.hoodAngle);

    updateTelemetry();
  }
}
