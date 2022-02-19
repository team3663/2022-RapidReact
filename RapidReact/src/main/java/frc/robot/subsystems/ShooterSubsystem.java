package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.drivers.Limelight;
import frc.robot.utils.FiringSolution;
import frc.robot.utils.Ranger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private Ranger ranger;
  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;
  private SparkMaxPIDController shooterPIDController;
  private CANSparkMax hoodMotor;
  private DigitalInput hoodLimit;
  private SparkMaxPIDController hoodPidController;
  private MotorControllerGroup shooterMotorGroup ;

  private boolean running = false;

  private Limelight limelight;

  private final int MAX_HOOD_ANGLE = 80; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY
  private final int MIN_HOOD_ANGLE = 25; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY

  private final double ROTATIONS_PER_DEGREE = 5; //THESE NUMBERS ARE ALL COMPLETELY IMAGINARY

  static final double powerIncrement = 0.05; 
  public double power = 0.0;

  private NetworkTableEntry shooterRPMEntry;
  private NetworkTableEntry hoodAngleEntry;
  private NetworkTableEntry hoodLimitSwitchEntry;

  /** Creates a new instance of the Shooter subsystem. */
  public ShooterSubsystem(int shooterMotor1CANID, int shooterMotor2CANID, int hoodMotorCANID, int hoodLimitCANID, Ranger ranger) {

    this.ranger = ranger;
    
    shooterMotor1 = new CANSparkMax(shooterMotor1CANID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(shooterMotor2CANID, MotorType.kBrushless);
    hoodMotor = new CANSparkMax(hoodMotorCANID, MotorType.kBrushless);
    hoodLimit = new DigitalInput(hoodLimitCANID);

    shooterMotorGroup = new MotorControllerGroup(shooterMotor1, shooterMotor2);

    hoodLimit = new DigitalInput(hoodLimitCANID);

    limelight = RobotContainer.getVision();

    // The motors in the shooter run in opposition to each other by default
    // invert one of them to fix this and initialize power to zero.
    shooterMotor1.setInverted(true);
    shooterMotorGroup.set(0);

    initTelemetry();
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter"); // Data is grouped with shooter and intake.

    shooterRPMEntry = tab.add("Feeder RPM", 0)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    hoodAngleEntry = tab.add("Entry Sensor", 0)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

    hoodLimitSwitchEntry = tab.add("Exit Sensor", 0)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();
  }
  
  @Override
  public void periodic() {
    
    double range = limelight.getDistance();
    FiringSolution solution = ranger.getFiringSolution(range);

    setRPM(solution.rpm);

    updateTelemetry();
  }

  private void updateTelemetry() {
    shooterRPMEntry.setNumber(shooterMotor2.getEncoder().getVelocity());
    hoodAngleEntry.setNumber(hoodMotor.getEncoder().getPosition());
    hoodLimitSwitchEntry.setBoolean(hoodLimit.get());
  }


  public void start() {
    running = true;
    shooterMotorGroup.set(power);
  }

  public void stop() {
    running = false;
    shooterMotorGroup.set(0.0);
  }

  public void raiseHood() {
    hoodMotor.set(0.05);
  }

  public void lowerHood() {
    hoodMotor.set(-0.05);
  }

  public void stopHood(){
    hoodMotor.set(0);
  }

  public void increasePower() {
      power += powerIncrement;
      if (power > 1.0) {
        power = 1.0;
      }

      if ( running ) {
        shooterMotorGroup.set(power);
      }
  }

  public void decreasePower() {
    power -= powerIncrement;
    if (power < 0.0) {
      power = 0.0;
    }

    if (running) {
      shooterMotorGroup.set(power);
    }
  }


  public void resetHoodEncoder(){
    hoodMotor.getEncoder().setPosition(MIN_HOOD_ANGLE);
  }

  public void setRPM(int rpm){
    shooterPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setHoodAngle(int degree){
    if(degree > MAX_HOOD_ANGLE){
      degree = MAX_HOOD_ANGLE;
    }
    if(degree < MIN_HOOD_ANGLE){
      degree = MIN_HOOD_ANGLE;
    }
    resetHoodEncoder();
    degree = MIN_HOOD_ANGLE;
    hoodPidController.setReference(degree * ROTATIONS_PER_DEGREE, ControlType.kPosition);
  }
}
