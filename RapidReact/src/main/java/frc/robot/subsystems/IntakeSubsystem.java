package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    RETRACTED,
    EXTENDING,
    EXTENDED,
    RETRACTING
  }

  private CANSparkMax intakeMotor;
  private final double POWER = 0.4;
  private final DoubleSolenoid boomIntakeSolenoid;
  private final DoubleSolenoid armIntakeSolenoid;
  private boolean boomIsOut;
  private boolean armIsOut;
  private final int MOTOR_CURRENT_LIMIT = 25;
  private Timer timer;
  private boolean timerStarted = false;

  // network table entries for Shuffleboard
  private NetworkTableEntry boomIsOutEntry;
  private NetworkTableEntry armIsOutEntry;
  private NetworkTableEntry intakeMotorSpeedEntry;

  /** Creates a new instance of the Shooter subsystem. */
  /**
   * Boom is the upper-arm of the intake, Arm is the fore-arm of the intake
   * 
   * @param motor1CANId
   * @param boomRetractSolenoidChan
   * @param boomExtendSolenoidChan
   * @param armRetractSolenoidChan
   * @param armExtendSolenoidChan
   */
  public IntakeSubsystem(int motor1CANId,
      int boomRetractSolenoidChan, int boomExtendSolenoidChan,
      int armRetractSolenoidChan, int armExtendSolenoidChan) {

    intakeMotor = new CANSparkMax(motor1CANId, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);

    boomIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, boomExtendSolenoidChan,
        boomRetractSolenoidChan);
    armIntakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, armExtendSolenoidChan, armRetractSolenoidChan);

    timer = new Timer();
    initTelemetry();
  }

  @Override
  public void periodic() {

    updateTelemetry();
  }

  /**
   * Boom is the upper-arm part of the intake mechanism
   * boomExtend() lowers the intake mechanism
   */
  public void extendBoom() {
    boomIntakeSolenoid.set(Value.kForward);
    boomIsOut = true;
  }

  /**
   * Boom is the upper-arm part of the intake mechanism
   * boomExtend() lowers the intake mechanism
   */
  public void retractBoom() {
    boomIntakeSolenoid.set(Value.kReverse);
    boomIsOut = false;
  }

  /**
   * Arm is the fore-arm part of the intake mechanism
   * armExtend() lowers the intake mechanism
   */
  public void extendArm() {
    armIntakeSolenoid.set(Value.kForward);
    armIsOut = true;
  }

  /**
   * Arm is the fore-arm part of the intake mechanism
   * armExtend() lowers the intake mechanism
   */
  public void retractArm() {
    armIntakeSolenoid.set(Value.kReverse);
    armIsOut = false;
  }

  public void spinBallIn() {
    if(boomIsOut && armIsOut)
    intakeMotor.set(POWER);
  }

  public void spinBallOut() {
    if(boomIsOut && armIsOut)
      intakeMotor.set(-POWER);
  }

  public void intakeOut() {
    if(timerStarted == false){
      System.out.print("Timer is Started -----------------");
      timer.start();
      timerStarted = true;
    }
    if(!armIsOut) {
      extendArm();
      System.out.println("ARM JUST GOT SENT OUT -----------------");
    }
    if(armIsOut){
      System.out.println("ARM IS STILL OUT ------------------------");
      if(timer.get() >= 1) {
        System.out.println("TIMER.GET WORKS ------------------------------");
        extendBoom();
        if(timer.get() >= 2) {
          spinBallIn();
          timerStarted = true;
        }
      }
    }
  }

  public void intakeIn() {
    timer.start();
    stopMotor();
    if(boomIsOut) {
      retractBoom();
      timer.reset();
      if(armIsOut && timer.get() >= 1000) {
        retractArm();
      }
    }
  }

  public void stopMotor() {
    intakeMotor.set(0);
  }

  public double getPower() {
    return intakeMotor.get();
  }

  public boolean getIsBoomOut() {
    return boomIsOut;
  }

  public boolean getIsArmOut() {
    return armIsOut;
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    intakeMotorSpeedEntry = tab.add("Motor Speed", 0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();

    boomIsOutEntry = tab.add("Boom Extended", false)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();

    armIsOutEntry = tab.add("Arm Extended", false)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();
  }

  private void updateTelemetry() {
    intakeMotorSpeedEntry.setValue(intakeMotor.get());
    boomIsOutEntry.setBoolean(boomIsOut);
    armIsOutEntry.setBoolean(armIsOut);
  }

}