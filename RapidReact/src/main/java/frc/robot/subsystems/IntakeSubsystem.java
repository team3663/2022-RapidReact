package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

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
  }

  /**
   * Arm is the fore-arm part of the intake mechanism
   * armExtend() lowers the intake mechanism
   */
  public void retractArm() {
    armIntakeSolenoid.set(Value.kReverse);
  }

  public void spinBallIn() {
    // if(getIsBoomOut() && getIsArmOut())
    intakeMotor.set(POWER);
  }

  public void spinBallOut() {
    if (getIsBoomOut() && getIsArmOut())
      intakeMotor.set(-POWER);
  }

  public void intakeOut() {
    timer.start();
    if (!boomIsOut) {
      extendBoom();
      timer.reset();
      if (!armIsOut && timer.get() >= 1000) {
        extendArm();
        if (timer.get() >= 2000) {
          spinBallIn();
        }
      }
    }
  }

  public void intakeIn() {
    timer.start();
    stopMotor();
    if (boomIsOut) {
      retractBoom();
      timer.reset();
      if (armIsOut && timer.get() >= 1000) {
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
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter/Intake"); // Data is grouped with shooter and intake.
    boomIsOutEntry = tab.add("Boom Is Out", false)
        .withPosition(0, 3)
        .withSize(1, 1)
        .getEntry();

    armIsOutEntry = tab.add("Arm Is Out", false)
        .withPosition(1, 3)
        .withSize(1, 1)
        .getEntry();

    intakeMotorSpeedEntry = tab.add("Intake Motor Speed", 0)
        .withPosition(2, 3)
        .withSize(1, 1)
        .getEntry();
  }

  private void updateTelemetry() {
    boomIsOutEntry.setBoolean(boomIsOut);
    armIsOutEntry.setBoolean(armIsOut);
    intakeMotorSpeedEntry.setValue(intakeMotor.get());
  }

}