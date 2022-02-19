package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private double power = 0.4;
  final DoubleSolenoid intakeSolenoid;
  private boolean isOut;
  private final int MOTOR_CURRENT_LIMIT = 25;

  /** Creates a new instance of the Shooter subsystem. */
  public IntakeSubsystem(int motor1CANId, int retractSolenoidChan, int extendSolenoidChan) {

    intakeMotor = new CANSparkMax(motor1CANId, MotorType.kBrushless);

    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);

 //   intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, retractSolenoidChan, extendSolenoidChan);
    intakeSolenoid = null;
  }

  @Override
  public void periodic() { }

  public void start() {
    intakeSolenoid.set(Value.kForward);
    if(isOut == true){
      intakeMotor.set(power);
    } 
  }

  public void stop() {
    retract();
    intakeMotor.set(0.0);
  }

  public void extend(){
    intakeSolenoid.set(Value.kForward);
    isOut = true;
  }

  public void retract(){
    intakeSolenoid.set(Value.kReverse);
    isOut = false;
  }

  public void spinBallIn(){
    intakeMotor.set(power);
  }

  public void spinBallOut(){
    intakeMotor.set(-power);
  }
}
