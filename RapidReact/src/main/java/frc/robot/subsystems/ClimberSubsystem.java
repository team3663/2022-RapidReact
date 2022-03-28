// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  public enum HookPosition {
    Grab,
    Release,
    Lock
  };

  public enum WindmillState {
    FirstBarClimb,
    FirstToSecond,
    SecondToThird,
    ShiftWeight,
    Hang
  };

  public enum HookSet {
    Red,
    Blue
  }

  public HookPosition currentHookPosition;
  public WindmillState currentWindmillState;
  public HookSet currentHookSet;

  // Hook Helper class.
  private class Hook {

    private RelativeEncoder hookPosition;
    private CANSparkMax hookMotor;
    private SparkMaxPIDController hookPositionPID;

    // PID values
    private double hookP = 0.001;
    private double hookI = 0;
    private double hookD = 0;
    private double hookIz = 0;
    private double hookFF = 0;

    // LimitSwitches
    public DigitalInput hookLimitSwitch1;
    public DigitalInput hookLimitSwitch2;

    // Hook Constants
    private final double MAX_HOOK_ANGLE = 160;
    private final double MIN_HOOK_ANGLE = 0; 
    private final double HOOK_UPPER_LIMIT = 95000;
    private final double HOOK_LOWER_LIMIT = 0;
    private final double ROTATIONS_PER_DEGREE = (HOOK_UPPER_LIMIT - HOOK_LOWER_LIMIT) / (MAX_HOOK_ANGLE - MIN_HOOK_ANGLE);
    private final double kHookMinOutput = -1;
    private final double kHookMaxOutput = 1;

    // positions based on encoders
    private double grab = -5;
    private double release = MAX_HOOK_ANGLE - 3;
    private double lock = -23;

    // Tracking Info
    public boolean goingHome = true;
    private double targetAngle  = 0;
    private double currentAngle = 0;

    // All of these args are in Degreas
    private Hook(int HookCanId, HookSet hookSet) {
      switch(hookSet){
        case Blue:
          hookMotor.setInverted(true);
      }

      hookMotor = new CANSparkMax(HookCanId, MotorType.kBrushless);
      hookPosition = hookMotor.getEncoder();
      hookPositionPID = hookMotor.getPIDController();

      // Setting the PID Values
      hookPositionPID.setP(hookP);
      hookPositionPID.setI(hookI);
      hookPositionPID.setD(hookD);
      hookPositionPID.setIZone(hookIz);
      hookPositionPID.setFF(hookFF);
      hookPositionPID.setOutputRange(kHookMinOutput, kHookMaxOutput);

      hookMotor.setIdleMode(IdleMode.kBrake);
    }

    private void homeHook() {
      if (!goingHome) {
          return;
      }

      if (hookMotor.getOutputCurrent() <= 1) { 
        System.out.println(encoderPositionToAngle(hookPosition.getPosition()));
        hookMotor.set(-0.2);
        return;
      }

      goingHome = false;
      hookMotor.set(0);
      targetAngle = release;
      hookPosition.setPosition(angleToEncoderPosition(targetAngle));
      hookPositionPID.setReference(angleToEncoderPosition(targetAngle), ControlType.kPosition);
    }

    private double angleToEncoderPosition(double angle) {
      return (MAX_HOOK_ANGLE - angle) * ROTATIONS_PER_DEGREE;
    }

    private double encoderPositionToAngle(double position) {
      return ((HOOK_UPPER_LIMIT - position) / ROTATIONS_PER_DEGREE) + MIN_HOOK_ANGLE;
    }

    public void setAngle(double angle) {
      targetAngle = angle;
      hookPositionPID.setReference(angleToEncoderPosition(targetAngle), ControlType.kPosition);
    }

    public void setHookPosition(HookPosition position) {
      switch (position) {
        case Grab:
          setAngle(grab);
          currentHookPosition = HookPosition.Grab;
          break;
        case Release:
          setAngle(release);
          currentHookPosition = HookPosition.Release;
          break;
        case Lock:
          setAngle(lock);
          currentHookPosition = HookPosition.Lock;
          break;
      }
    }

    public HookPosition getHookPosition(){
      return currentHookPosition;
    }
  }

  private class Windmill {
    // Phisical controllers
    private CANSparkMax windmillMotor;
    private CANSparkMax windmillFollowerMotor;
    private RelativeEncoder windmillEncoder;

    private SparkMaxPIDController windmillPIDController;

    private DigitalInput windmillLimitSwitch;

    // Phisical Offsets and speeds
    private double windmillRotaionZero;
    private double windmillRotationSpeed = 0.5;

    // Gear Ratios
    private final double GEAR_RATIO = 4; // 4 to 1

    // Windmill Constants
    private final double MAX_WINDMILL_ANGLE = 360;
    private final double MIN_WINDMILL_ANGLE = 0; 
    private final double WINDMILL_UPPER_LIMIT = 95000;
    private final double WINDMILL_LOWER_LIMIT = 0;
    private final double ROTATIONS_PER_DEGREE = (WINDMILL_UPPER_LIMIT - WINDMILL_LOWER_LIMIT) / (MAX_WINDMILL_ANGLE - MIN_WINDMILL_ANGLE);
    private final double kWindmillMinOutput = -1;
    private final double kWindmillMaxOutput = 1;

    // Windmill Positions
    private double currentAngle;
    private double targetAngle;

    private final double FIRST_BAR_CLIMB = 0;
    private final double FIRST_TO_SECOND = 80;
    private final double SHIFT_WEIGHT_ROTATION = currentAngle - 10;
    private final double SECOND_TO_THIRD = 1000;
    private final double HANG = 0;

    // PID Values
    private double windmillP = 0.0000001;
    private double windmillI = 0.0;
    private double windmillD = 0.0;

    public Windmill(int WindmillCanId, int WindmillFollowerCanId, int WindmillLimitSwitchId) {
      // Creating Objects
      windmillMotor = new CANSparkMax(WindmillCanId, MotorType.kBrushless);
      windmillFollowerMotor = new CANSparkMax(WindmillFollowerCanId, MotorType.kBrushless);

      // Setting Modes
      windmillFollowerMotor.follow(windmillMotor, true);
      windmillFollowerMotor.setIdleMode(IdleMode.kBrake);
      windmillMotor.setIdleMode(IdleMode.kBrake);

      // Setting Local Varbles
      windmillPIDController = windmillMotor.getPIDController();
      windmillEncoder = windmillMotor.getEncoder();
      windmillEncoder.setPositionConversionFactor(GEAR_RATIO);
      windmillLimitSwitch = new DigitalInput(WindmillLimitSwitchId);

      // Setting PIDs
      windmillPIDController.setP(windmillP);
      windmillPIDController.setI(windmillI);
      windmillPIDController.setD(windmillD);
      windmillPIDController.setOutputRange(windmillRotationSpeed, -windmillRotationSpeed);
    }

    public void homeWindmill() {
      windmillMotor.set(0.2);
      if (windmillLimitSwitch.get()) {
        windmillMotor.set(0);
        windmillRotaionZero = windmillEncoder.getPosition();
      }
    }

    private double angleToEncoderPosition(double angle) {
      return (MAX_WINDMILL_ANGLE - angle) * ROTATIONS_PER_DEGREE;
    }

    private double encoderPositionToAngle(double position) {
      return ((WINDMILL_UPPER_LIMIT - position) / ROTATIONS_PER_DEGREE) + MIN_WINDMILL_ANGLE;
    }

    public void setAngle(double angle) {
      targetAngle = angle;
      windmillPIDController.setReference(angleToEncoderPosition(targetAngle), ControlType.kPosition);
    }

    public void rotateWindmill(WindmillState position) {
      switch (position) {
        case FirstBarClimb:
          setAngle(FIRST_BAR_CLIMB);
          currentWindmillState = WindmillState.FirstBarClimb;
          break;
        case FirstToSecond:
          setAngle(FIRST_TO_SECOND);
          currentWindmillState = WindmillState.FirstToSecond;
          break;
        case SecondToThird:
          setAngle(SECOND_TO_THIRD);
          currentWindmillState = WindmillState.SecondToThird;
          break;
        case ShiftWeight:
          setAngle(SHIFT_WEIGHT_ROTATION);
          currentWindmillState = WindmillState.ShiftWeight;
          break;
        case Hang:
          setAngle(HANG);
          currentWindmillState = WindmillState.Hang;
          break;
      }
    }

    public WindmillState getWindmillState(){
        return currentWindmillState;
    }
  }

  private class Elevator {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;

    // Limit Switch
    private DigitalInput elevatorLimitSwitch;

    // Elevator PID
    private double elevatorP;
    private double elevatorI;
    private double elevatorD;

    // Gear Ratio
    private double gearRatio = 1;

    private double upPower = -0.1;
    private boolean elevatorUp;

    public Elevator(int ElevatorCanId, int ElevatorLimitSwitchId) {
      elevatorMotor = new CANSparkMax(ElevatorCanId, MotorType.kBrushless);

      elevatorEncoder = elevatorMotor.getEncoder();
      elevatorPIDController = elevatorMotor.getPIDController();

      elevatorEncoder.setPositionConversionFactor(gearRatio);

      elevatorLimitSwitch = new DigitalInput(ElevatorLimitSwitchId);

      elevatorMotor.setIdleMode(IdleMode.kBrake);

      elevatorPIDController.setP(elevatorP);
      elevatorPIDController.setI(elevatorI);
      elevatorPIDController.setD(elevatorD);
    }

    public void extendElevator(){
        if(!elevatorLimitSwitch.get()){
            elevatorMotor.set(upPower);
        } else {
            elevatorMotor.set(0);
            elevatorUp = true;
        }
    }

    public boolean elevatorExtended(){
        return elevatorUp;
    }
  }

  private Hook hookRed;
  private Hook hookBlue;
  private Windmill windmill;
  private Elevator elevator;

  // Shuffleboard Entrys
  private NetworkTableEntry currentHookABAngleEntry;
  private NetworkTableEntry targetHookABAngleEntry;
  private NetworkTableEntry currentHookXYAngleEntry;
  private NetworkTableEntry targetHookXYAngleEntry;


  public ClimberSubsystem(int ElevatorCanId, int WindmillCanId, int WindmillFollowerCanId, int HookABCanId, int HookXYCanId,
      int ElevatorLimitSwitchId, int WindmillLimitSwitchId) {

    hookRed = new Hook(HookABCanId, HookSet.Red);
    hookBlue = new Hook(HookXYCanId, HookSet.Blue);
    windmill = new Windmill(WindmillCanId, WindmillFollowerCanId, WindmillLimitSwitchId);
    elevator = new Elevator(ElevatorCanId, ElevatorLimitSwitchId);

    initTelemetry();
  }

  //public Climber Methods
 
  // PUBLIC HOOK METHODS
  public HookPosition getRedHookPosition(){
    return hookRed.getHookPosition();
  }

  public HookPosition getBlueHookPosition(){
    return hookBlue.getHookPosition();
  }

  public void setRedHookPosition(HookPosition position){
    hookRed.setHookPosition(position);
  }

  public void setBlueHookPosition(HookPosition position){
    hookBlue.setHookPosition(position);
  }

  // PUBLIC WINDMILL METHODS
  public void setWindmillAngle(WindmillState position){
      windmill.rotateWindmill(position);
  }

  public WindmillState getWindmillPosition(){
      return windmill.getWindmillState();
  }

  // PUBLIC ELEVATOR METHODS
  public void elevatorUp(){
    elevator.extendElevator();
  }

  public boolean getElevatorExtended(){
    return elevator.elevatorExtended();
  }

  public void initializeZeros(){
    hookRed.homeHook();
    hookBlue.homeHook();
    // windmill.homeWindmill();
  }
  
  // TESTING 
  public void homeHook(){
      hookRed.homeHook();
  }

  // END TESTING

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // initializeZeros();

    hookRed.homeHook();

    hookRed.currentAngle = hookRed.encoderPositionToAngle(hookRed.hookPosition.getPosition());

    updateTelemetry();
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Climber"); 

    // ELEVATOR

    // WINDMILL

    // HOOKS
    currentHookABAngleEntry = tab.add("Current Angle", 0)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();

    targetHookABAngleEntry = tab.add("Target Angle", 0)
            .withPosition(6, 2)
            .withSize(1, 1)
            .getEntry();
    
    currentHookXYAngleEntry = tab.add("Current Angle", 0)
            .withPosition(7, 2)
            .withSize(1, 1)
            .getEntry();

    targetHookXYAngleEntry = tab.add("Target Angle", 0)
            .withPosition(8, 2)
            .withSize(1, 1)
            .getEntry();
  }

  private void updateTelemetry() {
    currentHookABAngleEntry.setDouble(hookRed.MAX_HOOK_ANGLE - hookRed.currentAngle);
    currentHookXYAngleEntry.setDouble(hookBlue.MAX_HOOK_ANGLE - hookBlue.currentAngle);
    targetHookABAngleEntry.setDouble(hookRed.targetAngle);
    targetHookXYAngleEntry.setDouble(hookBlue.targetAngle);
}

}

//targetPosition to simplify end for the commands