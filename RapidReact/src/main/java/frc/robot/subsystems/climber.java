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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  public enum ClimbMode{
    Start,
    FirstBar,
    SecondBar, 
    Traversal,
  }

  public enum HookState {
    OpenFront,
    OpenBack,
    LockedFront
  };

  public enum WindmillState {
    FirstBarClimb,
    Continuous,
    ShiftWeight,
    Hang
  };

  public enum ElevatorState {
    ElevatorUp,
    ElevatorDown
  };

  public enum HookSet {
    AB,
    XY
  }

  private HookState currentHookState;
  private WindmillState currentWindmillState;
  private ElevatorState currentElevatorState;
  private ClimbMode currentClimbMode;
  private HookSet currentHookSet;

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
    public DigitalInput homeLimitSwitch;

    // Hook Constants
    private final double MAX_HOOK_ANGLE = 160;
    private final double MIN_HOOK_ANGLE = 0; 
    private final double HOOK_UPPER_LIMIT = 95000;
    private final double HOOK_LOWER_LIMIT = 0;
    private final double ROTATIONS_PER_DEGREE = (HOOK_UPPER_LIMIT - HOOK_LOWER_LIMIT) / (MAX_HOOK_ANGLE - MIN_HOOK_ANGLE);

    private final double kHookMinOutput = -1;
    private final double kHookMaxOutput = 1;

    // positions based on encoders
    private double openToFront = -5;
    private double openToBack = MAX_HOOK_ANGLE - 3;
    private double lockedToFront = -23;

    // Tracking Info
    public boolean goingHome = true;
    private double targetAngle  = 0;
    private double currentAngle = 0;

    // All of these args are in Degreas
    private Hook(int HookCanId, int HookLimitSwitch1Id, int HookLimitSwitch2Id, int HookRotationLimitSwitchId) {

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
      
      hookLimitSwitch1 = new DigitalInput(HookLimitSwitch1Id);
      hookLimitSwitch2 = new DigitalInput(HookLimitSwitch2Id);
      homeLimitSwitch = new DigitalInput(HookRotationLimitSwitchId);

      hookMotor.setIdleMode(IdleMode.kCoast);
    }

    private Hook(int HookCanId, int HookLimitSwitch1Id, int HookRotationLimitSwitchId) {
      hookMotor = new CANSparkMax(HookCanId, MotorType.kBrushless);
      hookPosition = hookMotor.getEncoder();
      hookPositionPID = hookMotor.getPIDController();

      hookMotor.setInverted(true);

      // Setting the PID Values
      hookPositionPID.setP(hookP);
      hookPositionPID.setI(hookI);
      hookPositionPID.setD(hookD);
      hookPositionPID.setIZone(hookIz);
      hookPositionPID.setFF(hookFF);
      hookPositionPID.setOutputRange(kHookMinOutput, kHookMaxOutput);
      
      hookLimitSwitch1 = new DigitalInput(HookLimitSwitch1Id);
      homeLimitSwitch = new DigitalInput(HookRotationLimitSwitchId);

      hookMotor.setIdleMode(IdleMode.kCoast);
    }

    private void homeHook() {
      if (!goingHome) {
          return;
      }

      // System.out.println(hookMotor.getOutputCurrent() + "------------");
      if (hookMotor.getOutputCurrent() <= 1) {
        hookMotor.set(-0.3);
        return;
      }

      // if (!homeLimitSwitch.get()) {
      //     hookMotor.set(-0.3);
      //     return;
      // }

      goingHome = false;
      hookMotor.set(0);
      targetAngle = openToBack;
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

    public void stopHook(){
      hookMotor.set(0);
    }

    // returns true if the hook is at the position.
    public void setHookPosition(HookState position) {
      switch (position) {
        case OpenFront:
          setAngle(openToFront);
          currentHookState = HookState.OpenFront;
          break;
        case OpenBack:
          setAngle(openToBack);
          currentHookState = HookState.OpenBack;
          break;
        case LockedFront:
          setAngle(lockedToFront);
          currentHookState = HookState.LockedFront;
          break;
      }
    }

    public boolean hooksAgainstBar(){
      //Hook XY does not have limitSwitch2
      if(hookLimitSwitch1.get() && hookLimitSwitch2 == null){
        return true;
      }
      if(hookLimitSwitch1.get() && hookLimitSwitch2.get()){
        return true;
      }
      return false;
    }

    public boolean hookLimitSwitch1(){
      return hookLimitSwitch1.get();
    }

    public boolean hookLimitSwitch2(){
      return hookLimitSwitch2.get();
    }

    public boolean homeLimitSwitch(){
      return homeLimitSwitch.get();
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

    // Timers
    // private Timer windmillOvertravelTimer;
    // private boolean windmillOvertravelTimerStarted;

    // Gear Ratios
    private final double GEAR_RATIO = 5; // 5 to 1

    // Windmill Positions
    private final double CLIMB_ROTAION = 100;
    private final double FIRST_TO_SECOND = 80;
    private final double CONTINOUS = 1000;
    private final double FINAL_BAR_CLIMB_ROTAION = 100;
    private final double SHIFT_WEIGHT_ROTATION = -10;
    private final double END_ROTATION = 0;

    // PID Values
    private double windmillP = 0.0000001;
    private double windmillI = 0.0;
    private double windmillD = 0.0;

    public Windmill(int WindmillCanId, int WindmillFollowerCanId, int WindmillLimitSwitchId) {
      // Creating Objects
      windmillMotor = new CANSparkMax(WindmillCanId, MotorType.kBrushless);
      windmillFollowerMotor = new CANSparkMax(WindmillFollowerCanId, MotorType.kBrushless);
      // windmillOvertravelTimer = new Timer();

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
      // if (!windmillOvertravelTimerStarted) {
      //   windmillOvertravelTimer.start();
      // }
      windmillMotor.set(0.2);
      if (windmillLimitSwitch.get()) {
        windmillMotor.set(0);
        windmillRotaionZero = windmillEncoder.getPosition();
      }
      // if (windmillOvertravelTimer.get() <= 0.5) {
      //   windmillMotor.set(0);
      // }
    }

    public void stopWindmill(){
      windmillMotor.set(0);
    }

    public boolean windmillLimitSwitch(){
      return windmillLimitSwitch.get();
    }

    public void rotateWindmill(WindmillState position) {
      switch (position) {
        case FirstBarClimb:
          windmillPIDController.setReference(CLIMB_ROTAION + windmillRotaionZero, ControlType.kPosition);
          currentWindmillState = WindmillState.FirstBarClimb;
          break;
        case Continuous:
          windmillPIDController.setReference(CONTINUOUS + windmillRotaionZero, ControlType.kVelocity);
          currentWindmillState = WindmillState.Continuous;
          break;
        case ShiftWeight:
          windmillPIDController.setReference(SHIFT_WEIGHT_ROTATION + windmillRotaionZero, ControlType.kPosition);
          currentWindmillState = WindmillState.ShiftWeight;
          break;
        case Hang:
          windmillPIDController.setReference(END_ROTATION + windmillRotaionZero, ControlType.kPosition);
          currentWindmillState = WindmillState.Hang;
          break;
      }
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

    // Speed
    private double upPower = -0.2;
    private double downPower = 0.1;

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

    public void stopElevator(){
      elevatorMotor.set(0);
    }

    public boolean elevatorLimitSwitch(){
      return elevatorLimitSwitch.get();
    }

    public void setElevatorPosition(ElevatorState position) {
      switch (position) {
        case ElevatorUp:
          elevatorMotor.set(upPower);
          if(elevatorLimitSwitch.get()) elevatorMotor.set(0);
          currentElevatorState = ElevatorState.ElevatorUp;
          break;
        case ElevatorDown:
          elevatorMotor.set(downPower);
          currentElevatorState = ElevatorState.ElevatorDown;
          break;
      }
    }
  }

  private Hook hookAB;
  private Hook hookXY;
  private Windmill windmill;
  private Elevator elevator;

  private boolean attatchedAB;
  private boolean attatchedXY;
  private boolean detatchedAB;
  private boolean detatchedXY;

  // Shuffleboard Entrys
  private NetworkTableEntry elevatorLimitSwitchEntry;

  private NetworkTableEntry windmillLimitSwitchEntry;
  private NetworkTableEntry windmillAngleEntry;

  private NetworkTableEntry hooksABHomeEntry;
  private NetworkTableEntry hooksXYHomeEntry;
  private NetworkTableEntry hooksAOnBarEntry;
  private NetworkTableEntry hooksBOnBarEntry;
  private NetworkTableEntry hooksXOnBarEntry;
  private NetworkTableEntry currentHookABAngleEntry;
  private NetworkTableEntry targetHookABAngleEntry;
  private NetworkTableEntry currentHookXYAngleEntry;
  private NetworkTableEntry targetHookXYAngleEntry;


  public ClimberSubsystem(int ElevatorCanId, int WindmillCanId, int WindmillFollowerCanId, int HookABCanId, int HookXYCanId,
      int ElevatorLimitSwitchId, int WindmillLimitSwitchId, int HookABRotationLimitSwitchId, int HookXYRotationLimitSwitchId, int Hook1LimitSwitchId, 
      int Hook2LimitSwitchId, int Hook3LimitSwitchId) {

    hookAB = new Hook(HookABCanId, Hook1LimitSwitchId, Hook2LimitSwitchId, HookABRotationLimitSwitchId);
    hookXY = new Hook(HookXYCanId, Hook3LimitSwitchId, HookXYRotationLimitSwitchId);
    windmill = new Windmill(WindmillCanId, WindmillFollowerCanId, WindmillLimitSwitchId);
    elevator = new Elevator(ElevatorCanId, ElevatorLimitSwitchId);

    initTelemetry();
  }

  public void setClimberPosition(ClimbMode position){
    switch (position) {
      case Start:
        hookAB.setHookPosition(HookState.OpenFront);
        hookXY.setHookPosition(HookState.OpenFront);
        currentClimbMode = ClimbMode.Start;
        break;
      case FirstBar:
        startToFirst();
        break;
      case SecondBar:
        if(currentClimbMode == ClimbMode.Start){
          startToSecond();
        }else if(currentClimbMode == ClimbMode.FirstBar){
          firstToSecond();
        }
        break;
      case Traversal:
        if(currentClimbMode == ClimbMode.Start){
          startToFinish();
        }else if(currentClimbMode == ClimbMode.FirstBar){
          firstToFinish();
        }else if(currentClimbMode == ClimbMode.SecondBar){
          secondToFinish();
        }
        currentClimbMode = ClimbMode.Traversal;
        break;
    }
  }

  // done
  public void startToFirst(){
    elevator.setElevatorPosition(ElevatorState.ElevatorUp);
    attatchAB();
    windmill.rotateWindmill(WindmillState.FirstBarClimb);
    currentClimbMode = ClimbMode.FirstBar;
  }

  // done
  public void startToSecond(){
    elevator.setElevatorPosition(ElevatorState.ElevatorUp);
    attatchAB();
    if(attatchedAB){
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchXY();
      if(attatchedXY){
        detatchAB();
        windmill.rotateWindmill(WindmillState.Hang);
        currentClimbMode = ClimbMode.SecondBar;
      }
    }
  }

  // done
  public void startToFinish(){
    elevator.setElevatorPosition(ElevatorState.ElevatorUp);
    attatchAB();
    if(attatchedAB){
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchXY();
      if(attatchedXY){
        currentClimbMode = ClimbMode.SecondBar;
        detatchAB();
      }
    }
    if(detatchedAB && currentClimbMode == ClimbMode.SecondBar){
      windmill.rotateWindmill(WindmillState.Hang);
      hookAB.setHookPosition(HookState.OpenFront);
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchAB();
    }
    if(attatchedAB && currentClimbMode == ClimbMode.SecondBar){
      detatchXY();
      windmill.rotateWindmill(WindmillState.Hang);
    }
    currentClimbMode = ClimbMode.Traversal;
  }

  // done
  public void firstToSecond(){
    if(currentClimbMode == ClimbMode.FirstBar){
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchXY();
      if(attatchedXY){
        detatchAB();
        windmill.rotateWindmill(WindmillState.Hang);
        currentClimbMode = ClimbMode.SecondBar;
      }
    }
  }

  //done
  public void firstToFinish(){
    if(currentClimbMode == ClimbMode.FirstBar){
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchXY();
      if(attatchedXY){
        currentClimbMode = ClimbMode.SecondBar;
        detatchAB();
      }
    }
    if(detatchedAB && currentClimbMode == ClimbMode.SecondBar){
      windmill.rotateWindmill(WindmillState.Hang);
      hookAB.setHookPosition(HookState.OpenFront);
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchAB();
    }
    if(attatchedAB && currentClimbMode == ClimbMode.SecondBar){
      detatchXY();
      windmill.rotateWindmill(WindmillState.Hang);
    }
    currentClimbMode = ClimbMode.Traversal;
  }

  // done
  public void secondToFinish(){
    if(currentClimbMode == ClimbMode.SecondBar){
      windmill.rotateWindmill(WindmillState.Hang);
      hookAB.setHookPosition(HookState.OpenFront);
      windmill.rotateWindmill(WindmillState.Continuous);
      attatchAB();
      if(attatchedAB){
        detatchXY();
        windmill.rotateWindmill(WindmillState.Hang);
        currentClimbMode = ClimbMode.Traversal;
      }
    }
  }

  public void attatchAB(){
    if(hookAB.hooksAgainstBar()){
      hookAB.setHookPosition(HookState.LockedFront);
      attatchedAB = true;
      detatchedAB = false;
    }
  }

  public void attatchXY(){
    if(hookXY.hooksAgainstBar()){
      hookXY.setHookPosition(HookState.LockedFront);
      attatchedXY = true;
      detatchedXY = false;
    }
  }

  public void detatchAB(){
    if(currentHookState == HookState.LockedFront){
      windmill.rotateWindmill(WindmillState.ShiftWeight);
      hookAB.setHookPosition(HookState.OpenBack);
      windmill.rotateWindmill(WindmillState.Hang);
      detatchedAB = true;
      attatchedAB = false;
    }
  }

  public void detatchXY(){
    if(currentHookState == HookState.LockedFront){
      windmill.rotateWindmill(WindmillState.ShiftWeight);
      hookXY.setHookPosition(HookState.OpenBack);
      windmill.rotateWindmill(WindmillState.Hang);
      detatchedXY = true;
      attatchedXY = false;
    }
  }

  public void stopAll(){
    hookAB.stopHook();
    hookXY.stopHook();
    windmill.stopWindmill();
    elevator.stopElevator();
  }

  public void initializeZeros(){
    hookAB.homeHook();
    hookXY.homeHook();
    // windmill.homeWindmill();
  }

  //TESTING METHODS

  public void testAttatchAB(){
    attatchAB();
    // attatchXY();
  }

  public void testAttatchXY(){
    // attatchAB();
    attatchXY();
  }

  public void hookOpenBack(){
    hookAB.setHookPosition(HookState.OpenBack);
    hookXY.setHookPosition(HookState.OpenBack);
  }

  public void hookOpenFront(){
    hookAB.setHookPosition(HookState.OpenFront);
    hookXY.setHookPosition(HookState.OpenFront);
  }

  public void hookLockFront(){
    hookAB.setHookPosition(HookState.LockedFront);
    hookXY.setHookPosition(HookState.LockedFront);
  }

  // ENDS TESTING METHODS

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    initializeZeros();

    hookAB.currentAngle = hookAB.encoderPositionToAngle(hookAB.hookPosition.getPosition());

    testAttatchAB();// automatic lock on bar
    testAttatchXY();

    updateTelemetry();
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Climber"); 

    // ELEVATOR
    elevatorLimitSwitchEntry = tab.add("Elevator Limit", false)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();

    // WINDMILL
    windmillLimitSwitchEntry = tab.add("Windmill Limit", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

    // HOOKS
    hooksABHomeEntry = tab.add("Hook AB Home Limit", false)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();

    hooksXYHomeEntry = tab.add("Hook XY Home Limit", false)
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry();

    hooksAOnBarEntry = tab.add("Hook A Bar Limit", false)
            .withPosition(2, 2)
            .withSize(1, 1)
            .getEntry();

    hooksBOnBarEntry = tab.add("Hook B Bar Limit", false)
            .withPosition(3, 2)
            .withSize(1, 1)
            .getEntry();

    hooksXOnBarEntry = tab.add("Hook X Bar Limit", false)
            .withPosition(4, 2)
            .withSize(1, 1)
            .getEntry();

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
    elevatorLimitSwitchEntry.setBoolean(elevator.elevatorLimitSwitch());
    windmillLimitSwitchEntry.setBoolean(windmill.windmillLimitSwitch());
    hooksABHomeEntry.setBoolean(hookAB.homeLimitSwitch());
    hooksXYHomeEntry.setBoolean(hookXY.homeLimitSwitch());
    hooksAOnBarEntry.setBoolean(hookAB.hookLimitSwitch1());
    hooksBOnBarEntry.setBoolean(hookAB.hookLimitSwitch2());
    hooksXOnBarEntry.setBoolean(hookXY.hookLimitSwitch1());
    currentHookABAngleEntry.setDouble(hookAB.MAX_HOOK_ANGLE - hookAB.currentAngle);
    currentHookXYAngleEntry.setDouble(hookXY.MAX_HOOK_ANGLE - hookAB.currentAngle);
    targetHookABAngleEntry.setDouble(hookAB.targetAngle);
    targetHookXYAngleEntry.setDouble(hookXY.targetAngle);
}

}

//current spike for hook zero
//robot pitch angle instead of windmill rotation (ask mr yang)
