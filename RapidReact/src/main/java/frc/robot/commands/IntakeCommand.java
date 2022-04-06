// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private FeederSubsystem feederSubsystem;

  private BooleanSupplier trigger;
  
  private Timer intakeTimer;
  private boolean isExtended = false;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, BooleanSupplier trigger) {
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.trigger = trigger;

    intakeTimer = new Timer();
    addRequirements(intakeSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.extend();
    intakeTimer.reset();
    intakeTimer.start();
    isExtended = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(!isExtended && intakeTimer.hasElapsed(0.6)){
        feederSubsystem.setFeedMode(FeedMode.INTAKE);
        isExtended = true;
      }
    if(trigger.getAsBoolean()){
      intakeSubsystem.spinBallOut();
    } else {
      intakeSubsystem.spinBallIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isExtended = false;
    intakeTimer.stop();
    intakeSubsystem.retract();
    feederSubsystem.setFeedMode(FeedMode.STOPPED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
