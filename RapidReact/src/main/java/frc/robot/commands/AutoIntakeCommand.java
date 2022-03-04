// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeedMode;

public class AutoIntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  
  public AutoIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
    this.intake = intake;
    this.feeder = feeder;

    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {
    intake.extend(); // TODO this extend-spin-retract sequence doesn't seem to always work
    feeder.setFeedMode(FeedMode.INTAKE);
  }

  @Override
  public void execute() {
      intake.spinBallIn();
  }

  @Override
  public void end(boolean interrupted) {
    intake.retract();
  }

  @Override
  public boolean isFinished() {
    return (feeder.isIdle());
  }
}