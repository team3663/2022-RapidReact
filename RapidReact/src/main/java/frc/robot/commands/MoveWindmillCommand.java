// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class MoveWindmillCommand extends CommandBase {
  private double speed = 0;
  private ClimberSubsystem climber;

  public MoveWindmillCommand(ClimberSubsystem climber, double speed) {
    addRequirements(climber);
    this.climber = climber;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    climber.windmill.setWindmillOutput(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.windmill.setWindmillOutput(0);
  }
}
