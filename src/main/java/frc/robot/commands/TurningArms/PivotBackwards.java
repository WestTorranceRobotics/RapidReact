// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurningArms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurningArms;

public class PivotBackwards extends CommandBase {
  private TurningArms arms;
  /** Creates a new PivotBackwards. */
  public PivotBackwards(TurningArms arms) {
    this.arms = arms;
    addRequirements(this.arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.liftBackwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.setNoPower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}