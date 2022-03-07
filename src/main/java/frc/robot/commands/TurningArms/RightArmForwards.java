// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurningArms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RightArm;

public class RightArmForwards extends CommandBase {
  /** Creates a new RightArmForwards. */
  private RightArm arm;
  private boolean enabled;
  public RightArmForwards(RightArm arm) {
    this.arm = arm;
    enabled = true;
    addRequirements(this.arm);
  }

  public RightArmForwards(RightArm arm, boolean overrideEnabled) {
    this.arm = arm;
    enabled = overrideEnabled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (enabled) {
      arm.armForwards();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
