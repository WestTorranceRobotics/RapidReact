// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootBallBasedOnPower extends CommandBase {
  Shooter mShooter;
  double mpower;
  /** Creates a new ShootBallBasedOnPower. */
  public ShootBallBasedOnPower(Shooter shooter, double power) {
    mShooter = shooter;
    mpower = power;
    addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double networkSpeed = NetworkTableInstance.getDefault().getTable("Vision").getEntry("speed").getDouble(0);
    mShooter.setPower(networkSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
