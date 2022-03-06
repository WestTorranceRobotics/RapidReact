// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ShootTwoBallsUsingDirectPower extends CommandBase {
  private Shooter mshooter;
  private Loader mLoader;
  private boolean isDone = false;
  /** Creates a new ShootTwoBallsUsingDirectPower. */
  public ShootTwoBallsUsingDirectPower(Shooter shooter, Loader loader) {
    mshooter = shooter;
    mLoader = loader;

    addRequirements(mshooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mshooter.setPower(0.85);
    

    if (Math.abs(mshooter.getVelocity()) >= 4000 && !mshooter.atSpeed()) {
      mshooter.atSpeed(true);
    }

    if (mshooter.atSpeed()) {
      mLoader.runLoader();
    }
    
    // unsure if two balls will allow the shooter to reach the velocity in line 35,
    // which means I'm unsure if it will be at the correct speed and if it will
    // even add one to the ball counter.
    if(mshooter.atSpeed() && mLoader.getAppliedOutput() < 0) {
      mshooter.currentWatch();
    }

    if (mshooter.getBallsShot() == 2) {
      isDone = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mshooter.setPower(0);
    mLoader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
