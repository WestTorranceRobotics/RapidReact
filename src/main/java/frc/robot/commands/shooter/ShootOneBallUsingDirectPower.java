// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ShootOneBallUsingDirectPower extends CommandBase {
  private Shooter mshooter;
  private Loader mLoader;
  private double mpower;
  private double mrpm;
  private boolean isDone;
  private Timer shootTimer;
  /** Creates a new ShootOneBallUsingDirectPower. */
  public ShootOneBallUsingDirectPower(Shooter shooter, Loader loader, double power, double rpm) {
    mshooter = shooter;
    mLoader = loader;
    mpower = power;
    mrpm = rpm;
    isDone = false;
    shootTimer = new Timer();

    addRequirements(mshooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mshooter.setPower(mpower);
    

    if (Math.abs(mshooter.getVelocity()) >= mrpm && !mshooter.atSpeed()) {
      mshooter.atSpeed(true);
    }

    if (mshooter.atSpeed()) {
      mLoader.runLoader();
    }
    
    if(mshooter.atSpeed() && mLoader.getAppliedOutput() < 0) {
      shootTimer.start();
    }

    if (shootTimer.hasElapsed(1.75)) {
      isDone = true;
    }

    /* previous code with currentWatch
    if (mshooter.getBallsShot() == 1) {
      isDone = true;
    }
    */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mshooter.setPower(0);
    mLoader.stopLoader();
    shootTimer.stop();
    shootTimer.reset();
    mshooter.resetBallShot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
