// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.security.Timestamp;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ShootingTwoBallsUsingLQR extends CommandBase {
  boolean isDone = false;
  Shooter mShooter;
  Loader mLoader;
  double m_rpm;
  boolean timeStart = false;
  boolean isEnd = false;
  int shotBall = 0;
  private Timer shootTimer;
  /** Creates a new ShootingTwoBallsUsingLQR. */
  public ShootingTwoBallsUsingLQR(Shooter shooter, Loader loader, double rpm, boolean end) {
    mShooter = shooter;
    m_rpm = rpm;
    mLoader = loader;
    isEnd = end;
    shootTimer = new Timer();
    addRequirements(mShooter, mLoader);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer.reset();
    shotBall = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setReferenceVelocity(m_rpm);

     // Correct our Kalman filter's state vector estimate with encoder data.
     mShooter.getLinearSystemLoopLeader().correct(VecBuilder.fill(mShooter.getVelocityLeader()));
     //mShooter.getLinearSystemLoopFollower().correct(VecBuilder.fill(mShooter.getVelocityFollower()));

     // Update our LQR to generate new voltage commands and use the voltages to predict the next
     // state with out Kalman filter.
     mShooter.getLinearSystemLoopLeader().predict(0.020);
     //mShooter.getLinearSystemLoopFollower().predict(0.020);
 
     // Send the new calculated voltage to the motors.
     // voltage = duty cycle * battery voltage, so
     // duty cycle = voltage / battery voltage
     double nextVoltageLeader = mShooter.getLinearSystemLoopLeader().getU(0);
     //double nextVoltageFollower = mShooter.getLinearSystemLoopFollower().getU(0);

     mShooter.getShootMotorLeader().setVoltage(nextVoltageLeader);
     //mShooter.getShootFollowerLeader().setVoltage(nextVoltageFollower);


    if (Math.abs(mShooter.getVelocity()) >= (m_rpm+50) && !mShooter.atSpeed()) {
      mShooter.atSpeed(true);
      shootTimer.start();
    }

    if (mShooter.atSpeed()) {
      if (isEnd) {
        if (shootTimer.hasElapsed(1.00)) {
          mLoader.runLoader(-0.35);
        }
      }
      else {
        mLoader.runLoader(-0.35);
      }
      if(mShooter.getVelocity() <= m_rpm-200){
        // System.out.println(shotBall);
        mLoader.stopLoader();
        mShooter.atSpeed(false);
        shotBall += 1;
      }
    }
    
    if (isEnd) {
      if (shootTimer.hasElapsed(4.50)) {
        isDone = true;
      }
    }
    else if (shotBall == 2 || shootTimer.hasElapsed(1.50)) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.getShootMotorLeader().setVoltage(0);
    mLoader.stopLoader();
    timeStart = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
