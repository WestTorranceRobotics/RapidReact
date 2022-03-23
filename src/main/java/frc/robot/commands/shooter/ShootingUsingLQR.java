// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import javax.swing.plaf.metal.MetalIconFactory.FolderIcon16;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootingUsingLQR extends CommandBase {
  Shooter mShooter;
  double m_rpm = 0;
  int prevDistance = 0;
  boolean isShot = false;
  /** Creates a new ShootingUsingLQR. */
  public ShootingUsingLQR(Shooter shooter, double rpm) {
    mShooter = shooter;
    m_rpm = 0;
    addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.getLinearSystemLoopLeader().reset(VecBuilder.fill(mShooter.getVelocityLeader()));
    // m_rpm = getSpeed(NetworkTableInstance.getDefault().getTable("Shooter").getEntry("distance").getDouble(0));
    // double dist = NetworkTableInstance.getDefault().getTable("Shooter").getEntry("distance").getDouble(0);
    // m_rpm = 2100.0 + 7.3 * dist;
    System.out.println("INIT)");
    //mShooter.getLinearSystemLoopFollower().reset(VecBuilder.fill(mShooter.getVelocityFollower()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(isShot);
    if (mShooter.getCurrent() == 0)
    {
      double distance = NetworkTableInstance.getDefault().getTable("Shooter").getEntry("distance").getDouble(0);
      m_rpm = getSpeed(distance);
    }

     mShooter.setReferenceVelocity(m_rpm);
    //  System.out.println(mShooter.getVoltage());
     System.out.println(m_rpm);
     System.out.println(NetworkTableInstance.getDefault().getTable("Shooter").getEntry("distance").getDouble(0));

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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.zeroReferenceVelocity();
    isShot = false;
    mShooter.getShootMotorLeader().setVoltage(0);

  }

  public double getSpeed(double distance) {
    return ((7.3 * distance) + 2100.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}