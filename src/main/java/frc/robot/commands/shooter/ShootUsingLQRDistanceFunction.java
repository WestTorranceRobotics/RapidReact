// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import javax.swing.plaf.metal.MetalIconFactory.FolderIcon16;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootUsingLQRDistanceFunction extends CommandBase {
  Shooter mShooter;
  double m_rpm = 0;
  boolean isStartedShooting = false;
  Timer timer = new Timer();
  /** Creates a new ShootingUsingLQR. */
  public ShootUsingLQRDistanceFunction(Shooter shooter) {
    mShooter = shooter;
    m_rpm = 0;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTING WITH DISTANCE");
    mShooter.getLinearSystemLoopLeader().reset(VecBuilder.fill(mShooter.getVelocityLeader()));
    isStartedShooting = false;
    timer.reset();
    timer.start();
    //mShooter.getLinearSystemLoopFollower().reset(VecBuilder.fill(mShooter.getVelocityFollower()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = NetworkTableInstance.getDefault().getTable("Shooter").getEntry("distance").getDouble(0);
    // debug next two lines
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // System.out.println(ty + "\t" + m_rpm + "\t" + distance + "\t" + ((7.3 * distance) + 2100.0));
    if (timer.hasElapsed(0.23) && !isStartedShooting) {
      isStartedShooting = true;
      m_rpm = getSpeed(distance);
    }

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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.zeroReferenceVelocity();
    mShooter.getShootMotorLeader().setVoltage(0);
    isStartedShooting = false;
    timer.stop();
  }

  // function for inputting speed and getting rpm determined by 
  // plotting data points and finding a line of best fit
  private double getSpeed(double distance) {
    double inside = 52000.0 * distance;
    return Math.sqrt(inside) + 790.0;
    // return ((7.3 * distance) + 2170.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}