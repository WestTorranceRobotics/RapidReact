// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class TurnToAngleUsingLimelight extends CommandBase {
  private DriveTrain subsystem;
  private PIDController controller;

  /** Creates a new TurnToAngleUsingLimelight. */
  public TurnToAngleUsingLimelight(DriveTrain subsystem) {
    this.subsystem = subsystem;
    subsystem.setP(0.052);
    controller = subsystem.getController();
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.enablePID();
    //getting epic stuff from the network table
    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0.0);
    
    // subsystem.isAutomatic(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight")
    .getEntry("tx").getDouble(0);
    System.out.println(tx);
    // System.out.println(subsystem.getP());
    double steeringAdjust;
    if (Math.abs(subsystem.getP() * tx) < 0.45) {
      steeringAdjust = subsystem.getP() * tx;
    }
    else {
      steeringAdjust = 0.45;
    }
    steeringAdjust = Math.signum(subsystem.getP() * tx) * Math.min(Math.abs(subsystem.getP() * tx), 0.45);
    // subsystem.tankDrive(0.5, 0.5);
    subsystem.tankDrive(steeringAdjust, steeringAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // subsystem.isAutomatic(false);
    subsystem.disablePID();
    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
