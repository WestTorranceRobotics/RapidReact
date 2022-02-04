// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleUsingLimelight extends CommandBase {
  private DriveTrain subsystem;
  private PIDController controller;
  private double kPDistance = 0.07;

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
    
    subsystem.setAutomatic(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCommand = 0;
    double rightCommand = 0;
    // /* turn to face the target */ 
    // double tx = NetworkTableInstance.getDefault().getTable("limelight")
    // .getEntry("tx").getDouble(0);
    // System.out.println(tx);
    // double steeringAdjust = Math.signum(tx) * Math.min(Math.abs(subsystem.getP() * tx), 0.45);

    // // attempt with built-in pid controller
    // // double steeringAdjust = Math.signum(tx) * Math.min(Math.abs(controller.calculate(tx, 0)), 0.45);
    // leftCommand += steeringAdjust;
    // rightCommand += steeringAdjust;


    /* drive to correct distance from target */

    double ty = NetworkTableInstance.getDefault().getTable("limelight")
    .getEntry("ty").getDouble(0);
    System.out.println(ty);

    leftCommand += Math.signum(ty) * Math.min(Math.abs(kPDistance * ty), 0.35);
    rightCommand -= Math.signum(ty) * Math.min(Math.abs(kPDistance * ty), 0.35);

    subsystem.tankDrive(leftCommand, rightCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setAutomatic(false);
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
