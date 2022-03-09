// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class StayOnTarget extends CommandBase {
  private DriveTrain subsystem;

  private PIDController anglePID;
  // private PIDController distancePID;
  // private double kPDist = 0.07;
  // private double kIDist = 0.09;
  // private double kDDist = 0;
  // private double initTY;

  /** Creates a new TurnToAngleUsingLimelight. */
  public StayOnTarget(DriveTrain subsystem) {
    this.subsystem = subsystem;
    anglePID = subsystem.getAngleController();
    // distancePID = subsystem.getDistanceController();
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // subsystem.setP(kP);
    
    // distancePID.setP(kPDist);
    anglePID.setSetpoint(0);
    subsystem.enablePID();

    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0.0);

    // initTY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    subsystem.setAutomatic(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCommand = 0;
    double rightCommand = 0;

    /* stays on target distancewise*/
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // // System.out.println("ty: " + ty);
    // distancePID.setP(LLTable.getEntry("distkP").getDouble(0));
    // distancePID.setD(LLTable.getEntry("distkD").getDouble(0));
    // double distAdjust = MathUtil.clamp(distancePID.calculate(ty, initTY), -0.5, 0.5);
    // LLTable.getEntry("distAdjust").setDouble(distAdjust);
    
    // leftCommand += distAdjust;
    // rightCommand += distAdjust;

    /* turn to face the target */ 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    double steeringAdjust = 0;
    steeringAdjust = MathUtil.clamp(anglePID.calculate(tx, 0), -0.8, 0.8);
    
    leftCommand -= steeringAdjust;
    rightCommand += steeringAdjust;
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
