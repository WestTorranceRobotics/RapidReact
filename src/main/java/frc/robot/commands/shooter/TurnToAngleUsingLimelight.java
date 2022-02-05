// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleUsingLimelight extends CommandBase {
  private DriveTrain subsystem;

  private PIDController anglePID;
  private double kP = 0.052;
  private double kI = 0;
  private double kD = 0;

  private PIDController distancePID;
  private double kPDist = 0.07;
  private double kIDist = 0.09;
  private double kDDist = 0;

  /** Creates a new TurnToAngleUsingLimelight. */
  public TurnToAngleUsingLimelight(DriveTrain subsystem) {
    this.subsystem = subsystem;
    anglePID = subsystem.getAngleController();
    distancePID = subsystem.getDistanceController();
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setP(kP);
    anglePID.setP(kP);
    anglePID.setI(kI);
    anglePID.setD(kD);
    anglePID.setSetpoint(0);
    distancePID.setP(kPDist);
    // subsystem.enablePID();
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

    /* drive to correct distance from target */
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // System.out.println("ty: " + ty);
    
    double tySteeringAdjust = MathUtil.clamp(distancePID.calculate(ty, 0), -0.7, 0.7);

    // izone because there is no built-in izone function for PIDController
    if (Math.abs(ty) <= 10) {
      if (distancePID.getI() == 0.0) {
        distancePID.setI(kIDist);
      }
    }
    else {
      distancePID.reset();
      distancePID.setI(0);
    }
    leftCommand -= tySteeringAdjust;
    rightCommand += tySteeringAdjust;

    /* turn to face the target */ 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // System.out.println("tx: " + tx);
    // double steeringAdjust = Math.signum(tx) * Math.min(Math.abs(subsystem.getP() * tx), 0.45);

    // attempt with built-in pid controller
    // izone because there is no built-in izone function for PIDController
    // if (Math.abs(tx) <= 10) {
    //   if (anglePID.getI() == 0.0) {
    //     anglePID.setI(kI);
    //   }
    // }
    // else {
    //   anglePID.reset();
    //   anglePID.setI(0);
    // }

    // // anglePID.setD(kD);
    double steeringAdjust = MathUtil.clamp(anglePID.calculate(tx, 0), -0.05, 0.05);
    // if (Math.abs(ty) <= 1.5) {
    //   leftCommand -= steeringAdjust * 3.5;
    //   rightCommand -= steeringAdjust * 3.5;
    // }
    // else {
    //   leftCommand -= steeringAdjust;
    //   rightCommand -= steeringAdjust;
    // }
    leftCommand -= steeringAdjust;
      rightCommand -= steeringAdjust;
    

    System.out.println("tx: " + tx + " |\t\t " + "ty: " + ty);

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
