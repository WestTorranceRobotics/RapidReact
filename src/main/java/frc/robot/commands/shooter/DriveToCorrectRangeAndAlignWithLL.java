// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class DriveToCorrectRangeAndAlignWithLL extends CommandBase {
  private DriveTrain subsystem;

  private PIDController anglePID;
  private double kP = 0.1945392;
  private double kI = 0;
  private double kD = 0.00323242;

  private PIDController distancePID;
  private double kPDist = 0.07;
  private double kIDist = 0.09;
  private double kDDist = 0;
  // Timer timer = new Timer();
  private double initTY;

  private NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("LLPID");

  /** Creates a new TurnToAngleUsingLimelight. */
  public DriveToCorrectRangeAndAlignWithLL(DriveTrain subsystem) {
    this.subsystem = subsystem;
    anglePID = subsystem.getAngleController();
    distancePID = subsystem.getDistanceController();
    addRequirements(this.subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // subsystem.setP(kP);
    anglePID.setP(kP);
    anglePID.setI(kI);
    anglePID.setD(kD);
    anglePID.setSetpoint(0);
    // distancePID.setP(kPDist);
    // subsystem.enablePID();

    NetworkTableInstance.getDefault().getTable("rpi").getEntry("aimbot").setDouble(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0.0);

    initTY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    subsystem.setAutomatic(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftCommand = 0;
    double rightCommand = 0;

    /* drive to correct distance from target */
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // // System.out.println("ty: " + ty);
    distancePID.setP(LLTable.getEntry("distkP").getDouble(0));
    distancePID.setD(LLTable.getEntry("distkD").getDouble(0));
    double distAdjust = MathUtil.clamp(distancePID.calculate(ty, initTY), -0.5, 0.5);
    LLTable.getEntry("distAdjust").setDouble(distAdjust);

    // // izone because there is no built-in izone function for PIDController
    // if (Math.abs(ty) <= 10) {
    //   if (distancePID.getI() == 0.0) {
    //     distancePID.setI(kIDist);
    //   }
    // }
    // else {
    //   distancePID.reset();
    //   distancePID.setI(0);
    // }
    
    leftCommand += distAdjust;
    rightCommand += distAdjust;

    /* turn to face the target */ 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

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

    // this is simply a minimum command 
    double steeringAdjust = 0;
    // if (Math.abs(ty) <= 3) {
    //   steeringAdjust = MathUtil.clamp(kP * tx, -0.35, 0.35);
    // }
    // else {
    //   steeringAdjust = Math.signum(tx) * 0.05;
    // }
    // steeringAdjust = MathUtil.clamp(kP * tx, -0.65, 0.65);
    
    // add graph for steeringAdjust, you need the period of steeringAdjust https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    // commit changes for laptop 2 on test branch
    anglePID.setP(LLTable.getEntry("anglekP").getDouble(0));
    anglePID.setD(LLTable.getEntry("anglekD").getDouble(0));
    steeringAdjust = MathUtil.clamp(anglePID.calculate(tx, 0), -0.7, 0.7);
    LLTable.getEntry("steeringAdjust").setDouble(steeringAdjust);
    
    // leftCommand -= steeringAdjust;
    // rightCommand += steeringAdjust;

    // System.out.println("tx: " + tx + " |\t\t " + "ty: " + ty);
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

/*
There are two pid controllers used in this command: one for the alignment (anglePID) and one for driving to the correct range (distPID). 

These are the problems so far:
  -If the distance is correct, the proportional angle constant is not enough for the Angle PID
    to even move the drive train. 

    I think that to correct this, we do not clamp the angle pid output and instead change
    the kP value to something that makes sense. 
    We also have to change the integral value again. 
    However, I believe that the distance pid code is fine. The clamping for that makes sense.
*/
