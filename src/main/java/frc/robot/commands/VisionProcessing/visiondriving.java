// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionProcessing;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class visiondriving extends CommandBase {
  /** Creates a new visiondriving. */
  DriveTrain m_driveTrain;
  boolean isFinished = false;
  public visiondriving(DriveTrain driveTrain) {

    m_driveTrain = driveTrain;
    addRequirements(driveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   //NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0);
   // NetworkTableInstance.getDefault().getTable("Vision").getEntry("yposition").getDouble(0);

    System.out.println("HIIII");

    if (NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) > 20) {

      m_driveTrain.tankDrive(-0.45, 0.45);

    }

    else if (NetworkTableInstance.getDefault().getTable("Vision").getEntry("Xposition").getDouble(0) < -20) {

      m_driveTrain.tankDrive(0.45, -0.45);

    }

    else {

      m_driveTrain.tankDrive(0,0);
      isFinished = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
