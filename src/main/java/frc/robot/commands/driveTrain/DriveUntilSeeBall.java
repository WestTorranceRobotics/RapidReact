// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import java.security.KeyStore.LoadStoreParameter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Loader;

public class DriveUntilSeeBall extends CommandBase {
  DriveTrain mDriveTrain;
  Loader mLoader;
  boolean isDone = false;
  /** Creates a new DriveUntilSeeBall. */
  public DriveUntilSeeBall(DriveTrain driveTrain, Loader loader) {
    mDriveTrain = driveTrain;
    mLoader = loader;
    addRequirements(mDriveTrain,mLoader);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("\tdriving until see ball");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveTrain.tankDrive(0.7, 0.7);
    System.out.println("driving until see ball");
    if(mLoader.seeBall()){
      System.out.println("DONE with driving until see ball");
      isDone = true;
      mDriveTrain.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = false;
    mDriveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
