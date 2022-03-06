// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class SeeBallRunLoader extends CommandBase {
  private Loader mLoader;
  private boolean isFinished = false;
  /** Creates a new SeeBallRunLoader. */
  public SeeBallRunLoader(Loader loader) {
    mLoader = loader;

    addRequirements(mLoader);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mLoader.seeBall()){
      mLoader.runLoader();
      if(!mLoader.seeBall()){
        mLoader.stopLoader();
        isFinished = true;
      }
    }
    else{
      mLoader.stopLoader();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
    mLoader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}