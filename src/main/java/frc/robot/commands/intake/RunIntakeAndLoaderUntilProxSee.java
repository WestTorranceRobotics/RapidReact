// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;

public class RunIntakeAndLoaderUntilProxSee extends CommandBase {
  Intake mIntake;
  Loader mLoader;
  boolean isDone = false;

  /** Creates a new RunIntakeUntilProxSee. */
  public RunIntakeAndLoaderUntilProxSee(Intake intake, Loader loader) {
    mLoader = loader;
    mIntake = intake;
    addRequirements(mIntake, mLoader);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!mLoader.seeBall()){
      mIntake.RunIntake();
      mLoader.runLoader(-0.3);
      if(mLoader.seeBall()){
        mIntake.stopIntake();
        mLoader.stopLoader();
        isDone = true;
      }
    }
    else{
      mIntake.stopIntake();
      mLoader.stopLoader();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = false;
    mIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
