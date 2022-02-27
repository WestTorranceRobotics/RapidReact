// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class RunLoader extends CommandBase {
  Loader mLoader;
  /** Creates a new RunLoader. */
  public RunLoader(Loader loader) {
    mLoader = loader;

    addRequirements(mLoader);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RUNNINNG");
    mLoader.runLoader();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLoader.stopLoader();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
