// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.loader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Loader;

public class SeeBallRunLoaderLonger extends CommandBase {
  private Loader mLoader;
  private boolean isFinished = false;
  private boolean seeingBall = false;
  Timer timer;
  /** Creates a new SeeBallRunLoader. */
  public SeeBallRunLoaderLonger(Loader loader) {
    mLoader = loader;
    timer = new Timer();

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
      mLoader.runLoader(-0.4);
      timer.start();
    }
    else{
      
      if (timer.hasElapsed(0.2)) {
        mLoader.stopLoader();
      }
      
      //isFinished = true;
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
