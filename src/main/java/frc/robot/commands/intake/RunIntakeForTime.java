// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeForTime extends CommandBase {
  private Intake intake;
  private Timer timer;
  private double timeToWait;
  private boolean isDone;
  /** Creates a new RunIntakeForTime. */
  public RunIntakeForTime(Intake intake, double timeToWait) {
    this.intake = intake;

    timer = new Timer();
    this.timeToWait = timeToWait;
    isDone = false;
    System.out.println("TIMER CREATED");
    // Use addRequirements() here to declare subsystem dependencies.
  }
// Called when the command is initially scheduled.
@Override
public void initialize() {
  timer.reset();
  timer.start();
  isDone = false;
  System.out.println("TIMER INIT");
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  intake.runIntake();
  System.out.println("started run intake for time\t" + timer.hasElapsed(0.2));
  System.out.println(timer.hasElapsed(timeToWait));

  if (timer.hasElapsed(timeToWait)) {
    isDone = true;
    intake.stopIntake();
    System.out.println("DONE");
  }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  timer.stop();
  intake.stopIntake();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return isDone;
}
}
