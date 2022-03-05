// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cg;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.i.ReverseIntake;
import frc.robot.commands.load.ReverseLoader;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveBallFromShooterToIntake extends ParallelCommandGroup {
  /** Creates a new MoveBallFromShooterToIntake. */
  public MoveBallFromShooterToIntake(Intake intake, Loader loader) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ReverseIntake(intake), new ReverseLoader(loader));
  }
}
