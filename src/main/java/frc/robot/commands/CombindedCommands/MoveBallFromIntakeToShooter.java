// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CombindedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Loader.RunLoader;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveBallFromIntakeToShooter extends ParallelCommandGroup {
  /** Creates a new MoveBallFromIntakeToShooter. */
  public MoveBallFromIntakeToShooter(Loader loader, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunLoader(loader), new RunIntake(intake));
  }
}
