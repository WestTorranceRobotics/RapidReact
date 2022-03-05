// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.cg;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shoot.ShootOneBallUsingDirectPower;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunShooterAndIntakeToShooter extends ParallelCommandGroup {
  /** Creates a new RunShooterAndLoader. */
  public RunShooterAndIntakeToShooter(Loader loader, Intake intake, Shooter shooter, double power) {
    
    addCommands(new MoveBallFromIntakeToShooter(loader, intake).deadlineWith(new ShootOneBallUsingDirectPower(shooter, loader)));
  }
}
