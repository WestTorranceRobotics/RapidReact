// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.intake.RunIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveOffAimAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoDriveOffAimAndShoot. */
  public AutoDriveOffAimAndShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ParallelDeadlineGroup(
      //   new DriveDistance(driveTrain, 200, 0.6),
      //   new RunIntake(intake)),
      // new ParallelDeadlineGroup(
      //     new ShootOneBallUsingDirectPower(shooter, loader),
      //     new StayOnTarget(driveTrain),
      //     new MoveBallFromIntakeToShooter(loader, intake)
      //   )
    );
  }
}
