// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.driveTrain.TurnToAngle;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShootOneBallUsingDirectPower;
import frc.robot.commands.shooter.StayOnTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallsNearTarmac extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */
  public ThreeBallsNearTarmac(DriveTrain driveTrain, Intake intake, Loader loader, Shooter shooter) {
    addCommands(
      new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter),
      new DriveDistance(driveTrain, -36, 0.6),
      new TurnToAngle(driveTrain, 90),
      new DriveDistance(driveTrain, 72, 0.6),
      new TurnToAngle(driveTrain, -45),
      new ParallelDeadlineGroup(
        //new ShootOneBallUsingDirectPower(shooter, loader),
        new StayOnTarget(driveTrain),
        new RunIntake(intake)
      )
    );
  }
}
