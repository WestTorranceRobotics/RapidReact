// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandGroups.CompletelyOuttake;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.driveTrain.DriveDistanceWithVisionTakeover;
import frc.robot.commands.driveTrain.TurnToAngleWithVisionTakeover;
import frc.robot.commands.driveTrain.TurnToDirection;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeForTime;
import frc.robot.commands.loader.SeeBallRunLoader;
import frc.robot.commands.loader.SeeBallRunLoaderLonger;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NopeBallAuto extends SequentialCommandGroup {
  /** Creates a new NopeBallAuto. */
  public NopeBallAuto(DriveTrain driveTrain, Intake intake, Loader loader, Shooter shooter) {
    addCommands(
      new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter),
      // turn to first blue ball
      new TurnToDirection(driveTrain, 50)
      // new DriveDistance(driveTrain, 35, 0.65),
      // new TurnToAngleWithVisionTakeover(driveTrain, 1),
      // new ParallelDeadlineGroup(
      //   new DriveDistanceWithVisionTakeover(driveTrain),
      //   new RunIntake(intake)
      // ),

      // // leap for the ball
      // new ParallelDeadlineGroup(
      //   new DriveDistance(driveTrain, 18, 0.65),
      //   new RunIntake(intake),
      //   new SeeBallRunLoader(loader)
      // ),

      // // wait at the first blue ball a bit
      // new ParallelDeadlineGroup(
      //   new RunIntakeForTime(intake, 0.5),
      //   // new SeeBallRunLoader(loader)
      //   new SeeBallRunLoader(loader)
      // ),

      // // turn towards second ball while also intaking to ensure that ball is inside loader
      // new ParallelDeadlineGroup(
      //   new TurnToDirection(driveTrain, 230),
      //   new ParallelCommandGroup(
      //     new RunIntake(intake),
      //     new SeeBallRunLoader(loader)
      //   )
      // ),

      // new DriveDistance(driveTrain, 35, 0.65),

      // // turn to second ball
      // new TurnToAngleWithVisionTakeover(driveTrain, 1),
      // new ParallelDeadlineGroup(
      //   new DriveDistanceWithVisionTakeover(driveTrain),
      //   new RunIntake(intake)
      // ),

      // // leap for the ball
      // new ParallelDeadlineGroup(
      //   new DriveDistance(driveTrain, 18, 0.65),
      //   new RunIntake(intake),
      //   new SeeBallRunLoader(loader)
      // ),

      // // wait at the  ball a bit
      // new ParallelDeadlineGroup(
      //   new RunIntakeForTime(intake, 0.5),
      //   new SeeBallRunLoader(loader)
      // ),

      // new TurnToDirection(driveTrain, 280),

      // new CompletelyOuttake(intake, loader, 0.4)
    );
  }
}
