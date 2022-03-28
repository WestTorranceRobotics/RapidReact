// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.driveTrain.DriveDistanceWithVisionTakeover;
import frc.robot.commands.driveTrain.Print;
import frc.robot.commands.driveTrain.TurnToAngle;
import frc.robot.commands.driveTrain.TurnToAngleWithVisionTakeover;
import frc.robot.commands.driveTrain.TurnToDirection;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeAndLoaderUntilProxSee;
import frc.robot.commands.intake.RunIntakeForTime;
import frc.robot.commands.intake.RunIntakeUntilProxSee;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.loader.RunLoader;
import frc.robot.commands.loader.SeeBallRunLoader;
import frc.robot.commands.loader.SeeBallRunLoaderLonger;
import frc.robot.commands.shooter.ShootOneBallUsingDirectPower;
import frc.robot.commands.shooter.ShootingTwoBallsUsingLQR;
import frc.robot.commands.shooter.StayOnTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuto extends SequentialCommandGroup {
  /** Creates a new FourBallAuto. */
  public FourBallAuto(DriveTrain driveTrain, Intake intake, Loader loader, Shooter shooter) {
    System.out.println("4 Ball Auto");
    addCommands(
      new InstantCommand(driveTrain::resetGyro),
      // original two ball auto
      new DeployIntake(intake),
      // drive while continuously intaking, stop when finished driving
      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, 75, 0.65),
        //new SeeBallRunLoader(loader),
        new RunIntakeUntilProxSee(intake, loader)
      ),
      new TurnToDirection(driveTrain, -5),
      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        new ShootingTwoBallsUsingLQR(shooter, loader, 3400, false),
        //new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500),
        new StayOnTarget(driveTrain),
        // new StopIntake(intake)
        new RunIntake(intake)
      ),
      new TurnToDirection(driveTrain, 6), // turn towards player station
      new DriveDistance(driveTrain, 130, 0.90),
      new TurnToAngleWithVisionTakeover(driveTrain, 1),
      new DriveDistanceWithVisionTakeover(driveTrain),
      // new ParallelDeadlineGroup(
      //   new TurnToAngleWithVisionTakeover(driveTrain, 1),
      //   new RunIntake(intake)
      // ),

      // make a leap for the ball
      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, 12, 0.65),
        new RunIntake(intake),
        new SeeBallRunLoader(loader)
      ),

      // new Wait(1.5),
      // wait at human player station and run intake continuously
      new ParallelDeadlineGroup(
        new RunIntakeForTime(intake, 2),
        // new SeeBallRunLoader(loader)
        new SeeBallRunLoaderLonger(loader)
      ),

      // coming back
      new TurnToDirection(driveTrain, 6), 

      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, -160, 1),
        // run loader and intake for one second
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new Wait(0.5),
            // new RunLoader(loader, -0.15)),
            new RunIntakeUntilProxSee(intake, loader)
            // new RunIntake(intake)
          ),
          new ParallelDeadlineGroup(
            new Wait(0.7), 
            new RunLoader(loader, -0.25)
          )
        )
      ),

      // shoot while continuously aiming and intaking, stop when finished shooting
      new RunIntakeForTime(intake, 1.0),
      new ParallelDeadlineGroup(
        new ShootingTwoBallsUsingLQR(shooter, loader, 3500, true),
        new StayOnTarget(driveTrain),
        new RunIntake(intake)
      )
    );
  }
}
