// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

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
import frc.robot.commands.intake.RunIntakeUntilProxSee;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.loader.RunLoader;
import frc.robot.commands.loader.SeeBallRunLoader;
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
      // original two ball auto
      new DeployIntake(intake),
      // drive while continuously intaking, stop when finished driving
      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, 70, 0.65),
        //new SeeBallRunLoader(loader),
        new RunIntakeUntilProxSee(intake, loader)
      ),
      // new ParallelDeadlineGroup(
      //   new DriveDistance(driveTrain, -40, 0.75),
      //   //new SeeBallRunLoader(loader),
      //   new RunIntakeUntilProxSee(intake, loader)
      // ),
      //new TurnToAngle(driveTrain, -15),
      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        new ShootingTwoBallsUsingLQR(shooter, loader, 4200),
        //new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500),
        new StayOnTarget(driveTrain),
        new StopIntake(intake)
        //new RunIntake(intake)
      ),
      new TurnToAngle(driveTrain, 9),
      new DriveDistance(driveTrain, 140, 0.90),
      new ParallelDeadlineGroup(
        new TurnToAngleWithVisionTakeover(driveTrain, 1),
        new RunIntake(intake)
      ),

      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, 25, 1),
        new RunIntake(intake),
        new SeeBallRunLoader(loader)
      ),

      new ParallelDeadlineGroup(
        new Wait(0.5),
        new RunIntake(intake),
        new SeeBallRunLoader(loader)
      ),

      //new TurnToDirection(driveTrain, -22),
      new TurnToAngle(driveTrain, -15),

      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, -300, 1),
        new ParallelDeadlineGroup(
          new Wait(1),
          new RunLoader(loader, -0.2))
        // new RunIntake(intake)
      ),

      new TurnToAngle(driveTrain, -15),
      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500),
        new StayOnTarget(driveTrain)
        //new RunIntake(intake)
      )

      //-------
      // new DriveOffAimAndShootTwoBalls(driveTrain, intake, loader, shooter),
      // new ParallelDeadlineGroup(
      //   new DriveDistanceWithVisionTakeover(driveTrain),
      //   new RunIntake(intake)
      // ),
      // new ParallelDeadlineGroup( // wait for all balls from human player station to load in
      //   new Wait(2),
      //   new RunIntake(intake),
      //   new RunLoader(loader, -0.2)
      // ),
      // new DriveDistance(driveTrain, 300, 0.7),
      // new ParallelDeadlineGroup(
      //   new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500), // actually shoots two balls
      //   new StayOnTarget(driveTrain),
      //   new RunIntake(intake)
      // )
    );
  }
}
