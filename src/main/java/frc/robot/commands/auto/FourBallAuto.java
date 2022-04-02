// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.driveTrain.DriveDistanceWithVisionTakeover;
import frc.robot.commands.driveTrain.DriveUntilSeeBall;
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
import frc.robot.commands.shooter.ShootUsingLQRDistanceFunction;
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
  public FourBallAuto(DriveTrain driveTrain, Intake intake, Loader loader, Shooter shooter, boolean isRed) {
    addCommands(
      new InstantCommand(() -> NetworkTableInstance.getDefault().getTable("Vision").getEntry("isRed").setBoolean(isRed)),
      new InstantCommand(driveTrain::resetGyro),
      // original two ball auto
      new DeployIntake(intake),
      // drive while continuously intaking, stop when finished driving
      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, 60, 0.70),
        //new SeeBallRunLoader(loader),
        new RunIntakeUntilProxSee(intake, loader)
      ),
      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        new ShootingTwoBallsUsingLQR(shooter, loader, 3800, false),
        //new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500),
        new StayOnTarget(driveTrain),
        // new StopIntake(intake)
        new RunIntake(intake)
      ),
      new TurnToDirection(driveTrain, 3), // turn towards player station // 6 at vitruvian
      
      new RunIntakeForTime(intake, 0.5),

      new DriveDistance(driveTrain, 60, 1),
      new DriveDistance(driveTrain, 50, 0.90),

      // new TurnToAngleWithVisionTakeover(driveTrain, 1),
      new ParallelDeadlineGroup(
        new DriveDistanceWithVisionTakeover(driveTrain),
        new RunIntake(intake)
      ),

      // make a leap for the ball
      // new ParallelDeadlineGroup(
      //   new DriveUntilSeeBall(driveTrain, loader),
      //   new RunIntake(intake)
      // ),
      new ParallelRaceGroup(
        new RunIntakeForTime(intake, 2.0),
        new DriveUntilSeeBall(driveTrain, loader)
      ),

      // wait at human player station and run intake continuously
      new ParallelDeadlineGroup(
        new RunIntakeForTime(intake, 1.5),
        // new SeeBallRunLoader(loader)
        new SeeBallRunLoaderLonger(loader)
      ),

      // coming back 

      new ParallelDeadlineGroup(
        new DriveDistance(driveTrain, -160, 1),
        // run loader and intake for one second
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new Wait(0.5),
            new RunIntakeUntilProxSee(intake, loader)
          ),
          new ParallelDeadlineGroup(
            new Wait(0.6), 
            new RunLoader(loader, -0.25)
          )
        )
      ),
      new TurnToDirection(driveTrain, 6),

      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        // new ShootingTwoBallsUsingLQR(shooter, loader, 3500, true),
        new RunIntakeForTime(intake, 4.0),
        new ShootUsingLQRDistanceFunction(shooter),
        new StayOnTarget(driveTrain),
        new SequentialCommandGroup(
          new Wait(1.0),
          new RunLoader(loader, -0.4)
        )
      )
    );
  }
}
