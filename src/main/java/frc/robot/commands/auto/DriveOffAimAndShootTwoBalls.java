// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeForTime;
import frc.robot.commands.intake.RunIntakeUntilProxSee;
import frc.robot.commands.shooter.ShootOneBallUsingDirectPower;
import frc.robot.commands.shooter.ShootingTwoBallsUsingLQR;
import frc.robot.commands.shooter.ShootingTwoBallsUsingLQRUsingTime;
import frc.robot.commands.shooter.StayOnTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOffAimAndShootTwoBalls extends SequentialCommandGroup {
  /** Creates a new DrieOffAimAndShootTwoBalls. */
  public DriveOffAimAndShootTwoBalls(DriveTrain driveTrain, Intake intake, Loader loader, Shooter shooter) {
    addCommands(
      new InstantCommand(driveTrain::resetGyro),
      // original two ball auto
      new DeployIntake(intake),
      // drive while continuously intaking, stop when finished driving
      new ParallelDeadlineGroup(
        //new RunIntakeForTime(intake, 2.5),
        new DriveDistance(driveTrain, 75, 0.70),
        new RunIntakeUntilProxSee(intake, loader)
      ),
      new DriveDistance(driveTrain, -30, 0.7),
      // shoot while continuously aiming and intaking, stop when finished shooting
      new ParallelDeadlineGroup(
        new ShootingTwoBallsUsingLQRUsingTime(shooter, loader, 3200, false, 4.50),
        //new ShootOneBallUsingDirectPower(shooter, loader, 0.65, 2500),
        new StayOnTarget(driveTrain),
        // new StopIntake(intake)
        new RunIntake(intake)
      ),
      new DriveDistance(driveTrain, 30, 0.7),
      new InstantCommand(loader::enableProxSensor, loader)
    );
  }
}
