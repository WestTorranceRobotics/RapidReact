// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.commandGroups.RunShooterAndIntakeToShooter;
import frc.robot.commands.driveTrain.DriveDistance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveOffAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoDriveOffAndShoot. */
  public AutoDriveOffAndShoot(DriveTrain driveTrain, Loader loader, Intake intake, Shooter shooter) {
    addCommands(new DriveDistance(driveTrain, 200, 0.6),
    new RunShooterAndIntakeToShooter(loader, intake, shooter, RobotMap.ShooterMap.shooterPowerLong));
  }
  
}
