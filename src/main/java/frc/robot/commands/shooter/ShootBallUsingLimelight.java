// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ShootBallUsingLimelight extends ParallelCommandGroup {
  /** Creates a new ShootBallUsingLimelight. */
  public ShootBallUsingLimelight(DriveTrain driveTrain, Shooter shooter, int rpm) {
    super(
      new DriveToCorrectRangeAndAlignWithLL(driveTrain),
      new ShootBallBasedOnRPM(shooter, rpm)
    );
  }
}

