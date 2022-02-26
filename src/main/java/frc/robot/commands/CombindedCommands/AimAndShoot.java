// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CombindedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.VisionProcessing.visiondriving;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShoot extends ParallelCommandGroup {
  /** Creates a new AimAndShoot. */
  public AimAndShoot(DriveTrain driveTrain, Loader loader, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new visiondriving(driveTrain), new RunShooterAndIntakeToShooter(loader, intake, shooter, RobotMap.ShooterMap.shooterPowerLong));
  
  }
}
