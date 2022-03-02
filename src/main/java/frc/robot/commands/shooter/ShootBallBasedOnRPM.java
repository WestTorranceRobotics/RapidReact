// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootBallBasedOnRPM extends CommandBase {

  private Shooter bShooter;
  private double rpm;
 //private Loader loader;

  /** Creates a new ShootBallBasedOnRPM. */

  //add Loader loader in ()
  public ShootBallBasedOnRPM(Shooter shooter, double rpm) {
   
    this.bShooter = shooter;
    this.rpm = rpm;
//private Loader loader;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //loader.ballIntaked(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double networkRPM = NetworkTableInstance.getDefault().getTable("Vision").getEntry("rpm").getDouble(0);
    bShooter.setVelocity(networkRPM);
    // bShooter.setVelocity(rpm);
    // bShooter.setPower(-0.4);

    // double error = bShooter.getVelocity() - rpm;
    // double comp = -1.2e-5 * Math.pow(error,3);
    // if (Math.abs(comp) > 0.2) {
    //   comp = Math.signum(comp) * 2;
    // }
    // bShooter.directVolts(0.147 + 0.0015538 * rpm );

    /* if (shooter.getVelocity() >= rpm-50 && loader.getAppliedOutput() == 0) {
      loader.runBelt(RobotMap.LoaderMap.runLoaderSpeed);
      shooter.atSpeed(true); */

      /* if (shooter.atSpeed()) {
        shooter.currentWatch(RobotMap.ShooterMap.lineShootRPM);
       }
        */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // bShooter.setVelocity(0);
    bShooter.setPower(0);
    bShooter.atSpeed(false);
   //loader.stopBelt();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return false;
  }
}