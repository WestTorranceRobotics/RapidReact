// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.shooter.ShootTwoBallsUsingDirectPower;

public class TestShooter extends SubsystemBase {
  private CANSparkMax shootMotorFollower = new CANSparkMax(RobotMap.ShooterMap.ShooterFollowerCANID, MotorType.kBrushless);
  private CANSparkMax shootMotorLeader = new CANSparkMax(RobotMap.ShooterMap.ShooterLeaderCANID, MotorType.kBrushless);
  // private CANPIDController shootPID;
  private boolean atSpeed;
  private int ballsShot = 0;
  private boolean passedBallCurrent = false;
  private static double kSpinupRadPerSec = 3500;

  private final LinearSystem<N1,N1,N1> m_flyWheel = LinearSystemId.identifyVelocitySystem(RobotMap.ShooterMap.kV, RobotMap.ShooterMap.KA);
  
  private final KalmanFilter<N1,N1,N1> m_KalmanFilter = 
  new KalmanFilter<>(
    Nat.N1(), 
    Nat.N1(), 
    m_flyWheel, 
    VecBuilder.fill(1.0), //How accurate we think our model is  
    VecBuilder.fill(0.001), //How accurate we think our encoder data is
    0.020);

  private final LinearQuadraticRegulator<N1,N1,N1> m_controller = 
    new LinearQuadraticRegulator<>(
      m_flyWheel, 
      VecBuilder.fill(17.7), //Velocity Error Tolerance
      VecBuilder.fill(7.0), //Control effort(Voltage) tolerance
      0.020);
      //Values obtained for the LinearQuadraticRegulator were done through characterization.
      //Have not been tested yet. 

  private final LinearSystemLoop<N1,N1,N1> m_loopLeader = 
  new LinearSystemLoop<>(m_flyWheel,  m_controller, m_KalmanFilter, 12.0, 0.020);

  private final LinearSystemLoop<N1,N1,N1> m_loopFollower = 
  new LinearSystemLoop<>(m_flyWheel,  m_controller, m_KalmanFilter, 12.0, 0.020);


  /** Creates a new TestShooter. */
  public TestShooter() {
  //leader
    shootMotorLeader.restoreFactoryDefaults();
    shootMotorLeader.setIdleMode(IdleMode.kCoast);
  //follower
    shootMotorFollower.restoreFactoryDefaults();
    shootMotorFollower.setIdleMode(IdleMode.kCoast);
    shootMotorFollower.follow(shootMotorLeader, true);

  //Space State Controller
    m_loopLeader.reset(VecBuilder.fill(shootMotorLeader.getEncoder().getVelocity()));
    // m_loopFollower.reset(VecBuilder.fill(shootMotorFollower.getEncoder().getVelocity()));
    m_controller.latencyCompensate(m_flyWheel, 0.02, 0.25);
  }

  public LinearSystemLoop<N1,N1,N1> getLinearSystemLoopLeader(){
    return m_loopLeader;
  }

  public LinearSystemLoop<N1,N1,N1> getLinearSystemLoopFollower(){
    return m_loopFollower;
  }

  public void setReferenceVelocity(){
    m_loopLeader.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    //m_loopFollower.setNextR(VecBuilder.fill(kSpinupRadPerSec));
  }

  public void zeroReferenceVelocity(){
    m_loopLeader.setNextR(VecBuilder.fill(0));
    //m_loopFollower.setNextR(VecBuilder.fill(0));
  }

  public double getVelocityLeader(){
    return shootMotorLeader.getEncoder().getVelocity();
  }

  public double getVelocityFollower(){
    return shootMotorFollower.getEncoder().getVelocity();
  }

  public CANSparkMax getShootMotorLeader(){
    return shootMotorLeader;
  }

  public CANSparkMax getShootFollowerLeader(){
    return shootMotorFollower;
  }

  @Override
  public void periodic() {
    //kSpinupRadPerSec = NetworkTableInstance.getDefault().getTable("vision").getEntry("x").getDouble(0);
    // This method will be called once per scheduler run
  }
}
