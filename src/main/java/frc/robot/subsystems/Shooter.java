/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
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

//   P: 9.2177e-4
// I: 2.114e-7
// D: 0.03395
 
  public Shooter() {

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

  public boolean active() {
    return shootMotorLeader.getAppliedOutput() != 0;
  }

  public double getCurrent() {
    return shootMotorLeader.getOutputCurrent();
  }

  public void currentWatch() {
    if (shootMotorLeader.getOutputCurrent() >= RobotMap.ShooterMap.ballCurrent && passedBallCurrent == false) {
      passedBallCurrent = true;
      ballsShot = ballsShot + 1;
    } else if (passedBallCurrent == true && shootMotorLeader.getOutputCurrent() < RobotMap.ShooterMap.ballCurrent-7) {
      passedBallCurrent = false;
    }
  }

  public void resetError() {
    shootMotorLeader.getPIDController().setIAccum(0);
    shootMotorFollower.getPIDController().setIAccum(0);
  }
  
  public double getVelocity() {
    // return (shootMotorLeader.getEncoder().getVelocity() / RobotMap.ShooterMap.gearRatio); 
    return (shootMotorLeader.getEncoder().getVelocity()); 
   }
   
   public void setVelocity(double velocity){
    shootMotorLeader.getPIDController().setReference(-velocity, com.revrobotics.CANSparkMax.ControlType.kVelocity);
    shootMotorFollower.getPIDController().setReference(-velocity, com.revrobotics.CANSparkMax.ControlType.kVelocity);
    
   }

   public double getVoltage() {
     return shootMotorLeader.getBusVoltage();
   }
  
   public void atSpeed(boolean atSpeed) {
    this.atSpeed= atSpeed;
  }
  public boolean atSpeed() {
    return this.atSpeed;
  }

  public void setPower(double power) {
    shootMotorLeader.set(-power);
    shootMotorFollower.set(-power);
  }

  public int getBallsShot() {
    return ballsShot;
  }

  public void addBallShot(){
    ballsShot += 1;
  }

  public void resetBallShot(){
    ballsShot = 0;
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
    
    // NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootI").setDouble(0);

  //   shootMotorLeader.getPIDController().setP(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootP").getDouble(0));
  //   shootMotorLeader.getPIDController().setI(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootI").getDouble(0));
  //   shootMotorLeader.getPIDController().setD(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootD").getDouble(0));
  //   shootMotorLeader.getPIDController().setFF(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootF").getDouble(0)); // 0.004
  // //follower
  //   shootMotorFollower.getPIDController().setP(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootP").getDouble(0));
  //   shootMotorFollower.getPIDController().setI(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootI").getDouble(0));
  //   shootMotorFollower.getPIDController().setD(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootD").getDouble(0));
  //   shootMotorFollower.getPIDController().setFF(NetworkTableInstance.getDefault().getTable("Vision").getEntry("shootF").getDouble(0));
  }

  /**
   * @deprecated Unreliable with higher loader speeds at the present
   */
  // public void currentWatch(double targetRPM) {
  //   if (shootMotorLeader.getOutputCurrent() >= RobotMap.ShooterMap.ballCurrent && passedBallCurrent == false) {
  //     passedBallCurrent = true;
  //     ballsShot = ballsShot + 1;
  //   } else if (passedBallCurrent == true && shootMotorLeader.getOutputCurrent() < RobotMap.ShooterMap.ballCurrent-7) {
  //     passedBallCurrent = false;
  //   }
  // }

  // public void directVolts(double volts) {
  //   shootMotorLeader.setVoltage(volts);
  // }

    // public void disablePID() {
  //   shootPID.setD(0);
  //   shootPID.setP(0);
  //   shootPID.setFF(0);
  // }

  // public void enablePID() {
  //   shootPID.setD(RobotMap.ShooterMap.Kd);
  //   shootPID.setP(RobotMap.ShooterMap.Kp);
  //   shootPID.setFF(RobotMap.ShooterMap.Kf);
  // }

  // public void updatePID() {
  //   shootPID.setD(SmartDashboard.getNumber("DSHOOT", RobotMap.ShooterMap.Kd));
  //   shootPID.setP(SmartDashboard.getNumber("PSHOOT", RobotMap.ShooterMap.Kp));
  //   shootPID.setFF(SmartDashboard.getNumber("FSHOOT", RobotMap.ShooterMap.Kf));
  // }

  /**
   * @param targetRPM desired RPM of shooter
   */
  // public void startShooter() {
  //   enablePID();
  //   shootPID.setReference(RobotMap.ShooterMap.lineShootRPM, ControlType.kVelocity);
  // }

  // public void startShooter(double rpm) {
  //   shootPID.setReference(rpm, ControlType.kVelocity);
  // }

  //   public void stopShooter () {
  //     disablePID();
  //    shootMotorLeader.set(0);
  //   }

}