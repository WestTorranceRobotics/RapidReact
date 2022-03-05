// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  //motor for elevator may be victor spx or falcon 500
  private CANSparkMax elevatorMotor;
  private CANSparkMax elevatorTurningLeader;
  private CANSparkMax elevatorTurningFollower;
  private DigitalInput topLimit;
  private DigitalInput bottomLimit;
  private Solenoid breakOff;

  /** Creates a new Elevator. */
  public Elevator() {
    //declaring motor and limits to their can ids
    ElevatorMotor = new CANSparkMax(RobotMap.ElevatorMap.elevatorCANID, MotorType.kBrushless);
    ElevatorTurningLeader = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningLeader, MotorType.kBrushless);
    ElevatorTurningFollower = new CANSparkMax(RobotMap.ElevatorMap.elevatorTurningFollower, MotorType.kBrushless);
    ElevatorTurningLeader.restoreFactoryDefaults();
    ElevatorTurningFollower.restoreFactoryDefaults();
    ElevatorTurningLeader.setInverted(false);
    ElevatorTurningLeader.setInverted(false);
    // TopLimit = new DigitalInput(RobotMap.ElevatorMap.topLimitChannelID);
    // BottomLimit = new DigitalInput(RobotMap.ElevatorMap.bottomLimitChannelID);
    // BreakOff = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.ElevatorMap.elevatorSolenoid);
    //ElevatorTurningFollower.follow(ElevatorTurningLeader);
    // ElevatorMotor.setIdleMode(IdleMode.Brake);

    elevatorMotor.setInverted(true);
    elevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkMax getElevatorMotor(){
    return elevatorMotor;
  }

  public double getElevatorMotorTicks(){
    return elevatorMotor.getEncoder().getPosition();
  }

  /*victorspx requires controlmode value (all options can be seen with ctrl + click on ControlMode)
    typically we use percent output, position is sometimes used for drivedistance, velocity is also sometimes used alongside pid
    most libraries do not ask for a control mode*/

  //lifts up elevator 
  public void liftUp(){
    elevatorMotor.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void turnLiftClockwise(double power){
    ElevatorTurningLeader.set(power);
    ElevatorTurningFollower.set(-power);
  }

  public void turnLiftCounterClockwise(double power){
    ElevatorTurningLeader.set(-power);
    ElevatorTurningFollower.set(power);
  }

  public void stopTurning(){
    ElevatorTurningLeader.set(0);
    ElevatorTurningFollower.set(0);
  }

  //lowers down elevator
  public void liftDown(){
    elevatorMotor.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  public void liftForwards(){
    elevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorMotorUp);
  }

  public void liftBackwards(){
    elevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorMotorDown);
  }

  //stops elevator
  public void setNoPower(){
    elevatorMotor.set(RobotMap.ElevatorMap.elevatorHalt);
    elevatorTurningLeader.set(RobotMap.ElevatorMap.elevatorHalt);
    elevatorTurningFollower.set(RobotMap.ElevatorMap.elevatorHalt);
  }

  //notifies when top is reached
  public boolean hasReachedTopLimit(){
    return topLimit.get();
  }

  //notifies when bottom is reached
  public boolean hasReachedBottomLimit(){
    return bottomLimit.get();
  }
}
