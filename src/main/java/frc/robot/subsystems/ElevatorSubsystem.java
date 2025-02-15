// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {

  TalonFX motorLeft = new TalonFX(ElevatorConstants.leftMotorID, RobotContainer.MainBus);
  TalonFX motorRight = new TalonFX(ElevatorConstants.rightMotorID, RobotContainer.MainBus);
  PIDController ArmPID = new PIDController(0.03, 0, 0.0005);
  double encoderLeft = motorLeft.getPosition().getValueAsDouble();
  double encoderRight = motorRight.getPosition().getValueAsDouble();
  double ElevatorSetpoint = encoderLeft;

  public ElevatorSubsystem() {

  }


  public Command exampleMethodCommand() {
    return runOnce(
        () -> {

        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    double currentLeftPos = getMotorLeftPosition();
    double currentRightPos = getMotorRightPosition();
    ElevatorSetpoint = MathUtil.clamp(ElevatorSetpoint, 0, 200);
    //SmartDashboard.putNumber("Arm Position", currentPos); 
    //SmartDashboard.putNumber("Arm Target Pos", ArmSetpoint);
    motorLeft.set(MathUtil.clamp(ArmPID.calculate(currentLeftPos, -ElevatorSetpoint), -0.5, 0.5));
    motorRight.set(MathUtil.clamp(ArmPID.calculate(currentRightPos, ElevatorSetpoint), -0.5, 0.5));
    SmartDashboard.putNumber("Elevator Setpoint", ElevatorSetpoint);
    SmartDashboard.putNumber("Elevator Position", currentRightPos) ;
    
    //SmartDashboard.putNumber("Percentage to Pos: ", (currentPos / ArmSetpoint) * 100);
  }

  @Override
  public void simulationPeriodic() {

  }

  public void setPosition(double val)
  {
    ElevatorSetpoint = val;
  }
  public void addPosition(double val)
  {
    ElevatorSetpoint += val;
  }

  public void forceSetSpeed(double speed)
  {
    motorLeft.set(speed);
    motorRight.set(-speed);
  }

  public void gotoAlgae_Highest()
  {
    setPosition(Constants.AlgaeConstants.positions.highest);
    AlgaeSubsystem.gotoOut(); //Sets the Algae arm out
  }
  public void gotoAlgae_High()
  {
    setPosition(Constants.AlgaeConstants.positions.high);
    AlgaeSubsystem.gotoOut();
  }
  public void gotoAlgae_Low()
  {
    setPosition(Constants.AlgaeConstants.positions.low);
    AlgaeSubsystem.gotoOut();
  }
  public void gotoAlgae_Floor()
  {
    gotoHome();
    AlgaeSubsystem.gotoOut();
  }
  public void gotoHome()
  {
    setPosition(3); //Setting a little above zero just in case our zero gets effed up
  }



  public double getMotorLeftPosition()
  {
    return motorLeft.getPosition().getValueAsDouble();
  }
  public double getMotorRightPosition()
  {
    return motorRight.getPosition().getValueAsDouble();
  }



}
