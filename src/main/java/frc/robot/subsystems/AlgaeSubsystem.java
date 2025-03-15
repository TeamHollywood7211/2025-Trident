// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;



public class AlgaeSubsystem extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(Constants.AlgaeConstants.intakeID, RobotContainer.MainBus)  ;
  TalonFXS wristMotor =  new TalonFXS(Constants.AlgaeConstants.wristID, RobotContainer.MainBus) ;

  CANcoder encoder = new CANcoder(46, RobotContainer.MainBus);

  double wristSetpoint = encoder.getAbsolutePosition().getValueAsDouble();

  PIDController wristPID = new PIDController(5.0, 0, 0.000006);

  

    //TalonFXSConfiguration config = new TalonFXSConfiguration();

    public AlgaeSubsystem(){
      wristPID.setTolerance(0.00001);
    }
    public Command exampleMethodCommand() {
  
      return runOnce(
          () -> {
  
          });
    }
  
    @Override
    public void periodic() {
      
      double encoderVal = encoder.getAbsolutePosition().getValueAsDouble();


      SmartDashboard.putNumber("Wrist Encoder", encoderVal)                ;
      SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint)            ;
      SmartDashboard.putBoolean("Wrist At Position", wristPID.atSetpoint());
      SmartDashboard.putNumber("Wrist error", wristPID.getError());

      wristPID.setSetpoint(wristSetpoint);
      double bottomPos;

      if(RobotContainer.m_ElevatorSubsystem.motorRight.getPosition().getValueAsDouble() < ElevatorConstants.positions.c_low)
      {
        bottomPos = AlgaeConstants.positions.bottomL1;
      }
      else
      {
        bottomPos = AlgaeConstants.positions.bottomPostL1;
      }

      wristSetpoint = MathUtil.clamp(wristSetpoint, AlgaeConstants.positions.top, bottomPos);

      if(!wristPID.atSetpoint())
      {
        wristMotor.set(-MathUtil.clamp(
          wristPID.calculate(encoderVal, wristSetpoint),
         -0.3, 0.3)); //Probably dont exceed 0.4 lol, broke a gearbox :(
      }
    }
  

    public void runGrip(double speed)
    {
      //System.out.println(speed);
      intakeMotor.set(MathUtil.clamp(speed, -0.5, 0.5)); 
    }
  
    public void addPosition(double val)
    {
      wristSetpoint += val/20;
    }
    public void setPosition(double val)
    {
      wristSetpoint = val;
    }
    public double getPosition()
    {
      return encoder.getAbsolutePosition().getValueAsDouble();
    }
    public void gotoIn()
    {
        setPosition(0);
    }
    public void gotoOut()
    {
      setPosition(Constants.AlgaeConstants.positions.grabbing);
    }
    public double getTarget()
    {
      return wristSetpoint;
    }
}