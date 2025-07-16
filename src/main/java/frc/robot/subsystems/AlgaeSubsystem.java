// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AlgaeCommand;



public class AlgaeSubsystem extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(Constants.AlgaeConstants.intakeID ,  RobotContainer.MainBus)  ;
  TalonFXS wristMotor =  new TalonFXS(Constants.AlgaeConstants.wristID,  RobotContainer.MainBus) ;
  DigitalInput algaeSensor = new DigitalInput(AlgaeConstants.algaeSwitch);

  CANcoder encoder = new CANcoder(46, RobotContainer.MainBus);
  double wristSetpoint = encoder.getAbsolutePosition().getValueAsDouble();
  PIDController wristPID = new PIDController(5.0, 0, 0.000006); //I hate PID loops

  boolean algaeNotRead = false;

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
      //The algae wrist has its own independent encoder (canCoder) so that
      //we dont need to re-zero the arm each time, and save some space



      //DEBUG VALUES  
      SmartDashboard.putNumber ("Wrist Encoder", encoderVal)                ;
      SmartDashboard.putNumber ("Wrist Setpoint", wristSetpoint)            ;
      SmartDashboard.putBoolean("Wrist At Position", wristPID.atSetpoint()) ;
      SmartDashboard.putNumber ("Wrist error", wristPID.getError())         ;
      SmartDashboard.putNumber ("Wrist Home", AlgaeConstants.positions.home);
      SmartDashboard.putNumber ("Wrist Speed", wristMotor.get())            ;
      //SmartDashboard.putBoolean("Wrist ", algaeNotRead)

      wristPID.setSetpoint(wristSetpoint); 
      double bottomPos;



      //Changes the min value of the wrist depending on how high the elevator works.
      if(RobotContainer.m_ElevatorSubsystem.motorRight.getPosition().getValueAsDouble() < AlgaeConstants.ElevatorSafetyPos)
      {
        bottomPos = AlgaeConstants.positions.bottomL1;
      }
      else
      {
        bottomPos = AlgaeConstants.positions.bottomPostL1;
      }




      double followSetpoint = MathUtil.clamp(wristSetpoint, AlgaeConstants.positions.top, bottomPos);
      //^ We have a "ghost" setpoint (the original) and then one that tries to follow that setpoint.



      if(!wristPID.atSetpoint()) //if not close to the setpoint
      {
        wristMotor.set(MathUtil.clamp( 
          wristPID.calculate(encoderVal, followSetpoint), 
         -0.1, 0.1)); //Probably dont exceed 0.4 lol, broke a gearbox :(
      }
      else{
        wristMotor.set(0);
      }

      if(readSensor()) //Cool LED indicator
      {
        RobotContainer.m_LedSubsystem.setTeal(); 
        algaeNotRead = false; 
       }
       else
       {
        if(!algaeNotRead)
        {
          algaeNotRead = true;
          RobotContainer.m_LedSubsystem.setRed();
        }
       } 
    }
  

    public void runGrip(double speed) 
    {
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
        setPosition(Constants.AlgaeConstants.positions.home);
    }
    public void gotoOut()
    {
      setPosition(Constants.AlgaeConstants.positions.grabbing);
    }
    public void gotoClear()
    {
      setPosition(Constants.AlgaeConstants.positions.wristUpPos);
    }

    public void gotoHome()
    {
      setPosition(Constants.AlgaeConstants.positions.home);
    }

    public void gotoFloorGrab()
    {
      setPosition(Constants.AlgaeConstants.positions.floorGrab);
    }

    public double getTarget()
    {
      return wristSetpoint;
    }
    public boolean readSensor(){
      return !algaeSensor.get();
    }
}