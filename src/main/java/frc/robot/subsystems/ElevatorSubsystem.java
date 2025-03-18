// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANrange;
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

  TalonFX motorLeft = new TalonFX(ElevatorConstants.leftMotorID, RobotContainer.MainBus)  ; //Sets up the motors
  TalonFX motorRight = new TalonFX(ElevatorConstants.rightMotorID, RobotContainer.MainBus);
  PIDController ArmPID = new PIDController(0.03, 0, 0.0005); //PID for the arm
  double encoderLeft = motorLeft.getPosition().getValueAsDouble(); //Gets the encoder values of the left and right motor
  double encoderRight = motorRight.getPosition().getValueAsDouble();
  double ElevatorSetpoint = encoderRight;
  CANrange rangeSensor = new CANrange(ElevatorConstants.canRangeID, RobotContainer.MainBus);
  boolean elevatorNotRead = false;

  boolean emergencyHome = false;
  
  
  //We will trust the right motor as the main motor as it going up is always positive.
  public ElevatorSubsystem() { 

  }

  @Override
  public void periodic() {
    double currentLeftPos = getMotorLeftPosition(); //Gets positions
    double currentRightPos = getMotorRightPosition();
    ElevatorSetpoint = MathUtil.clamp(ElevatorSetpoint, 0, Constants.ElevatorConstants.positions.max); //Clamps the values for safety


    SmartDashboard.putNumber("ELEVATOR VOLTAGE", motorRight.getMotorVoltage().getValueAsDouble());

    if(intakeClear()) //intakeClear() just checks if the range sensor on the robot sees a note.
    { 
      motorLeft.set(MathUtil.clamp(ArmPID.calculate(currentLeftPos, -ElevatorSetpoint), -1, 1)); //The actual code for the PID loops
      motorRight.set(MathUtil.clamp(ArmPID.calculate(currentRightPos, ElevatorSetpoint), -1, 1));
      SmartDashboard.putNumber("Elevator Setpoint", ElevatorSetpoint); //Sends debug information to SmartDashboard
      SmartDashboard.putNumber("Elevator Position", currentRightPos) ;
    }
    else
    {
      System.out.println("Item in intake!!!");
    }



    if(!intakeClear())
    {
      RobotContainer.m_LedSubsystem.setPurple();
      elevatorNotRead = false;
    }
    else{
      if(!elevatorNotRead)
      {
        RobotContainer.m_LedSubsystem.setRed();
        elevatorNotRead = true;
      }
    }

    /* 
    if(emergencyHome) //DO NOT RUN MOTORS WILL STOP 
    {
      double velocity = motorRight.getVelocity().getValueAsDouble();
      if(Math.abs(velocity) < 0.1)
      {
        motorRight.setPosition(0);
        motorLeft.setPosition(0);
        emergencyHome = true;
      }
      else{
        motorRight.set(-0.2);
        motorLeft.set(0.2);
      }
    }*/
    
  }
  /**
   * SETS the setpoint of this subsystem, use for auto control
   * @param val The value to set to
   */
  public void setPosition(double val) 
  {
    if(intakeClear())
    {
      ElevatorSetpoint = val;
    }
  }
  /**
   * ADDS to the setpoint of this subsystem, use for manual control
   * @param val The value to add to
   */
  public void addPosition(double val)
  {
    ElevatorSetpoint += val; 
  }



  public void gotoHome() 
  {
    setPosition(3); //Setting a little above zero just in case our zero gets effed up
  }



  public double getMotorLeftPosition() //
  {
    return motorLeft.getPosition().getValueAsDouble();
  }
  public double getMotorRightPosition() //
  {
    return motorRight.getPosition().getValueAsDouble();
  }
    /**
   * Gives the range sensor for the Elevator
   */
  public double getRange()
  {
    return rangeSensor.getDistance().getValueAsDouble();
  }

  public double getElevatorPosition()
  {
    SmartDashboard.putNumber("Elevator Pos", getMotorRightPosition());
    return Math.abs(getMotorRightPosition());
  }

  /**
   * Checks if the intake/elevator is clear
   */
  public boolean intakeClear()
  {
    //return true;
    
    if(getRange() < ElevatorConstants.coralRange)
    {
      return false;
    }
    else
    {
      return true;
    } 
  }
}
