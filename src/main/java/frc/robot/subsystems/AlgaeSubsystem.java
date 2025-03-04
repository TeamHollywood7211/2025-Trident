// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
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



public class AlgaeSubsystem extends SubsystemBase {
  TalonFXS intakeMotor = new TalonFXS(AlgaeConstants.intakeID, RobotContainer.MainBus)  ;
  TalonFXS wristMotor =  new TalonFXS(AlgaeConstants.wristID, RobotContainer.MainBus) ;
  DigitalInput algaeSensor = new DigitalInput(AlgaeConstants.algaeSwitch);
  //DigitalInput encoder = new DigitalInput(1);
  DutyCycleEncoder encoder = new DutyCycleEncoder(1);

  double wristEncoder = wristMotor.getPosition().getValueAsDouble();
  double wristSetpoint = wristMotor.getPosition().getValueAsDouble(); 
  PIDController wristPID = new PIDController(0.03, 0, 0.005);
  boolean algaeNotRead = false;
    //TalonFXSConfiguration config = new TalonFXSConfiguration();
    
    public AlgaeSubsystem(){}
    public Command exampleMethodCommand() {
  
      return runOnce(
          () -> {
  
          });
    }

  
    @Override
    public void periodic() {
      //var wristSim = wristMotor.getSimState();
      wristEncoder = wristMotor.getPosition().getValueAsDouble(); //20

      SmartDashboard.putNumber("Encoder", encoder.get());
      
      wristMotor.set(MathUtil.clamp(
        wristPID.calculate(wristEncoder, wristSetpoint),
       -1, 1));


       if(readSensor())
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
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void runGrip(double speed)
    {
      //System.out.println(speed);
      intakeMotor.set(MathUtil.clamp(speed, -0.5, 0.5)); 
    }
  
    public void addPosition(double val)
    {
      wristSetpoint += val;
    }
    public void setPosition(double val)
    {
      wristSetpoint = val;
    }
    public double getPosition()
    {
      return wristMotor.getPosition().getValueAsDouble();
    }


    public void gotoIn()
    {
        setPosition(0);
    }
    public void gotoOut()
    {
      setPosition(AlgaeConstants.positions.grabbing);
    }
    public double getTarget()
    {
      return wristSetpoint;
    }
    
    public boolean readSensor(){
      return !algaeSensor.get();
    }



}