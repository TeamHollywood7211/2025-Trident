// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class AlgaeSubsystem extends SubsystemBase {

  TalonFXS intakeMotor = new TalonFXS(Constants.AlgaeConstants.intakeID, RobotContainer.MainBus);
  static TalonFXS wristMotor =  new TalonFXS(Constants.AlgaeConstants.wristID, RobotContainer.MainBus);

    double wristEncoder = wristMotor.getPosition().getValueAsDouble();
    static double wristSetpoint = wristMotor.getPosition().getValueAsDouble(); 

    PIDController wristPID = new PIDController(0.03, 0, 0.005);

    TalonFXSConfiguration config = new TalonFXSConfiguration();
    
    public AlgaeSubsystem(){}
    public Command exampleMethodCommand() {
  
      return runOnce(
          () -> {
  
          });
    }

    public boolean exampleCondition() {
      // Query some boolean state, such as a digital sensor.
      return false;
    }
  
    @Override
    public void periodic() {
      wristEncoder = wristMotor.getPosition().getValueAsDouble(); //20
  
      wristMotor.set(MathUtil.clamp(
        wristPID.calculate(wristEncoder, wristSetpoint),
       -1, 1));
  
  
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
    public void runGrip(double speed)
    {
      intakeMotor.set(speed);
    }
  
    public static void addPosition(double val)
    {
      wristSetpoint += val;
    }
    public static void setPosition(double val)
    {
      wristSetpoint = val;
    }



    public void gotoIn()
    {
        setPosition(0);
    }
    public void gotoOut()
    {
      setPosition(46);
    }

  








  /* 
  public void runGripIn(double speed){
    IntakeMotor1.set(-1 * speed);
  }
  public void runGripOut(double speed){
    IntakeMotor1.set(speed);
  }
  public void stopGrip(){
    IntakeMotor1.set(0);
  }*/

}