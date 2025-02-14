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
  TalonFXS wristMotor =  new TalonFXS(Constants.AlgaeConstants.wristID, RobotContainer.MainBus);
  
  

  double wristEncoder = wristMotor.getPosition().getValueAsDouble();
  double wristSetpoint = wristMotor.getPosition().getValueAsDouble(); 

  

  PIDController wristPID = new PIDController(0.03, 0, 0.005);

  TalonFXSConfiguration config = new TalonFXSConfiguration();
  
  public AlgaeSubsystem(){
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {

    return runOnce(
        () -> {

        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    wristEncoder = wristMotor.getPosition().getValueAsDouble();

    wristMotor.set(MathUtil.clamp(wristPID.calculate(wristEncoder, wristSetpoint), -0.5, 0.5));


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void runGrip(double speed)
  {
    intakeMotor.set(speed);
  }

  public void setPosition(double val)
  {
    wristSetpoint += val;
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