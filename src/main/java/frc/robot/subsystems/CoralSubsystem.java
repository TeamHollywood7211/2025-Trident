// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class CoralSubsystem extends SubsystemBase {

  TalonFX intakeMotor = new TalonFX(Constants.CoralConstants.intakeID, RobotContainer.MainBus);
  TalonFXS moverMotor = new TalonFXS(Constants.CoralConstants.moverID, RobotContainer.MainBus);
  PIDController moverPID = new PIDController(0.03, 0, 0.005);
  CANrange rangeSensor = new CANrange(Constants.CoralConstants.canRangeID, RobotContainer.MainBus);

  double moverEncoder = moverMotor.getPosition().getValueAsDouble();
  double moverSetpoint = moverMotor.getPosition().getValueAsDouble();



  public CoralSubsystem(){ 


   
    
   
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
    moverEncoder = moverMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Mover Setpoint", moverSetpoint);
    moverMotor.set(MathUtil.clamp(
      moverPID.calculate(moverEncoder, moverSetpoint)
    , -1, 1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void runCoral(double speed){
    intakeMotor.set(speed);
  }
  public void stopCoral(){
    intakeMotor.set(0);
  }

  public void addPosition(double val)
  {
    moverSetpoint += val;
  }
  public void setPosition(double val)
  {
    moverSetpoint = val;
  }
  public double getPosition()
  {
    return moverMotor.getPosition().getValueAsDouble();
  }

  
  public void gotoLeft(){
    setPosition(Constants.CoralConstants.positions.left);
  }
  public void gotoRight(){
    setPosition(Constants.CoralConstants.positions.right);
  }
  public void gotoHome(){
    setPosition(0);
  }
  
  public double getRange()
  {
    return rangeSensor.getDistance().getValueAsDouble();
  }
  public boolean pieceInRange()
  {
    if(getRange() < Constants.CoralConstants.coralInRange)
    {
      return true;
    }
    else
    {
      return false;
    }

  }
  
  public void setSpeed(double speed)
  {
    intakeMotor.set(speed);
  }



}