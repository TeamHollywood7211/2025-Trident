// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  
  Servo climberServo = new Servo(2);
  public ClimberSubsystem() {}

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

  }
  public void servoEngage(){
    climberServo.set(0);
  }

  @Override
  public void simulationPeriodic() {

  }

  public void servoOpen()
  {
    climberServo.setAngle(ClimberConstants.servoOpen); //servo open 
  }

  

  public void servoClose()
  {
    climberServo.setAngle(0);
  }
}