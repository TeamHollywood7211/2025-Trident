// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  
  Servo climberServo = new Servo(2);
  Servo intakeServo = new Servo(0);
  TalonFX climber = new TalonFX(Constants.ClimberConstants.armMotorID, RobotContainer.MainBus);
  double encoder = climber.getPosition().getValueAsDouble();
  double setpoint = encoder;
  PIDController pid = new PIDController(0.03, 0, 0.0005);



  public ClimberSubsystem() {
    intakeServo.setPulseTimeMicroseconds(500); //?
  }


  /*ON CLIMB ENGAGE
  
    - intakeServo switch on
    - Cage thingy goes up

  */
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
    SmartDashboard.putNumber("Intake Servo", intakeServo.getAngle());
    SmartDashboard.putNumber("Climber Servo", climberServo.getAngle());
    SmartDashboard.putNumber("Climber Setpoint", setpoint);

    encoder = climber.getPosition().getValueAsDouble();

    climber.set(MathUtil.clamp(pid.calculate(encoder, setpoint), -1, 1));
  }

  @Override
  public void simulationPeriodic() {

  }

  public void climberEngage()
  {
    intakeServo.setAngle(75);
  }
  public void climberServoHome()
  {
    intakeServo.setAngle(0);
  }

  public void addPosition(double val)
  {
    setpoint += val;
  }

  public void climberRun1()
  {
    setpoint = Constants.ClimberConstants.clOpenPosition;
    climberEngage();
  }
  public void climberRun2()
  {
    setpoint = 0;
  }

}