// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_waitIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;
  private int m_state;
  private boolean finished;
  private boolean reset;

  

  public auto_waitIntake(CoralSubsystem coral) {
    m_coral = coral;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_state = 0;
    this.reset = false;
    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Intake State", m_state);
    if(reset)
    {
      m_state = 0;
    }
    if(m_state == 0) //take in the piece
    {
      m_coral.setSpeed(-0.5);
      if(m_coral.getRange() < CoralConstants.coralInRange)
      {
        m_state = 1;
      }
    }
    if(m_state == 1) //once sensor hit, run intake more but slower till we cant see piece
    {
      m_coral.setSpeed(0.2);
      if(m_coral.getRange() > CoralConstants.coralInRange)
      {
        m_coral.setSpeed(0);
        m_state = 0;
        reset = true;
        finished = true;
      }
    }
    if(m_state == 2)
    {
      m_coral.setSpeed(-0.1);
      if(m_coral.getRange() < CoralConstants.coralInRange)
      {
        m_state = 0;
        reset = true;
        finished = true;
      }
    }
    if(m_state == 4)
    {
      m_coral.setSpeed(0);
      reset = true;
      finished = true;
      m_state = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
