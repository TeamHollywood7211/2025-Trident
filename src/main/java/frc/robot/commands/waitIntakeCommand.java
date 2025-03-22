// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class waitIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;
  private int m_state;
  private boolean finished;

  

  public waitIntakeCommand(CoralSubsystem coral) {
    m_coral = coral;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_state = 0;
    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Auto Intake State", m_state);
    if(m_state == 0)
    {
      m_coral.setSpeed(-0.3);
      if(m_coral.getRange() < CoralConstants.coralInRange)
      {
        m_state = 1;
      }
    }
    if(m_state == 1)
    {
      m_coral.setSpeed(-0.2);
      if(m_coral.getRange() > CoralConstants.coralInRange)
      {
        m_state = 2;
      }
    }
    if(m_state == 2)
    {
      m_coral.setSpeed(0.1);
      if(m_coral.getRange() < CoralConstants.coralInRange)
      {
        m_coral.setSpeed(0);
        m_state = 0;
        finished = true;
      }
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
