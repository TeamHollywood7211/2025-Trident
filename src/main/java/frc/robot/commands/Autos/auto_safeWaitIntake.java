// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.subsystems.CoralSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_safeWaitIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralSubsystem m_coral;
  boolean gottenPiece = false;
  boolean finished = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public auto_safeWaitIntake(CoralSubsystem coral) {
    m_coral = coral;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gottenPiece = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gottenPiece = false)
    {
      m_coral.runCoral(0.25);
      if(m_coral.pieceInRange())
      {
        gottenPiece = true;
      }
      else
      {
        gottenPiece = false;
      }
    }
    if(gottenPiece)
    {
      m_coral.runCoral(0.05);
      if(!m_coral.pieceInRange())
      {
        m_coral.runCoral(0);
        finished = true;
      }
    }
    
    //m_coral.getRange();


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
