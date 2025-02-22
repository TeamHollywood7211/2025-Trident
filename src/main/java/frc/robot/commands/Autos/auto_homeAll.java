// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_homeAll extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeSubsystem m_algae;
  private final CoralSubsystem m_coral;
  private final ElevatorSubsystem m_elevator;

  boolean algaeMoved = false;
  boolean coralMoved = false;
  boolean finished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public auto_homeAll(CoralSubsystem coral, ElevatorSubsystem elevator, AlgaeSubsystem algae) {
    m_algae = algae;
    m_coral = coral;
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeMoved = false;
    coralMoved = false;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coral.setPosition(0);
    if(Math.abs(m_coral.getPosition()) < 10)
    {
      coralMoved = true;
    }
    if(coralMoved){
      m_algae.setPosition(0);
      if(m_algae.getPosition() > -7)
      {
        algaeMoved = true;
      }
    }
    if(algaeMoved)
    {
      m_elevator.setPosition(Constants.ElevatorConstants.positions.home);
      finished = true;/*                                               */
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
