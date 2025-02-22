// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class auto_coralMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;
  private final CoralSubsystem m_coral;
  
  double coralPos;
  double elevatorPos;
  double algaePos;
  boolean finished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public auto_coralMove(ElevatorSubsystem elevator, CoralSubsystem coral, double corPos, double ElPos, double alPos) {
    m_elevator = elevator;
    m_coral = coral;
    coralPos = corPos;
    elevatorPos = ElPos;
    algaePos = alPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_elevator.setPosition(elevatorPos);
    
    RobotContainer.m_AlgaeSubsystem.setPosition(algaePos);
    
    //RobotContainer.m_AlgaeSubsystem.gotoOut(); //Just being safe :)
    //Safety check: Ensures that the algae is passed the safety position
    if((RobotContainer.m_AlgaeSubsystem.getPosition() < (Constants.AlgaeConstants.positions.safety) || (Constants.safetyBypass)))
    {
      // -12 < -11.5
      m_coral.setPosition(coralPos);
      finished = true;
    }
//    
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
