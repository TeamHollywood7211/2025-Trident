// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeSubsystem m_algae;
  
  double algaePos;
  double elevatorPos;
  boolean finished = false;
  double overrideSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristMoveCommand(AlgaeSubsystem algae, double alPos) {
    m_algae = algae;
    algaePos = alPos;
    //this.overrideSpeed = overrideSpeed[0];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //m_elevator.setPosition(elevatorPos);
    m_algae.setPosition(algaePos);
    System.out.println("Moving algae wrist...");
    //m_algae.gotoOut();
    //m_algae.setPosition(algaePos);
    finished = true;
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
