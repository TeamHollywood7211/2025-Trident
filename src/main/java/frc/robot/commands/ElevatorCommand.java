// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;
  private final CommandXboxController m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem elevator, CommandXboxController controller) {
    m_elevator = elevator;
    m_controller = controller;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {}
  @Override
  public void execute() { 
    //MANUAL CONTROL:

    /*
     *  m_algae.runGrip(
      RobotContainer.booleanToDouble(m_controller.leftTrigger().getAsBoolean())
      -
      RobotContainer.booleanToDouble(m_controller.rightTrigger().getAsBoolean())
    );
     */

     m_elevator.addPosition(
      -m_controller.getLeftY()
     );
  }
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
