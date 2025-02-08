// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;
  private final CommandXboxController m_controller;


  public IntakeCommand(IntakeSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    if(m_controller.getLeftTriggerAxis() > 0.1)
    {
      m_subsystem.setMotorSpeed(0.5);
    }
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
