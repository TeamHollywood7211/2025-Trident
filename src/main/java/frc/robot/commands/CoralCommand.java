package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
//import frc.robot.Constants.*;

public class CoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private CoralSubsystem m_coral;
  private CommandXboxController m_controller;
  private CommandXboxController m_bb0;
  private CommandXboxController m_bb1;

  public CoralCommand(CoralSubsystem intakeSubsystem, CommandXboxController bb0, CommandXboxController bb1, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_coral = intakeSubsystem;
    this.m_bb0 = bb0;
    this.m_bb1 = bb1;
    addRequirements(intakeSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  
  public void execute() {
    double power = 1.00;

    if(RobotContainer.m_ElevatorSubsystem.getElevatorPosition() < Constants.ElevatorConstants.positions.c_low)
    {
      power = 0.3;
    }
    m_coral.runCoral(
      


      RobotContainer.booleanToDouble(m_bb0.button(9).getAsBoolean())-RobotContainer.booleanToDouble(m_bb0.button(2).getAsBoolean()) * power
    );
    if(m_bb0.button(9).getAsBoolean())
    {
      RobotContainer.m_LedSubsystem.setRed();
    }


     if(Math.abs(m_controller.getLeftX()) > 0.05 )
     {
      m_coral.addPosition(m_controller.getLeftX() * 0.1);
     }
  
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}