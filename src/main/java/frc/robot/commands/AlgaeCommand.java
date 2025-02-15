package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeSubsystem;
//import frc.robot.Constants.*;

public class AlgaeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AlgaeSubsystem m_algae;
  private CommandXboxController m_controller;
  public AlgaeCommand(AlgaeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_algae = intakeSubsystem;
    addRequirements(intakeSubsystem);
  } 

  //public IntakeCommand(IntakeSubsystem intakeSubsystem, CommandXboxController m_driverController) {
  //}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_algae.runGrip(RobotContainer.booleanToDouble(m_controller.leftTrigger().getAsBoolean()) 
    - RobotContainer.booleanToDouble(m_controller.rightTrigger().getAsBoolean()));
    //System.out.println(RobotContainer.booleanToDouble(m_controller.leftTrigger().getAsBoolean()));
    if(m_controller.leftTrigger().getAsBoolean())
    {
      m_algae.runGrip(1);
    }
    if(m_controller.rightTrigger().getAsBoolean())
    {
      m_algae.runGrip(-1);
    }
    if((!m_controller.rightTrigger().getAsBoolean()) && (!m_controller.leftTrigger().getAsBoolean()))
    {
      m_algae.runGrip(0);
    }


    if(Math.abs(m_controller.getRightY()) > 0.05 )
    {
      m_algae.addPosition(m_controller.getRightY() * 0.1);
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