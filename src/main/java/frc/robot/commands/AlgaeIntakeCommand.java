package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
//import frc.robot.Constants.*;

public class AlgaeIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private AlgaeIntakeSubsystem m_algaeIntakeSystem;
  private CommandXboxController m_controller;

  public AlgaeIntakeCommand(AlgaeIntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.m_controller = controller;
    this.m_algaeIntakeSystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getRightTriggerAxis() > 0.05){
      m_algaeIntakeSystem.runGripIn(m_controller.getRightTriggerAxis());
      
    } else if(m_controller.getLeftTriggerAxis() > 0.05){
      m_algaeIntakeSystem.runGripOut(m_controller.getLeftTriggerAxis());

    }
    else{
      //m_gripSubsystem.setGripOut();
      m_algaeIntakeSystem.stopGrip();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  public void grabAlgaeHard(double speed){
    m_algaeIntakeSystem.runGripIn(speed * 2);
    } 
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}