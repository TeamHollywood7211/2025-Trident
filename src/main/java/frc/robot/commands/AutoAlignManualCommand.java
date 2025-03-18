// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.autoAlign;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlignManualCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drivetrain;
  private final boolean isRightScore;
  private PIDController rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private double tagID = -1;
  private double maxSpeed = RobotContainer.MaxSpeed/2;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignManualCommand(boolean isRightScore, CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    this.isRightScore = isRightScore;
    rotController = new PIDController(autoAlign.ROT_REEF_ALIGNMENT_P, 0, 0);
    




    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(autoAlign.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(autoAlign.ROT_TOLERANCE_REEF_ALIGNMENT);

    if(LimelightHelpers.getTV(""))
    {
      tagID = LimelightHelpers.getFiducialID("");


    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID)
    {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

      final var forward_limelight = RobotContainer.limelight_range_proportional();
      final var hori_limelight = RobotContainer.limelight_aim_proportional();
        
      //System.out.println("(AUTO ALIGN): CURRENT TIMER: " + stopTimer.get());

      double xSpeed = forward_limelight;
      double ySpeed = hori_limelight;

      double rotValue = -rotController.calculate(positions[4]);

      xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
      ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

      if(rotController.atSetpoint())
      {
        rotValue = 0;
      }


      m_drivetrain.setControl(
        RobotContainer.forwardStraight.withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotValue)
      );      

    }
    else{
      m_drivetrain.setControl(
        RobotContainer.forwardStraight.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
        
      );
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
