// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ImportantConstants;

import frc.robot.Constants.autoConfigConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.HomeAllCommand;
import frc.robot.commands.waitIntakeCommand;
import frc.robot.commands.AllMoveCommand;

import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoAlignManualCommand;
import frc.robot.commands.AlgaeMoveCommand;
import frc.robot.commands.AlgaeMoveCommandWait;
import frc.robot.commands.Autos.auto_algaeRunner;
import frc.robot.commands.Autos.auto_coralRunner;
import frc.robot.commands.Autos.auto_waitIntake;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimberSubsystem;



public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double OriginalMaxSpeed = MaxSpeed*ImportantConstants.driveSpeed; //Takes our MaxSpeed then multiplies it by our "safety" value (driveSpeed), and saves it.
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
           
    public static boolean forceLEDoff = false;
    
    public static CANBus MainBus = new CANBus("rio"); //This is for our CAN system (what speaks to all components), the default is the roborio. We used to run it on a CANivore.




    //Sets up our Subsysystems
    public final static AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();                                                                                // velocity
    public final static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    public final static CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
    public final static LEDSubsystem   m_LedSubsystem   = new LEDSubsystem();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
    private final CameraSubsystem m_CameraSubsystem = new CameraSubsystem();

    //These are "swerve requests" aka how you communicate with the swervedrive.
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    
    
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //CONTROLLERS
    private final CommandXboxController driverStick   = new CommandXboxController(0); // For driver
    private final CommandXboxController operatorStick = new CommandXboxController(1); // For operator
//  |
    private final CommandXboxController buttonBox1 = new CommandXboxController(2); //Our button box is technically two different controllers
    public final CommandXboxController buttonBox2 = new CommandXboxController(3);
//  | 
    public final CommandXboxController servoStick = new CommandXboxController(4);

    //!!IMPORTANT!! This actually *creates* our Swervedrive. You will want to talk to "drivetrain".
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    //A pid for our limelight (ll) rotations. (unecessary to be here honestly)
    public final static PIDController ll_rotatePID = new PIDController(0.3, 0, 0.005);

    //Creates our main constantly running commands.
    /*
     * I pass in our controllers, but you can make the controllers "static" and access them that way
     */

    private final AlgaeCommand m_AlgaeCommand = new AlgaeCommand(m_AlgaeSubsystem, buttonBox1, buttonBox2, operatorStick);
    private final CoralCommand m_CoralCommand = new CoralCommand(m_CoralSubsystem, buttonBox1, buttonBox2, operatorStick);
    private final ElevatorCommand m_ElevatorCommand = new ElevatorCommand(m_ElevatorSubsystem, operatorStick);
    private final ClimberCommand m_ClimberCommand = new ClimberCommand(m_ClimberSubsystem, driverStick);
    private final AutoAlignCommand a_autoAligncommand = new AutoAlignCommand(false, drivetrain);
    private final AutoAlignManualCommand a_AutoAlignManualCommand = new AutoAlignManualCommand(false, drivetrain);
    private final auto_waitIntake a_waitIntake = new auto_waitIntake(m_CoralSubsystem, m_ElevatorSubsystem); //The second input is how much time (in seconds) we take till we give up on intake


    //Ok so the following are commands for positions. 
    //This may look like a mess, but notice the different between "coral" and "algae".
    //Notice: Im stupid. I named things "Low", "Mid", and "High". These are BAD names, the true names are
    //However I'm lazy and didnt rename them (typical Noah)
    /*
     * Bottom -> L1
     * Low    -> L2
     * Mid    -> L3
     * High   -> L4
     */
    //CORAL LEFT
    private final AllMoveCommand a_coralLowL = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_low, AlgaeConstants.positions.home);
    private final AllMoveCommand a_coralMidL = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_mid, AlgaeConstants.positions.grabbing);
    private final AllMoveCommand a_coralHighL = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_high, AlgaeConstants.positions.grabbing);

    //CORAL RIGHT
    private final AllMoveCommand a_coralLowR = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_low, AlgaeConstants.positions.home);
    private final AllMoveCommand a_coralMidR = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_mid, AlgaeConstants.positions.grabbing);
    private final AllMoveCommand a_coralHighR = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_high, AlgaeConstants.positions.grabbing);

    //L1
    private final AllMoveCommand a_coralBottom = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     0, Constants.ElevatorConstants.positions.c_bottom, AlgaeConstants.positions.lowpos);

    //ALGAE (Low -> Low, Mid -> High)
    private final AlgaeMoveCommand a_algaeLow = new AlgaeMoveCommand(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.a_low, AlgaeConstants.positions.grabbing);
    private final AlgaeMoveCommand a_algaeMid = new AlgaeMoveCommand(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.a_high, AlgaeConstants.positions.grabbing);
    private final AlgaeMoveCommand a_algaeFloor = new AlgaeMoveCommand(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.home, AlgaeConstants.positions.grabbing);
    private final AlgaeMoveCommand a_algaeProcessor = new AlgaeMoveCommand(m_ElevatorSubsystem, m_AlgaeSubsystem
    , Constants.ElevatorConstants.positions.a_processing, AlgaeConstants.positions.grabbing);

    //This command wait a few seconds before running the command, this is meant for pathplanner and paths that take place at the same time.
    private final AlgaeMoveCommandWait a_algaeProcessorWait = new AlgaeMoveCommandWait(m_ElevatorSubsystem, m_AlgaeSubsystem
    , Constants.ElevatorConstants.positions.a_processing, AlgaeConstants.positions.grabbing);

    
    //CORAL MIDDLE
    private final AllMoveCommand a_coralLowM = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.home, Constants.ElevatorConstants.positions.c_low, AlgaeConstants.positions.home);
     private final AllMoveCommand a_coralMidM = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.home, Constants.ElevatorConstants.positions.c_mid, AlgaeConstants.positions.grabbing);
    private final AllMoveCommand a_coralHighM = new AllMoveCommand(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.home, Constants.ElevatorConstants.positions.c_high, AlgaeConstants.positions.home);
    
    



    /////RUNNERS (intakes and such)

    //ALGAE
    private final auto_algaeRunner a_algaeIntake = new auto_algaeRunner(m_AlgaeSubsystem, Constants.AlgaeConstants.intakeSpeed);
    private final auto_algaeRunner a_algaeOuttake = new auto_algaeRunner(m_AlgaeSubsystem, -Constants.AlgaeConstants.intakeSpeed);
    private final auto_algaeRunner a_algaeStop = new auto_algaeRunner(m_AlgaeSubsystem, 0.1);

    //CORAL
    private final auto_coralRunner a_coralIntake = new auto_coralRunner(m_CoralSubsystem, -Constants.CoralConstants.intakeSpeed);
    private final auto_coralRunner a_coralOuttake = new auto_coralRunner(m_CoralSubsystem, Constants.CoralConstants.intakeSpeed);
    private final auto_coralRunner a_coralStop = new auto_coralRunner(m_CoralSubsystem, 0);

    private final waitIntakeCommand c_waitIntake = new waitIntakeCommand(m_CoralSubsystem);


    public final static HomeAllCommand a_homeAll = new HomeAllCommand(m_CoralSubsystem, m_ElevatorSubsystem, m_AlgaeSubsystem);

    /* Path follower */ //(auto)
    private final SendableChooser<Command> autoChooser;



    public RobotContainer() {
        //The following sets up the "NamedCommands" for PathPlanner. I.E. if you put "algaeLow" in pathplanner, it runs a_algaeLow
        //Algae positions
        NamedCommands.registerCommand("algaeHighest",    a_algaeMid); //OBSOLETE: This is just old backup
        NamedCommands.registerCommand("algaeHigh"   ,    a_algaeMid); //Algae top
        NamedCommands.registerCommand("algaeLow"    ,     a_algaeLow); //Algae bottom
        NamedCommands.registerCommand("algaeFloor"  ,   a_algaeFloor); //Picking up/placing from floor
        NamedCommands.registerCommand("algaeProcessor"  ,   a_algaeProcessor); //Placing in the processor
        NamedCommands.registerCommand("algaeProcessorWait", a_algaeProcessorWait);
        
        //Coral positions
        //THESE ARE LEFT
        NamedCommands.registerCommand("coral_L2_Left", a_coralLowL); //Coral low tier
        NamedCommands.registerCommand("coral_L3_Left", a_coralMidL); //Coral middle tier
        NamedCommands.registerCommand("coral_L4_Left" , a_coralHighL); //Coral high tier
        //THESE ARE RIGHT
        NamedCommands.registerCommand("coral_L2_Right", a_coralLowR);
        NamedCommands.registerCommand("coral_L3_Right", a_coralMidR);
        NamedCommands.registerCommand("coral_L4_Right" , a_coralHighR);

        NamedCommands.registerCommand("coral_L1_Right", a_coralBottom);
        NamedCommands.registerCommand("coral_L1_Left" , a_coralBottom);
        NamedCommands.registerCommand("coral_HP"      , a_waitIntake);

        NamedCommands.registerCommand("coral_Home",     a_homeAll); //Redundant cuz I (noah) spelt it wrong once
        NamedCommands.registerCommand("robot_Home",     a_homeAll);
        NamedCommands.registerCommand("robotHome",     a_homeAll) ;
        NamedCommands.registerCommand("robot_home",     a_homeAll);

        NamedCommands.registerCommand("algaeRintake"  , a_algaeIntake);
        NamedCommands.registerCommand("algaeRouttake" , a_algaeOuttake);
        NamedCommands.registerCommand("algaeRstop"    , a_algaeStop);

        NamedCommands.registerCommand("coralRintake" , a_coralIntake) ;
        NamedCommands.registerCommand("coralRouttake", a_coralOuttake);
        NamedCommands.registerCommand("coralRstop"   , a_coralStop);

        NamedCommands.registerCommand("autoPos", a_autoAligncommand);


        
        DriverStation.silenceJoystickConnectionWarning(true); //When you have debug joysticks that are unplugged, it complains... a lot.
        //(btw DriverStation can give you some cool info, like match and comp)

        autoChooser = AutoBuilder.buildAutoChooser("Tests"); //Actually makes the chooser for SmartDashboard
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        LimelightHelpers.outputToSmartDashboard(); //This is a quick function that just outputs the X and Y to SmartDashboard
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // (Wow I've never heard of any program that says X is forward lol -Noah)
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverStick.getLeftY() * MaxSpeed) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
                        .withVelocityY(-driverStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverStick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
                )
        );
        //NOTICE: This is what actually runs the subsystems and commands!
        m_AlgaeSubsystem.setDefaultCommand(m_AlgaeCommand)      ;
        m_CoralSubsystem.setDefaultCommand(m_CoralCommand)      ;
        m_ElevatorSubsystem.setDefaultCommand(m_ElevatorCommand);
        m_ClimberSubsystem.setDefaultCommand(m_ClimberCommand)  ;
        
        drivetrain.run(() -> drivetrain.antiTip()); //Only works with Pitch (no yaw) and kinda sucks
        //^^ I dont think that actually works :(, but look into .run()


        
        driverStick.a().whileTrue(drivetrain.applyRequest(() -> brake)); //This is what is known as an "x-stance", where it prevents the bot from moving.

        driverStick.leftTrigger().onTrue(new InstantCommand(drivetrain::setDriveSlow));
        driverStick.leftTrigger().onFalse(new InstantCommand(drivetrain::setDriveNormal)); //This *should* be a command

        //this drives the robot in a "robot-oriented" path depending on what DPAD button is pressed.
        //(POV is like the DPAD direction)
        driverStick.pov(0).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(0).withRotationalRate(0)));
        driverStick.pov(180).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(-1).withVelocityY(0).withRotationalRate(0))

        );
        driverStick.pov(90).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(-1).withVelocityX(0).withRotationalRate(0)));
        driverStick.pov(270).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(1).withVelocityX(0).withRotationalRate(0)));

        // reset the field-centric heading on left select/share/minus/whatever your preference of button name is although minus is correct!
        //Your "field-centric heading" is your gyro and affects what way the robot sees as forward.
        driverStick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        
        driverStick.x().whileTrue(a_AutoAlignManualCommand);


        driverStick.start().onTrue(a_autoAligncommand);


        //Coral Left
        buttonBox2.button(10).onTrue(a_coralHighL);
        buttonBox2.button(5).onTrue(a_coralMidL);
        buttonBox1.button(11).onTrue(a_coralLowL);



        buttonBox2.button(6).onTrue(a_coralHighM)    ;
        buttonBox1.button(8).onTrue(a_coralMidM)     ;
        buttonBox1.button(5).onTrue(a_coralLowM)     ;
        //Coral Right
        buttonBox1.button(12).onTrue(a_coralHighR)    ;
        buttonBox1.button(7).onTrue(a_coralMidR)     ;
        buttonBox1.button(6).onTrue(a_coralLowR)     ;
        //
        buttonBox2.button(3).onTrue(a_algaeMid)      ;
        buttonBox2.button(4).onTrue(a_algaeLow)      ;
        buttonBox1.button(3).onTrue(a_algaeProcessor);
        buttonBox1.button(4).onTrue(a_homeAll)       ;
        buttonBox1.button(2).onTrue(c_waitIntake)    ;

        //

        buttonBox2.button(1).onTrue(new InstantCommand(m_AlgaeSubsystem::gotoOut));
        buttonBox2.button(8).onTrue(new InstantCommand(m_AlgaeSubsystem::gotoIn));

        buttonBox2.button(2).onTrue(new InstantCommand(m_ClimberSubsystem::climberRun1));
        buttonBox2.button(9).onTrue(new InstantCommand(m_ClimberSubsystem::climberRun2)); //Notice: This doesnt have an undo button.


        buttonBox2.button(7).onTrue(new InstantCommand(m_CameraSubsystem::toggleCam));



        servoStick.a().onTrue(new InstantCommand(m_ClimberSubsystem::climberEngage));
        servoStick.y().onTrue(new InstantCommand(m_ClimberSubsystem::climberServoHome));
        //servoStick.x().onTrue(new InstantCommand(m_ClimberSubsystem::climberRun1));
        //servoStick.y().onTrue(new InstantCommand(m_ClimberSubsystem::climberRun2));

        servoStick.b().onTrue(new InstantCommand(m_ClimberSubsystem::lockClimb)); //This is the end state
        servoStick.x().onTrue(new InstantCommand(m_ClimberSubsystem::unlockClimb)); //This is the basic state

        //buttonBox2.button(7).onTrue(a_algaeFloor);



        //somewhere here add home

        //deb_CoralStick.povLeft().onTrue(new InstantCommand(m_ElevatorSubsystem::gotoCoralMid_L));


        // AUTO STICK
        //autoStick.a().onTrue(drivetrain.run(() -> AutoToPosition(ImportantPositions.coral4))); // dont ask me how
                                                                                               // drivetrain.run() is
                                                                                               // used here, it just
                                                                                               // maybe works.
                                                                                               
        //autoStick.b().onTrue(m_MusicSubsystem.runOnce(() -> m_MusicSubsystem.playSong("chirp/portal.chrp")));
        // OPERATOR STICK

        // DUMB TELEMETRY THATS ACTUALLY REALLY USEFUL BUT IT ANNOYS ME

        drivetrain.registerTelemetry(logger::telemeterize);
        


    }


    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }


    public void antiTip()
    {
        double pitch = drivetrain.getPitch();
        double yaw = drivetrain.getYaw();
        double amount = 0;  
        SmartDashboard.putNumber("pitch", pitch);
        SmartDashboard.putNumber("gyro", drivetrain.getGyro().getDegrees());
        if(pitch > 15)
        {
            amount = 2;
        }
        if(pitch < -15)
        {
            amount = -2;
        }
        if(Math.abs(pitch) < 15)
        {
            amount = 0;
        }

        if(Math.abs(amount) != 0)
        {
            drivetrain.setControl(
            forwardStraight.withVelocityX(0)
            .withVelocityY(amount)
            );
        }

        if(Math.abs(pitch) > 15)
        {
            System.out.println("ERROR!!!: INITIATING EMERGENCY DROPPY STUFF!!");
            new SequentialCommandGroup(a_homeAll);  
        }
        if(Math.abs(yaw) > 15)
        {
            System.out.println("ERROR!!! INITIATING EMERGENCY DROPPY STUFF!!");
            new SequentialCommandGroup(a_homeAll);  
        }
        
    }
    

    public static double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
    public static double limelight_horizontal() { //for the horizontal section
        double kP = .01;
        double targetingForwardSpeed = LimelightHelpers.getTX("limelight") * kP;
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    public static double limelight_aim_proportional() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .014;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= MaxAngularRate;

        // invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }



    public void followAprilTag() { // Basic method for driving towards a limelight.
                                   
        final var forward_limelight = RobotContainer.limelight_range_proportional();
        final var rot_limelight = RobotContainer.limelight_aim_proportional();
        
        double ang = limelight_horizontal();

        double sideway_limelight = ang;//Math.cos(Math.toRadians(ang))*dis; //Its been a minute since I've done trig so this could be very wrong.

        System.out.println(ang);

        double current_rotation = drivetrain.getGyro().getDegrees();
        double pid_output = ll_rotatePID.calculate(current_rotation, current_rotation + rot_limelight);

        SmartDashboard.putNumber("Limelight Rot", ang)               ;
        SmartDashboard.putNumber("Limelight Fwd", forward_limelight) ;
        SmartDashboard.putNumber("Limelight Side", sideway_limelight);

        drivetrain.setControl(
                forwardStraight.withVelocityX(MathUtil.clamp(forward_limelight, -1, 1))
                        .withVelocityY(MathUtil.clamp(sideway_limelight, -.5, .5))
                        .withRotationalRate(MathUtil.clamp(pid_output, -1, 1)));

    }

    public void AutoToPosition(double[] positions) // Should in theory move us to a pose of x, y based on PathPlanner positions.
    {
        // stolen from https://pathplanner.dev/pplib-pathfinding.html#pathfind-to-pose
        String stupid_dumb_string = String.format("Positions are %s, $2d", positions[0], positions[1]);
        System.out.println(stupid_dumb_string);

        Pose2d targetPose = new Pose2d(positions[0], positions[1], Rotation2d.fromDegrees(positions[2]));
        PathConstraints constraints = new PathConstraints(
                autoConfigConstants.maxVelocityMPS, autoConfigConstants.maxAngularVelocitySq,
                Units.degreesToRadians(autoConfigConstants.maxAngularVelocity),
                Units.degreesToRadians(autoConfigConstants.maxAngularVelocitySq));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        @SuppressWarnings("unused") //I hope to remove this
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);
    }
    public static double booleanToDouble(boolean b) {
        if (b) {
            return 1;
        }
        return 0;
    }


    public boolean shouldRejectLL3G(final LimelightHelpers.PoseEstimate botPose) {
    if (botPose.tagCount < 2) {
      return true;
    }

    return false;
    }
}