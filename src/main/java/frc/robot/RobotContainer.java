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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ImportantConstants;
import frc.robot.Constants.ImportantPositions;
import frc.robot.Constants.autoConfigConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.Autos.auto_algaeMove;
import frc.robot.commands.Autos.auto_algaeRunner;
import frc.robot.commands.Autos.auto_coralMove;
import frc.robot.commands.Autos.auto_coralRunner;
import frc.robot.commands.Autos.auto_homeAll;

import frc.robot.commands.Autos.auto_waitIntake;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.CoralCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    public static double OriginalMaxSpeed = MaxSpeed*ImportantConstants.driveSpeed; //Gonna be so On G I have no clue if this actually did anything lol
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
           
    
    public static CANBus MainBus = new CANBus("main");    
    
    // second max angular
    public final static AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();                                                                                // velocity
    private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    public final static CoralSubsystem m_CoralSubsystem = new CoralSubsystem();
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();
    private final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /*
     * public final static SwerveRequest.RobotCentric autonMovement = new
     * SwerveRequest.RobotCentric()
     * .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
     * .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.3);
     */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverStick   = new CommandXboxController(0); // For driver
    private final CommandXboxController operatorStick = new CommandXboxController(1); // For operator
    //private final CommandXboxController autoStick     = new CommandXboxController(2); // For handling the autonomous "drive to positions"
    //private final CommandXboxController deb_CoralStick = new CommandXboxController(3);
    
    private final CommandXboxController buttonBox1 = new CommandXboxController(2);
    public final CommandXboxController buttonBox2 = new CommandXboxController(3);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();



    public final static PIDController ll_rotatePID = new PIDController(0.3, 0, 0.005);

    private final AlgaeCommand m_AlgaeCommand = new AlgaeCommand(m_AlgaeSubsystem, buttonBox1, buttonBox2, operatorStick);
    private final CoralCommand m_CoralCommand = new CoralCommand(m_CoralSubsystem, buttonBox1, buttonBox2, operatorStick);
    private final ElevatorCommand m_ElevatorCommand = new ElevatorCommand(m_ElevatorSubsystem, operatorStick);
    
    
    
    //AUTO COMMANDS
    private final auto_waitIntake a_waitIntake = new auto_waitIntake(m_CoralSubsystem, 4); //The second input is how much time (in seconds) we take till we give up on intake



    //CORAL POSITIONS
    private final auto_coralMove a_coralLowL = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_low);
    private final auto_coralMove a_coralMidL = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_mid);

    private final auto_coralMove a_coralHighL = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.left, Constants.ElevatorConstants.positions.c_high);

    private final auto_coralMove a_coralLowR = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_low);
    private final auto_coralMove a_coralMidR = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_mid);

    private final auto_coralMove a_coralHighR = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     Constants.CoralConstants.positions.right, Constants.ElevatorConstants.positions.c_high);

    private final auto_coralMove a_coralBottom = new auto_coralMove(m_ElevatorSubsystem, m_CoralSubsystem,
     0, Constants.ElevatorConstants.positions.c_bottom);

    private final auto_algaeMove a_algaeLow = new auto_algaeMove(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.a_low);
    private final auto_algaeMove a_algaeMid = new auto_algaeMove(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.a_mid);
    //private final auto_algaeMove a_algaeHigh = new auto_algaeMove(m_ElevatorSubsystem, m_AlgaeSubsystem,
    // Constants.ElevatorConstants.positions.a_high);
    private final auto_algaeMove a_algaeFloor = new auto_algaeMove(m_ElevatorSubsystem, m_AlgaeSubsystem,
     Constants.ElevatorConstants.positions.a_floor);
    private final auto_algaeMove a_algaeProcessor = new auto_algaeMove(m_ElevatorSubsystem, m_AlgaeSubsystem
    , Constants.ElevatorConstants.positions.a_processing);
    //
        //
    //
    //private final auto_moveCoral a_coralLeft = new auto_moveCoral(m_CoralSubsystem, Constants.CoralConstants.positions.left);
    //private final auto_moveCoral a_coralRight = new auto_moveCoral(m_CoralSubsystem, Constants.CoralConstants.positions.right);
    //private final auto_moveCoral a_coralHome = new auto_moveCoral(m_CoralSubsystem, 0);



    /////RUNNERS (intakes and such)

    //ALGAE
    private final auto_algaeRunner a_algaeIntake = new auto_algaeRunner(m_AlgaeSubsystem, Constants.AlgaeConstants.intakeSpeed);
    private final auto_algaeRunner a_algaeOuttake = new auto_algaeRunner(m_AlgaeSubsystem, -Constants.AlgaeConstants.intakeSpeed);
    private final auto_algaeRunner a_algaeStop = new auto_algaeRunner(m_AlgaeSubsystem, 0);

    //CORAL
    private final auto_coralRunner a_coralIntake = new auto_coralRunner(m_CoralSubsystem, Constants.CoralConstants.intakeSpeed);
    private final auto_coralRunner a_coralOuttake = new auto_coralRunner(m_CoralSubsystem, -Constants.CoralConstants.intakeSpeed);
    private final auto_coralRunner a_coralStop = new auto_coralRunner(m_CoralSubsystem, 0);


        //


    //private final 


    //
    private final auto_homeAll a_homeAll = new auto_homeAll(m_CoralSubsystem, m_ElevatorSubsystem, m_AlgaeSubsystem);
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public void createFrontUsbCamera() {
        CameraServer.startAutomaticCapture(); // Camera stuff :3
    }

    public RobotContainer() {
        //Algae positions
        //NamedCommands.registerCommand("act_algaeNet", a_algaeHigh);
        NamedCommands.registerCommand("act_algaeHigh ",    a_algaeMid); //Algae top
        NamedCommands.registerCommand("act_algaeLow  ",     a_algaeLow); //Algae bottom
        NamedCommands.registerCommand("act_algaeFloor",   a_algaeFloor); //Picking up/placing from floor
        //Coral positions
        //THESE ARE LEFT
        NamedCommands.registerCommand("act_coralLow_L ", a_coralLowL); //Coral low tier
        NamedCommands.registerCommand("act_coralMid_L ", a_coralMidL); //Coral middle tier
        NamedCommands.registerCommand("act_coralHigh_L", a_coralHighL); //Coral high tier
        //THESE ARE RIGHT
        NamedCommands.registerCommand("act_coralLow_R ", a_coralLowR);
        NamedCommands.registerCommand("act_coralMid_R ", a_coralMidR);
        NamedCommands.registerCommand("act_coralHigh_R", a_coralHighR);
        NamedCommands.registerCommand("act_coralBottom", a_coralBottom);
        NamedCommands.registerCommand("act_coralHP", a_waitIntake);

        NamedCommands.registerCommand("act_algaeRintake", a_algaeIntake);
        NamedCommands.registerCommand("act_algaeRouttake", a_algaeOuttake);
        NamedCommands.registerCommand("act_algaeRstop", a_algaeStop);

        NamedCommands.registerCommand("act_coralRintake", a_coralIntake);
        NamedCommands.registerCommand("act_coralRouttake", a_coralOuttake);
        NamedCommands.registerCommand("act_coralRstop", a_coralStop);



        //Add act_algaeRintake and act_algaeRstop, act_algaeRouttake
        //Add act_intakeRstart and act_intakeRstop 
        //act_coralHP
        //R means runner. Idk man

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        createFrontUsbCamera();
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
        m_AlgaeSubsystem.setDefaultCommand(m_AlgaeCommand);
        m_CoralSubsystem.setDefaultCommand(m_CoralCommand);
        m_ElevatorSubsystem.setDefaultCommand(m_ElevatorCommand);
        driverStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverStick.leftTrigger().onTrue(new InstantCommand(drivetrain::setDriveSlow));
        driverStick.leftTrigger().onFalse(new InstantCommand(drivetrain::setDriveNormal));

        driverStick.pov(0).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0)));
        driverStick.pov(180).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0).withRotationalRate(0))

        );

        driverStick.pov(90).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(-0.5).withVelocityX(0).withRotationalRate(0)));
        driverStick.pov(270).whileTrue(drivetrain
                .applyRequest(() -> forwardStraight.withVelocityY(0.5).withVelocityX(0).withRotationalRate(0)));

        // reset the field-centric heading on left bumper press
        driverStick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverStick.x().whileTrue(drivetrain.run(() -> followAprilTag()));

        //deb_CoralStick.x().onTrue(new InstantCommand( m_CoralSubsystem::gotoLeft  ));
        //deb_CoralStick.b().onTrue(new InstantCommand(m_CoralSubsystem::gotoRight  ));


        //Coral Left
        buttonBox2.button(6).onTrue(a_coralHighL);
        buttonBox1.button(8).onTrue(a_coralMidL);
        buttonBox1.button(5).onTrue(a_coralLowL);
        //Coral Right
        buttonBox2.button(5).onTrue(a_coralHighR);
        buttonBox1.button(7).onTrue(a_coralMidR) ;
        buttonBox1.button(6).onTrue(a_coralLowR) ;

        buttonBox2.button(3).onTrue(a_algaeMid)  ;
        buttonBox2.button(4).onTrue(a_algaeLow)  ;
        buttonBox1.button(3).onTrue(a_algaeProcessor);
        buttonBox1.button(4).onTrue(a_homeAll)   ;



        buttonBox2.button(1).onTrue(new InstantCommand(m_AlgaeSubsystem::gotoOut));
        buttonBox2.button(8).onTrue(new InstantCommand(m_AlgaeSubsystem::gotoIn));
        
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

    public static double limelight_range_proportional() {
        double kP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
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
        double kP = .035;

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

    public void followAprilTag() { // Basic method for driving towards a limelight. Good for debugging not likely
                                   // to actually use this.
        final var forward_limelight = RobotContainer.limelight_range_proportional();
        final var rot_limelight = RobotContainer.limelight_aim_proportional();

        double current_rotation = drivetrain.getGyro().getDegrees();
        double pid_output = ll_rotatePID.calculate(current_rotation, current_rotation + rot_limelight);

        SmartDashboard.putNumber("Limelight Rot", rot_limelight);
        SmartDashboard.putNumber("Limelight Fwd", forward_limelight);

        drivetrain.setControl(
                forwardStraight.withVelocityX(MathUtil.clamp(forward_limelight, -.5, .5))
                        .withVelocityY(0)
                        .withRotationalRate(MathUtil.clamp(pid_output, -1, 1)));

    }

    public void AutoToPosition(double[] positions) // Should in theory move us to a pose of x, y. Waiting till we have
                                                   // more of the field to tell
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
        @SuppressWarnings("unused") //I get annoyed by yellow
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
}
