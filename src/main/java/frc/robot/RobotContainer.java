// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootActiveCmd;
import frc.robot.commands.StopShooter;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.intakeNote;
//import frc.robot.commands.intakeNote;
import frc.robot.commands.setTo0;
import frc.robot.commands.ArmPositons.ArmAmp;
import frc.robot.commands.ArmPositons.ArmIntake;
import frc.robot.commands.ArmPositons.ArmLimelight;
import frc.robot.commands.ArmPositons.ArmShotSpeaker;
import frc.robot.commands.ArmPositons.ArmWithController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  public static ArmSubsystem arm = new ArmSubsystem();
  private final ShootSubsystem shooter = new ShootSubsystem();

  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


  //AUTO STUFFF
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () ->  -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> true));

       //intake.setDefaultCommand(new intakeNoteWithController(intake));



      NamedCommands.registerCommand("ArmShoot", new ArmLimelight(arm));
      NamedCommands.registerCommand("Shoot", new ShootActiveCmd(shooter, 0.5));
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    /*
    m_driverController.a().whileTrue(
        Commands.sequence(
                new ShootActiveCmd(shooter, 0.5),
                new WaitCommand(2),
                new intakeNote(intake, -1),
                new WaitCommand(2),
                new ShootActiveCmd(shooter, 0)
        )
        ); 
*/
    m_driverController.y().onTrue(new intakeNote(intake, -1).withTimeout(0.2)
                                        .andThen(new ShootActiveCmd(shooter, 3500))
                                        .alongWith(new ArmShotSpeaker(arm)));

        //new ShootActiveCmd(shooter, 0.85));//Commands.sequence(new intakeNoteOuttake(intake).withTimeout(0.25).andThen(wait(1000))// new ShootActiveCmd(shooter, 0.85))));
    m_driverController.a()//whileTrue(new ArmLimelight(arm));
    .onTrue(new StopShooter(shooter).withTimeout(0.2).andThen(
                                new ArmIntake(arm)));

    m_driverController.x()//.whileTrue(new ArmLimelight(arm).alongWith(new ShootActiveCmd(shooter, 5000)));
    .onTrue(new StopShooter(shooter).withTimeout(0.2).andThen(
                           new ArmAmp(arm)));
      /*
        Commands.sequence(
        new intakeNote(intake, 0.1)
        )
        );
        */
        
    m_driverController.b().onTrue(new setTo0(swerveSubsystem, arm).withTimeout(0.5));

    m_driverController.leftBumper().onTrue(new ArmWithController(arm, .25));
    m_driverController.rightBumper().onTrue(new ArmWithController(arm, -0.25));

    m_driverController.rightTrigger().whileTrue(new intakeNote(intake, 0.85)).whileFalse(new intakeNote(intake, 0));
    m_driverController.leftTrigger().whileTrue(new intakeNote(intake, -0.85)).whileFalse(new intakeNote(intake, 0));

    

    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    swerveSubsystem.frontRight.setCoast();
    swerveSubsystem.frontLeft.setCoast();
    swerveSubsystem.backLeft.setCoast();
    swerveSubsystem.backRight.setCoast();

    PathPlannerPath LeftNote = PathPlannerPath.fromPathFile("LeftNote");
    PathPlannerPath LeftNoteRev = PathPlannerPath.fromPathFile("LeftNoteReverse");
    PathPlannerPath MidNote = PathPlannerPath.fromPathFile("MidNote");
    PathPlannerPath MidNoteRev = PathPlannerPath.fromPathFile("MidNoteRev");
    PathPlannerPath FifthNote = PathPlannerPath.fromPathFile("FifthNote");
    PathPlannerPath FifthNoteRev = PathPlannerPath.fromPathFile("FifthNoteRev");

    PathPlannerPath Test = PathPlannerPath.fromPathFile("Testing");

    //return autoChooser.getSelected();
    return new PathPlannerAuto("test1");
    //return Commands.runOnce(()->swerveSubsystem.resetOdemetry(LeftNote.getPreviewStartingHolonomicPose()))
        //.andThen(AutoBuilder.followPath(Test));
/*
    // Create a path following command using AutoBuilder. This will also trigger event markers.
        return Commands.runOnce(()->swerveSubsystem.resetOdemetry(LeftNote.getPreviewStartingHolonomicPose()))
        .andThen(AutoBuilder.followPath(LeftNote))
        .andThen(AutoBuilder.followPath(LeftNoteRev))
        .andThen(AutoBuilder.followPath(MidNote))
        .andThen(AutoBuilder.followPath(MidNoteRev))
        .andThen(AutoBuilder.followPath(FifthNote))
        .andThen(AutoBuilder.followPath(FifthNoteRev));
    
    // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);
        

     // 2. Generate trajectory
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1,0),
                        new Translation2d(1.5,0)),
                new Pose2d(2,0, Rotation2d.fromDegrees(180)),
                trajectoryConfig);
        
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1,0),
                        new Translation2d(1.5,0)),
                new Pose2d(2,0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);
        


        var concattraj = trajectory1.concatenate(trajectory2);

                swerveSubsystem.m_field.getObject("traj").setTrajectory(trajectory2);


    // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-180, 180);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory1,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                concattraj,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdemetry(trajectory1.getInitialPose())),
                swerveControllerCommand,
                //swerveControllerCommand2,
                new InstantCommand(() -> swerveSubsystem.stopModules()));

                */
  }
}
