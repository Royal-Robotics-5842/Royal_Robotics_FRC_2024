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
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.setTo0;
import frc.robot.commands.ArmPositons.ArmA;
import frc.robot.commands.ArmPositons.ArmB;
import frc.robot.commands.ArmPositons.ArmWithController;
import frc.robot.commands.ArmPositons.ArmX;
import frc.robot.commands.ArmPositons.ArmY;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private final ShootActiveCmd shootActiveCmd = new ShootActiveCmd(shootSubsystem);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final setTo0 setTo0 = new setTo0(swerveSubsystem);

  private final ArmWithController ArmWithController = new ArmWithController(arm); 

  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  
  public final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


        //AUTO STUFFF
        private final SendableChooser<Command> autoChooser;

  public static ArmSubsystem arm = new ArmSubsystem();

  public static ArmA armA = new ArmA(arm);
  public static ArmB armB = new ArmB(arm);
  public static ArmX armX = new ArmX(arm);
  public static ArmY armY = new ArmY(arm);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    /*
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () ->  -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> true));
*/
      arm.setDefaultCommand(ArmWithController);

      NamedCommands.registerCommand("ArmShoot", armA);
      NamedCommands.registerCommand("Shoot", shootActiveCmd);
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
    m_driverController.a().whileTrue(shootActiveCmd);
    m_driverController.x().onTrue(armA);
    m_driverController.b().onTrue(setTo0.withTimeout(0.2));

    //m_driverController.b().onTrue(armB);
    //m_driverController.x().onTrue(armX);
    //m_driverController.y().onTrue(armY);
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
