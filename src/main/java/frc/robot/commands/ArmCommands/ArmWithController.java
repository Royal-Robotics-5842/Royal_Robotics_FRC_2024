package frc.robot.commands.ArmCommands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmWithController extends Command {
  /** Creates a new ArmtoSetpoint. */
  
  private final ArmSubsystem arm;
  private double speed;

  public ArmWithController(ArmSubsystem arm, double speed){
    this.arm = arm;
    this.speed = speed;
    addRequirements(arm);
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    arm.setSpeed(speed);
    SmartDashboard.putNumber("ARM ENCODER", arm.ArmLeftEncoder.getPosition());
    SmartDashboard.putNumber("NCODER", RobotContainer.m_driverController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    arm.setSpeed(0);
    arm.ArmRight.setIdleMode(IdleMode.kBrake);
    arm.ArmLeft.setIdleMode(IdleMode.kBrake);
  }


  // Returns true when the command should end.`

  @Override
  public boolean isFinished() {

  if((arm.getLimit() == false) && (speed < 0.0))
    return true;


    return ((RobotContainer.m_driverController.leftBumper().getAsBoolean() == false && RobotContainer.m_driverController.rightBumper().getAsBoolean() == false));
  }
}