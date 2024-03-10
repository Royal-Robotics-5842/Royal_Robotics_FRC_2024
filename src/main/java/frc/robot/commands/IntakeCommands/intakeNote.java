// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeNote extends Command {
  /** Creates a new intakeNote. */
  private final IntakeSubsystem intakeSubsystem;
  private double speed;

  public intakeNote(IntakeSubsystem intakeSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.speed = speed;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    System.out.println("IntakeNote START");  
    intakeSubsystem.setMotors(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //ommands.sequence(new intakeNote(intakeSubsystem, 0.1).withTimeout(1));
    System.out.println("IntakeNote START");
    intakeSubsystem.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//RobotContainer.m_driverController.leftTrigger().getAsBoolean() == false && RobotContainer.m_driverController.rightTrigger().getAsBoolean() == false ;
  }
}