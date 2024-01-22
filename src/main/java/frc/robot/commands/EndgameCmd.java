package frc.robot.commands;

import frc.robot.subsystems.EndgameSubsystem;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

public class EndgameCmd extends Command {

    private final EndgameSubsystem m_endgameSubsystem;
    private final double m_speed;

    public EndgameCmd(EndgameSubsystem endgameSubsystem) {
      m_endgameSubsystem = endgameSubsystem;
      //temp value change later duh -Joe Smoe
      m_speed = 0;
      addRequirements(endgameSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("EndgameCmd.java started");
    }
  
    @Override
    public void execute() {
      m_endgameSubsystem.setMotors(m_speed);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_endgameSubsystem.setMotors(0);
      System.out.println("EndgameCmd.java ended");
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
