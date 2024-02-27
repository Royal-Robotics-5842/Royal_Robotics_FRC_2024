package frc.robot.commands;

import frc.robot.subsystems.EndgameSubsystem;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

public class EndgameCmd extends Command {

    private final EndgameSubsystem m_endgameSubsystem;
    private double egspeedL;
    private double egspeedR;
    private boolean invooort = false;


    public EndgameCmd(EndgameSubsystem endgameSubsystem, double speed, double speed2, boolean invoort) {
      egspeedL = speed;
      egspeedR = speed2;
      invooort = invoort;
      m_endgameSubsystem = endgameSubsystem;
      addRequirements(endgameSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("EndgameCmd.java started");
    }
  
    @Override
    public void execute() {
      if(invooort==true) {
        egspeedL *= -1;
        egspeedR *= -1;
      }
      m_endgameSubsystem.setMotors(egspeedL, egspeedR);
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
