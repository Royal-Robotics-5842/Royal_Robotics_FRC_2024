package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command  {

    private ShooterSubsystem shootSubsystem;
    private double speed;
    public StopShooter(ShooterSubsystem shootSubsystem) {
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shootSubsystem.leftShooter.set(0);
        shootSubsystem.rightShooter.set(0);
    }

    @Override
    public void end(boolean interupted) {
          
    }

    @Override
    public boolean isFinished() {
        return false;//(Math.abs(shootSubsystem.shooterEncoder.getVelocity()) - Math.abs(speed) >= 1);
    }

    

    
}
