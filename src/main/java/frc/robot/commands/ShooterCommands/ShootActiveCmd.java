package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootActiveCmd extends Command  {

    private ShooterSubsystem shootSubsystem;
    private double speed;
    public ShootActiveCmd(ShooterSubsystem shootSubsystem, double speed) {
        this.shootSubsystem = shootSubsystem;
        this.speed = speed;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ShootActiveCmd started!");
    }

    @Override
    public void execute() {
        shootSubsystem.setRPM(speed);
    }

    @Override
    public void end(boolean interupted) {
        System.out.println("ShootActiveCmd ended!");
        shootSubsystem.setRPM(speed);        
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(shootSubsystem.shooterEncoder.getVelocity()) - Math.abs(speed) >= 1);
    }

    

    
}
