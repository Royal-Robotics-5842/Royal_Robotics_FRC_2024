package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootActiveCmd extends Command  {

    private ShootSubsystem shootSubsystem;
    private double speed;
    public ShootActiveCmd(ShootSubsystem shootSubsystem, double speed) {
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
        shootSubsystem.rightShooter.set(speed);
        shootSubsystem.leftShooter.set(speed);

    }

    @Override
    public void end(boolean interupted) {
        System.out.println("ShootActiveCmd ended!");
        shootSubsystem.rightShooter.set(0);
        shootSubsystem.leftShooter.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    

    
}
