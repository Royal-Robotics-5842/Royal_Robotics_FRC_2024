package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShootSubsystem extends SubsystemBase {

    public CANSparkMax rightShooter = new CANSparkMax(3, MotorType.kBrushless); 
    public CANSparkMax leftShooter = new CANSparkMax(35, MotorType.kBrushless);
    
    
    public ShootSubsystem() {
        rightShooter.restoreFactoryDefaults();
        leftShooter.restoreFactoryDefaults();

        rightShooter.setSmartCurrentLimit(30);
        leftShooter.setSmartCurrentLimit(30);
        
        rightShooter.setInverted(false);
        leftShooter.setInverted(false);

    }
     
}
