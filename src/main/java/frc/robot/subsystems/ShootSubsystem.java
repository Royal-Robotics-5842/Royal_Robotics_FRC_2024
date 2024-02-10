package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShootSubsystem extends SubsystemBase {

    public CANSparkMax rightShooter = new CANSparkMax(3, MotorType.kBrushless); 
    public CANSparkMax leftShooter = new CANSparkMax(4, MotorType.kBrushless);
    public CANSparkMax lefShooter = new CANSparkMax(4, MotorType.kBrushless);
    
    
    public ShootSubsystem() {
        rightShooter.restoreFactoryDefaults();
        leftShooter.restoreFactoryDefaults();
        
        rightShooter.setInverted(true);
        leftShooter.setInverted(false);
    }
     
}
