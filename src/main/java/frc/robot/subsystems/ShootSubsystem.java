package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShootSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor1 = new CANSparkMax(0, MotorType.kBrushless); 
    private CANSparkMax shootMotor2 = new CANSparkMax(1, MotorType.kBrushless);
    
    public ShootSubsystem() {
        shootMotor1.restoreFactoryDefaults();
        shootMotor2.restoreFactoryDefaults();
        
        shootMotor1.setInverted(true);
        shootMotor2.setInverted(false);
    }
    public void setPosition() {
            shootMotor1.set(.5);
            shootMotor2.set(0.5);
    }
     
}
