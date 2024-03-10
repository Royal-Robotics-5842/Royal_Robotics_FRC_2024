package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSubsystem extends SubsystemBase {

    public CANSparkMax rightShooter = new CANSparkMax(Constants.shooterConstants.rightCANID, MotorType.kBrushless); 
    public CANSparkMax leftShooter = new CANSparkMax(Constants.shooterConstants.leftCANID, MotorType.kBrushless);

    public RelativeEncoder shooterEncoder = rightShooter.getEncoder();
    
    public SparkPIDController pid = rightShooter.getPIDController();
    
    public ShooterSubsystem() {
        rightShooter.restoreFactoryDefaults();
        leftShooter.restoreFactoryDefaults();

        rightShooter.setSmartCurrentLimit(40);
        leftShooter.setSmartCurrentLimit(40);

        pid.setFF(0.00019);

        pid.setOutputRange(-1, 1);

        leftShooter.follow(rightShooter);

        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);

    }

    public void setRPM(double rpm)
    {
        double setpoint = rpm;        
        pid.setReference(setpoint, ControlType.kVelocity);
    }
     
}
