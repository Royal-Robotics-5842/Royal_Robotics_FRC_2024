package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    public CANSparkMax ArmLeft = new CANSparkMax(Constants.armConstants.leftCANID, MotorType.kBrushless);
    public CANSparkMax ArmRight = new CANSparkMax(Constants.armConstants.rightCANID, MotorType.kBrushless);

    public RelativeEncoder ArmLeftEncoder = ArmLeft.getEncoder();
    

    SparkPIDController ArmLeftController = ArmLeft.getPIDController();
    
    DigitalInput armLimitSwitch = new DigitalInput(0);

    public double angle = 0.0;

    private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(180,180);
    private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.055, 0,0, m_constraints, 0.02);

    public double pidSpeed;


    public ArmSubsystem(){
        ArmLeft.setInverted(true);
        ArmRight.setInverted(false);

        ArmLeft.setSmartCurrentLimit(80);
        ArmRight.setSmartCurrentLimit(80);

        ArmRight.setIdleMode(IdleMode.kCoast);
        ArmLeft.setIdleMode(IdleMode.kCoast);

        ArmLeftEncoder.setPositionConversionFactor(420/360);
        ArmLeftEncoder.setPosition(0);
        
        ArmRight.follow(ArmLeft, true);
    
    }

    public void setSpeed(double speed)
    {
        ArmLeft.set(speed);
        ArmRight.set(speed);
    }

    public boolean getLimit()
    {
        return armLimitSwitch.get();
    }

    public void moveArm(double angle){
   
        pidSpeed = m_controller.calculate(ArmLeftEncoder.getPosition(), angle);
         

        ArmLeft.set(pidSpeed);
        }

    public void getEncoder()
    {
        ArmLeftEncoder.getPosition();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
       SmartDashboard.putNumber("ARMM",ArmLeftEncoder.getPosition());
       
 } 
}

