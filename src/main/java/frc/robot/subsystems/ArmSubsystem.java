package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax ArmLeft = new CANSparkMax(41, MotorType.kBrushless);
    CANSparkMax ArmRight = new CANSparkMax(42, MotorType.kBrushless);

    public RelativeEncoder ArmLeftEncoder = ArmLeft.getEncoder();
    

    SparkPIDController ArmLeftController = ArmLeft.getPIDController();
 

    public double angle = 0.0;

        private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(180, 180);
        private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.0375, 0,0, m_constraints, 0.02);


    public ArmSubsystem(){
        ArmLeft.setInverted(true);
        ArmRight.setInverted(false);

        ArmLeft.setSmartCurrentLimit(80);
        ArmRight.setSmartCurrentLimit(80);

        ArmRight.setIdleMode(IdleMode.kBrake);
        ArmLeft.setIdleMode(IdleMode.kBrake);

        ArmLeftEncoder.setPositionConversionFactor(420/360);
        
        ArmRight.follow(ArmLeft, true);
    
    }

    public void setSpeed(double speed)
    {
        ArmLeft.set(speed);
        ArmRight.set(speed);
    }

    public void moveArm(double angle){
            /*
        m_goal = new TrapezoidProfile.State(angle, 0);
        m_setpoint = m_profile.calculate(.02,m_setpoint,m_goal);
        double armSpeed = m_setpoint.position;

        ArmLeftController.setReference(armSpeed,ControlType.kPosition);
*/
        
        ArmLeft.set(m_controller.calculate(ArmLeftEncoder.getPosition(), angle));
        }

    public void getEncoder()
    {
        ArmLeftEncoder.getPosition();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
       SmartDashboard.putNumber("ARMM",ArmLeftEncoder.getPosition());    } 
}
