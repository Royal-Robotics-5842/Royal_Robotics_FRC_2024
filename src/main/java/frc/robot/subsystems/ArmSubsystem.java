package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    CANSparkMax ArmLeft = new CANSparkMax(41, MotorType.kBrushless);
    CANSparkMax ArmRight = new CANSparkMax(42, MotorType.kBrushless);

    public RelativeEncoder ArmLeftEncoder = ArmLeft.getEncoder();
    

    SparkPIDController ArmLeftController = ArmLeft.getPIDController();
 

    public double angle = 0.0;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(180, 180);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(); //Making Goal
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(); //Making Setpoint


    public ArmSubsystem(){
        ArmLeft.setInverted(true);
        ArmRight.setInverted(false);

        ArmLeft.setSmartCurrentLimit(80);
        ArmRight.setSmartCurrentLimit(80);

        ArmLeftController.setP(0.3);
        ArmLeftController.setI(0);
        ArmLeftController.setD(0.02);

        ArmRight.setIdleMode(IdleMode.kBrake);
        ArmLeft.setIdleMode(IdleMode.kBrake);

        ArmLeftEncoder.setPositionConversionFactor(426.666/360);
     
        ArmLeftEncoder.setPosition(0);
        ArmRight.follow(ArmLeft, true);
    }

    public void setSpeed(double speed)
    {
        ArmLeft.set(speed);
        ArmRight.set(speed);
    }

    public void moveArm(double input){
        angle = input;

        m_goal = new TrapezoidProfile.State(angle, 0);
        var profile = new TrapezoidProfile(m_constraints);
        m_setpoint = profile.calculate(.02,m_setpoint,m_goal);
        double armSpeed = m_setpoint.position;

        ArmLeftController.setReference(armSpeed,ControlType.kPosition);
    }

    public void getEncoder()
    {
        ArmLeftEncoder.getPosition();
    }
}
