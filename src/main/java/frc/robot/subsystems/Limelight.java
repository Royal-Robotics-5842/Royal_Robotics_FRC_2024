package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      
      

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches; 

    // distance from the target to the floor
    double goalHeightInches; 

    //height of perfect entry into speaker
    double speakerHoleHeight = 80.515;

    //distance from april tag to the middle of the hole
    double speakerHoleDistance = 9.0;

    
    public Limelight(double limelightMountAngleDegrees,
                     double limelightLensHeightInches,
                     double goalHeightInches)
    {
        this.limelightMountAngleDegrees = limelightMountAngleDegrees;
        this.limelightLensHeightInches = limelightLensHeightInches;
        this.goalHeightInches = goalHeightInches;
    }


    public double getLimelightY()
    {
        return table.getEntry("ty").getDouble(0);
    }

    public double getLimelightX()
    {
        return table.getEntry("tx").getDouble(0);
    }

    public double getLimelightArea()
    {
        return table.getEntry("ta").getDouble(0);
    }

    public double getLimelightMountAngleDegrees()
    {
        return limelightMountAngleDegrees;
    }

    public double getLimelightLensHeightInches()
    {
        return limelightLensHeightInches;
    }
    
    public double getGoalHeightInches()
    {
        return goalHeightInches;
    }
    
    public double getTargetOffsetAngle_Vertical()
    {
        return getLimelightY();
    }

    public double getAngleToGoalDegrees()
    {
        return (getLimelightMountAngleDegrees() + getTargetOffsetAngle_Vertical());
    }

    public double getAngleToGoalRadians()
    {
        return (getAngleToGoalDegrees() * (Math.PI / 180.0));
    }

    public double getDistanceFromLimelightToGoalInches()
    {
        return (getGoalHeightInches() - getLimelightLensHeightInches()) / Math.tan(getAngleToGoalRadians());
    }

    public double getAngleFromAprilTag()
    {
        double xDistance = (getDistanceFromLimelightToGoalInches() - speakerHoleDistance);
        double yDistance = speakerHoleHeight;
        double test = (Math.atan(yDistance/xDistance)) * (180/Math.PI);
        return test;
    }

}

