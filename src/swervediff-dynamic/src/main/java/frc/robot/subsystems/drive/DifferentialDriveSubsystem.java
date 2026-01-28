package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.util.Units;

public class DifferentialDriveSubsystem extends SubsystemBase {
    // Motor controllers - update these IDs to match your robot
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(1);
    private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
    private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(3);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);
    
    private final DifferentialDrive drive;
    
    // Robot track width (distance between left and right wheels) in meters
    private static final double kTrackWidthMeters = Units.inchesToMeters(24);
    private final DifferentialDriveKinematics kinematics = 
        new DifferentialDriveKinematics(kTrackWidthMeters);
    
    public DifferentialDriveSubsystem() {
        // Configure motor controllers
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        
        // Invert right side
        rightLeader.setInverted(true);
        rightFollower.setInverted(true);
        
        // Create differential drive
        drive = new DifferentialDrive(leftLeader, rightLeader);
    }
    
    /**
     * Arcade drive method for differential drive
     * @param speed Forward/backward speed, from -1.0 to 1.0
     * @param rotation Rotation speed, from -1.0 to 1.0
     */
    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }
    
    /**
     * Tank drive method for differential drive
     * @param leftSpeed Left side speed, from -1.0 to 1.0
     * @param rightSpeed Right side speed, from -1.0 to 1.0
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }
    
    /**
     * Get the wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftLeader.getSelectedSensorVelocity() * 10 * (Math.PI * Units.inchesToMeters(6)) / 4096,
            rightLeader.getSelectedSensorVelocity() * 10 * (Math.PI * Units.inchesToMeters(6)) / 4096
        );
    }
    
    /**
     * Get the differential drive kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }
    
    @Override
    public void periodic() {
        // Update any necessary periodic tasks
    }
}
