package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.pose_estimation.AprilTagConstraintManager;
import frc.robot.subsystems.pose_estimation.constraints.DistanceConstraint;
import frc.robot.subsystems.pose_estimation.constraints.MaintainOffsetConstraint;
import frc.robot.subsystems.pose_estimation.constraints.RotationConstraint;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that applies AprilTag-based constraints to teleop driving.
 */
public class ConstrainedDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final AprilTagConstraintManager constraintManager;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    
    /**
     * Creates a new ConstrainedDriveCommand.
     * 
     * @param swerve The swerve subsystem to control
     * @param limelight The limelight subsystem for AprilTag detection
     * @param xSupplier Supplier for forward/backward movement (-1.0 to 1.0)
     * @param ySupplier Supplier for left/right movement (-1.0 to 1.0)
     * @param rotSupplier Supplier for rotation (-1.0 to 1.0)
     */
    public ConstrainedDriveCommand(
            SwerveSubsystem swerve,
            LimelightSubsystem limelight,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        
        // Initialize constraint manager
        this.constraintManager = new AprilTagConstraintManager(limelight);
        
        // Example constraints (customize these for your needs)
        
        // Keep between 1m and 3m from tag with ID 3
        constraintManager.addConstraint(new DistanceConstraint(
            3,      // Tag ID
            1.0,    // Min distance (m)
            3.0,    // Max distance (m)
            4.0,    // Influence distance (m)
            10      // Priority
        ));
        
        // Maintain a specific offset from tag with ID 4
        constraintManager.addConstraint(new MaintainOffsetConstraint(
            4,                          // Tag ID
            new Translation2d(1.5, 0),  // 1.5m in front of tag
            0.1,                       // 10cm tolerance
            2.0,                       // 2m influence distance
            20                         // Higher priority than distance constraint
        ));
        
        // Limit rotation relative to tag with ID 5
        constraintManager.addConstraint(new RotationConstraint(
            5,                          // Tag ID
            -Math.PI/4,                 // Min angle (-45°)
            Math.PI/4,                  // Max angle (45°)
            Math.PI/2,                  // 90° influence angle
            15                         // Priority
        ));
        
        addRequirements(swerve);
    }
    
    @Override
    public void execute() {
        // Get desired movement from joysticks
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double rotSpeed = rotSupplier.getAsDouble();
        
        // Apply constraints
        var result = constraintManager.applyConstraints(
            new Translation2d(xSpeed, ySpeed),
            swerve.getPose()
        );
        
        // Apply rotation correction if needed
        if (result.rotationModified) {
            rotSpeed += result.rotationCorrection;
        }
        
        // Drive with constrained values
        swerve.drive(
            result.movement,
            rotSpeed,
            true,  // Field-relative
            true   // Open-loop control
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }
}
