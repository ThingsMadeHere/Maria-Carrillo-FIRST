package frc.robot.subsystems.pose_estimation.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Base interface for all AprilTag constraints
 */
public interface AprilTagConstraint {
    /**
     * Apply this constraint to the desired movement
     * @param desiredMovement The desired movement vector (field-relative)
     * @param currentPose The robot's current pose
     * @param tagPose The detected AprilTag's pose relative to the robot
     * @return The constrained movement vector
     */
    ConstraintResult applyConstraint(Translation2d desiredMovement, Pose2d currentPose, Pose2d tagPose);
    
    /**
     * @return The priority of this constraint (higher = evaluated first)
     */
    int getPriority();
    
    /**
     * @return The AprilTag ID this constraint applies to (-1 for any tag)
     */
    int getTagId();
    
    /**
     * Check if this constraint is currently active
     */
    boolean isActive();
    
    /**
     * Result of applying a constraint
     */
    class ConstraintResult {
        public final Translation2d movement;
        public final boolean movementModified;
        public final boolean rotationModified;
        public final double rotation;
        
        public ConstraintResult(Translation2d movement, double rotation, 
                              boolean movementModified, boolean rotationModified) {
            this.movement = movement;
            this.rotation = rotation;
            this.movementModified = movementModified;
            this.rotationModified = rotationModified;
        }
        
        public static ConstraintResult noChange(Translation2d movement, double rotation) {
            return new ConstraintResult(movement, rotation, false, false);
        }
    }
}
