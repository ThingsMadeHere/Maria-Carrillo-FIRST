package frc.robot.subsystems.pose_estimation.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constraint that limits rotation relative to the tag
 */
public class RotationConstraint implements AprilTagConstraint {
    private final int tagId;
    private final double minAngle; // Radians
    private final double maxAngle; // Radians
    private final double influenceAngle; // Radians
    private final int priority;
    private boolean active = true;
    
    public RotationConstraint(int tagId, double minAngle, double maxAngle, 
                            double influenceAngle, int priority) {
        this.tagId = tagId;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.influenceAngle = influenceAngle;
        this.priority = priority;
    }
    
    @Override
    public ConstraintResult applyConstraint(Translation2d desiredMovement, 
                                         Pose2d currentPose, 
                                         Pose2d tagPose) {
        if (!active) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate angle from tag to robot
        double angleToTag = Math.atan2(tagPose.getY(), tagPose.getX());
        
        // Normalize angle to [-π, π]
        angleToTag = Math.IEEEremainder(angleToTag, 2 * Math.PI);
        
        // Check if we're within influence angle
        if (Math.abs(angleToTag) > influenceAngle) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate rotation correction
        double correction = 0;
        if (angleToTag < minAngle) {
            correction = (minAngle - angleToTag) * 0.1; // Damping factor
        } else if (angleToTag > maxAngle) {
            correction = (maxAngle - angleToTag) * 0.1; // Damping factor
        } else {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        return new ConstraintResult(desiredMovement, correction, false, true);
    }
    
    @Override public int getPriority() { return priority; }
    @Override public int getTagId() { return tagId; }
    @Override public boolean isActive() { return active; }
    public void setActive(boolean active) { this.active = active; }
}
