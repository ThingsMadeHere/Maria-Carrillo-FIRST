package frc.robot.subsystems.pose_estimation.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constraint that limits movement to a certain distance from the tag
 */
public class DistanceConstraint implements AprilTagConstraint {
    private final int tagId;
    private final double minDistance;
    private final double maxDistance;
    private final double influenceRadius;
    private final int priority;
    private boolean active = true;
    
    public DistanceConstraint(int tagId, double minDistance, double maxDistance, 
                            double influenceRadius, int priority) {
        this.tagId = tagId;
        this.minDistance = minDistance;
        this.maxDistance = maxDistance;
        this.influenceRadius = influenceRadius;
        this.priority = priority;
    }
    
    @Override
    public ConstraintResult applyConstraint(Translation2d desiredMovement, 
                                         Pose2d currentPose, 
                                         Pose2d tagPose) {
        if (!active) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        double distance = tagPose.getTranslation().getNorm();
        
        // If we're outside the influence radius, don't modify movement
        if (distance > influenceRadius + maxDistance) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate the direction vector from tag to robot
        Translation2d toRobot = tagPose.getTranslation().times(-1).div(distance);
        
        // Calculate how much we're violating the constraints
        double violation = 0;
        if (distance < minDistance) {
            violation = minDistance - distance;
        } else if (distance > maxDistance) {
            violation = maxDistance - distance; // Negative value
        } else {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate the correction vector (points away from tag if too close, toward if too far)
        Translation2d correction = toRobot.times(violation * 0.1); // Damping factor
        
        // Blend the correction with the desired movement
        double blend = 1.0 - Math.min(1.0, Math.max(0, distance - minDistance) / (maxDistance - minDistance));
        Translation2d constrainedMovement = desiredMovement.plus(correction.times(blend));
        
        return new ConstraintResult(constrainedMovement, 0, true, false);
    }
    
    @Override public int getPriority() { return priority; }
    @Override public int getTagId() { return tagId; }
    @Override public boolean isActive() { return active; }
    public void setActive(boolean active) { this.active = active; }
}
