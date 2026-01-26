package frc.robot.subsystems.pose_estimation.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constraint that maintains a fixed offset from the tag
 */
public class MaintainOffsetConstraint implements AprilTagConstraint {
    private final int tagId;
    private final Translation2d desiredOffset;
    private final double positionTolerance;
    private final double influenceDistance;
    private final int priority;
    private boolean active = true;
    
    public MaintainOffsetConstraint(int tagId, Translation2d desiredOffset, 
                                  double positionTolerance, 
                                  double influenceDistance, 
                                  int priority) {
        this.tagId = tagId;
        this.desiredOffset = desiredOffset;
        this.positionTolerance = positionTolerance;
        this.influenceDistance = influenceDistance;
        this.priority = priority;
    }
    
    @Override
    public ConstraintResult applyConstraint(Translation2d desiredMovement, 
                                         Pose2d currentPose, 
                                         Pose2d tagPose) {
        if (!active) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate current offset from tag
        Translation2d currentOffset = new Translation2d(
            -tagPose.getX(), // Invert because tagPose is from robot's perspective
            -tagPose.getY()
        );
        
        // Calculate offset error
        Translation2d error = desiredOffset.minus(currentOffset);
        
        // If we're outside influence distance, don't modify movement
        if (error.getNorm() > influenceDistance) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // If we're within tolerance, don't modify movement
        if (error.getNorm() < positionTolerance) {
            return ConstraintResult.noChange(desiredMovement, 0);
        }
        
        // Calculate correction vector (points toward desired offset)
        Translation2d correction = error.times(0.2); // Damping factor
        
        return new ConstraintResult(
            desiredMovement.plus(correction), 
            0, 
            true, 
            false
        );
    }
    
    @Override public int getPriority() { return priority; }
    @Override public int getTagId() { return tagId; }
    @Override public boolean isActive() { return active; }
    public void setActive(boolean active) { this.active = active; }
}
