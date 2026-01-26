package frc.robot.subsystems.pose_estimation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.pose_estimation.constraints.AprilTagConstraint;

/**
 * Manages multiple AprilTag constraints and applies them to robot movement
 */
public class AprilTagConstraintManager {
    private final LimelightSubsystem limelight;
    private final Map<Integer, List<AprilTagConstraint>> constraints = new HashMap<>();
    private boolean enabled = true;
    
    public AprilTagConstraintManager(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }
    
    /**
     * Add a constraint for a specific AprilTag
     */
    public void addConstraint(AprilTagConstraint constraint) {
        int tagId = constraint.getTagId();
        constraints.computeIfAbsent(tagId, k -> new ArrayList<>()).add(constraint);
        
        // Sort constraints by priority (highest first)
        constraints.get(tagId).sort(Comparator.comparingInt(AprilTagConstraint::getPriority).reversed());
    }
    
    /**
     * Apply all active constraints to the desired movement
     */
    public ConstraintResult applyConstraints(Translation2d desiredMovement, Pose2d currentPose) {
        if (!enabled || !limelight.hasTarget()) {
            return new ConstraintResult(desiredMovement, 0, false, false);
        }
        
        // Get the current AprilTag detection
        var tagPoseOpt = limelight.getRobotPose();
        if (tagPoseOpt.isEmpty()) {
            return new ConstraintResult(desiredMovement, 0, false, false);
        }
        
        Pose2d tagPose = tagPoseOpt.get();
        int tagId = limelight.getAprilTagId();
        
        // Get constraints for this specific tag and for all tags (-1)
        List<AprilTagConstraint> applicableConstraints = new ArrayList<>();
        if (constraints.containsKey(tagId)) {
            applicableConstraints.addAll(constraints.get(tagId));
        }
        if (constraints.containsKey(-1)) { // -1 means "any tag"
            applicableConstraints.addAll(constraints.get(-1));
        }
        
        // Sort by priority (highest first)
        applicableConstraints.sort(Comparator.comparingInt(AprilTagConstraint::getPriority).reversed());
        
        // Apply constraints in order of priority
        Translation2d currentMovement = desiredMovement;
        double rotationCorrection = 0;
        boolean movementModified = false;
        boolean rotationModified = false;
        
        for (AprilTagConstraint constraint : applicableConstraints) {
            if (!constraint.isActive()) continue;
            
            AprilTagConstraint.ConstraintResult result = constraint.applyConstraint(
                currentMovement, currentPose, tagPose);
                
            if (result.movementModified) {
                currentMovement = result.movement;
                movementModified = true;
            }
            if (result.rotationModified) {
                rotationCorrection += result.rotation;
                rotationModified = true;
            }
        }
        
        return new ConstraintResult(
            currentMovement, 
            rotationCorrection,
            movementModified,
            rotationModified
        );
    }
    
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    public boolean isEnabled() {
        return enabled;
    }
    
    public static class ConstraintResult {
        public final Translation2d movement;
        public final double rotationCorrection;
        public final boolean movementModified;
        public final boolean rotationModified;
        
        public ConstraintResult(Translation2d movement, double rotationCorrection,
                              boolean movementModified, boolean rotationModified) {
            this.movement = movement;
            this.rotationCorrection = rotationCorrection;
            this.movementModified = movementModified;
            this.rotationModified = rotationModified;
        }
    }
}
