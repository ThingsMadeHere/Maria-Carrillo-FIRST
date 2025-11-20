package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightUtils;

/**
 * Command to point the robot at the nearest AprilTag using the Limelight.
 */
public class PointToNearestAprilTag extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController rotationController;
    private double rotationSpeed = 0;
    
    /**
     * Creates a new PointToNearestAprilTag command.
     * @param swerve The swerve subsystem to control
     */
    public PointToNearestAprilTag(SwerveSubsystem swerve) {
        this.swerve = swerve;
        // Configure PID controller for rotation
        this.rotationController = new PIDController(0.1, 0, 0.01);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(1.0); // 1 degree tolerance
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        // Configure Limelight for AprilTag detection
        LimelightUtils.setPipeline(LimelightUtils.APRILTAG_PIPELINE);
        LimelightUtils.setLedMode(LimelightUtils.LED_ON);
        rotationController.reset();
    }
    
    @Override
    public void execute() {
        if (LimelightUtils.hasTarget()) {
            // Get horizontal offset from target
            double tx = LimelightUtils.getTx();
            // Calculate rotation speed using PID controller
            rotationSpeed = -rotationController.calculate(tx, 0);
            // Apply rotation to swerve drive
            swerve.drive(0, 0, rotationSpeed, true, false);
        } else {
            // No target found, stop rotation
            swerve.drive(0, 0, 0, true, false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerve.drive(0, 0, 0, true, false);
        // Turn off Limelight LED to save power
        LimelightUtils.setLedMode(LimelightUtils.LED_OFF);
    }
    
    @Override
    public boolean isFinished() {
        // End when we're pointing at the target (within tolerance)
        return Math.abs(LimelightUtils.getTx()) < 1.0;
    }
}
