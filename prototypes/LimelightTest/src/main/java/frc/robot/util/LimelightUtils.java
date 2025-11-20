package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Utility class for interacting with the Limelight camera.
 */
public class LimelightUtils {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    // LED modes
    public static final int LED_OFF = 1;
    public static final int LED_ON = 3;
    public static final int LED_BLINK = 2;
    
    // Camera modes
    public static final int VISION_PROCESSOR = 0;
    public static final int DRIVER_CAMERA = 1;
    
    // Pipeline indices
    public static final int APRILTAG_PIPELINE = 0;
    public static final int RETROREFLECTIVE_PIPELINE = 1;
    
    /**
     * Get the horizontal offset from crosshair to target (-27 to 27 degrees)
     * @return Horizontal offset in degrees
     */
    public static double getTx() {
        return table.getEntry("tx").getDouble(0);
    }
    
    /**
     * Get the vertical offset from crosshair to target (-20.5 to 20.5 degrees)
     * @return Vertical offset in degrees
     */
    public static double getTy() {
        return table.getEntry("ty").getDouble(0);
    }
    
    /**
     * Get target area (0% to 100% of image)
     * @return Target area as a percentage
     */
    public static double getTa() {
        return table.getEntry("ta").getDouble(0);
    }
    
    /**
     * Check if a target is detected
     * @return True if a target is detected
     */
    public static boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }
    
    /**
     * Get the ID of the primary AprilTag
     * @return AprilTag ID, or -1 if no target
     */
    public static int getAprilTagId() {
        return (int)table.getEntry("tid").getDouble(-1);
    }
    
    /**
     * Get the rotation of the primary AprilTag
     * @return Rotation2d of the AprilTag
     */
    public static Rotation2d getAprilTagRotation() {
        return Rotation2d.fromDegrees(table.getEntry("ts").getDouble(0));
    }
    
    /**
     * Set the LED mode
     * @param mode LED mode (LED_OFF, LED_ON, or LED_BLINK)
     */
    public static void setLedMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Set the camera mode
     * @param mode Camera mode (VISION_PROCESSOR or DRIVER_CAMERA)
     */
    public static void setCameraMode(int mode) {
        table.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Set the pipeline (0-9)
     * @param pipeline Pipeline index
     */
    public static void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Get the current pipeline
     * @return Current pipeline index
     */
    public static int getPipeline() {
        return (int)table.getEntry("getpipe").getDouble(0);
    }
}
