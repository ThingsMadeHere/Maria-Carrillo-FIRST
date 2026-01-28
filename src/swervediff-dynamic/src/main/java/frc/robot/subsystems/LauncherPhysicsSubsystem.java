package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A subsystem that controls a launcher with physics calculations including
 * Magnus effect and drag.
 */
public class LauncherPhysicsSubsystem extends SubsystemBase {
    // Motor and encoder
    private final Spark launcherMotor;
    private final Encoder encoder;
    
    // PID Controller for RPM control
    private final PIDController pidController;
    
    // Physical constants
    private static final double WHEEL_RADIUS = 0.0762; // 3 inches in meters
    private static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level, 15°C
    private static final double DRAG_COEFFICIENT = 0.47; // For a smooth sphere
    private static final double MAX_RPM = 6000; // Maximum RPM of the launcher
    private static final double TOLERANCE_RPM = 50; // RPM tolerance for at-speed check
    
    // State
    private double targetRPM = 0;
    private double currentRPM = 0;
    
    // Ball parameters (default values)
    private double ballMass = 0.5; // kg (standard game piece mass)
    private double ballRadius = 0.15; // meters
    
    public LauncherPhysicsSubsystem() {
        launcherMotor = new Spark(0); // Update channel as needed
        encoder = new Encoder(0, 1); // Update DIO channels as needed
        
        // Configure encoder distance per pulse (meters per pulse)
        // Assuming 20 pulses per revolution and wheel circumference = 2 * PI * radius
        double distancePerPulse = (2 * Math.PI * WHEEL_RADIUS) / 20.0;
        encoder.setDistancePerPulse(distancePerPulse);
        
        // PID Controller (tune these values)
        pidController = new PIDController(0.1, 0.0, 0.01);
        pidController.setTolerance(TOLERANCE_RPM);
    }
    
    @Override
    public void periodic() {
        // Calculate current RPM
        currentRPM = (encoder.getRate() * 60.0) / (2 * Math.PI * WHEEL_RADIUS);
        
        // Update motor output using PID
        double output = pidController.calculate(currentRPM, targetRPM);
        launcherMotor.set(output);
        
        // Update SmartDashboard
        updateSmartDashboard();
    }
    
    /**
     * Sets the target RPM for the launcher
     * @param rpm Target RPM (0 to MAX_RPM)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.min(Math.max(rpm, 0), MAX_RPM);
    }
    
    /**
     * Calculates the required RPM to launch a ball to a specific distance
     * @param distanceMeters Target distance in meters
     * @param launchAngleDeg Launch angle in degrees
     * @return Required RPM
     */
    public double calculateRequiredRPM(double distanceMeters, double launchAngleDeg) {
        // Convert angle to radians
        double theta = Math.toRadians(launchAngleDeg);
        
        // Initial velocity calculation (simplified, ignoring air resistance for initial estimate)
        double g = 9.81; // m/s^2
        double v0 = Math.sqrt((distanceMeters * g) / Math.sin(2 * theta));
        
        // Calculate required wheel surface speed (m/s)
        // This is a simplified model - in reality, you'd need to account for:
        // 1. Slip between wheel and ball
        // 2. Energy losses
        // 3. Magnus effect
        double wheelSurfaceSpeed = v0 * 1.1; // 10% extra to account for losses
        
        // Convert to RPM
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        double requiredRPM = (wheelSurfaceSpeed * 60) / circumference;
        
        return Math.min(requiredRPM, MAX_RPM);
    }
    
    /**
     * Calculates the Magnus force on the ball
     * @param velocity Ball velocity in m/s
     * @param spinRate Ball spin rate in rad/s
     * @return Magnus force in Newtons
     */
    private double calculateMagnusForce(double velocity, double spinRate) {
        // Magnus force = (2 * π² * r³ * ρ * v * ω) / (4 + (r * ω / v))
        // where r is ball radius, ρ is air density, v is velocity, ω is spin rate
        double numerator = 2 * Math.PI * Math.PI * Math.pow(ballRadius, 3) * AIR_DENSITY * velocity * spinRate;
        double denominator = 4 + (ballRadius * spinRate / velocity);
        return numerator / denominator;
    }
    
    /**
     * Calculates the drag force on the ball
     * @param velocity Ball velocity in m/s
     * @return Drag force in Newtons
     */
    private double calculateDragForce(double velocity) {
        // Drag force = 0.5 * ρ * v² * C_d * A
        // where ρ is air density, v is velocity, C_d is drag coefficient, A is cross-sectional area
        double crossSectionalArea = Math.PI * ballRadius * ballRadius;
        return 0.5 * AIR_DENSITY * velocity * velocity * DRAG_COEFFICIENT * crossSectionalArea;
    }
    
    /**
     * Gets the current RPM of the launcher
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return currentRPM;
    }
    
    /**
     * Checks if the launcher is at the target RPM within tolerance
     * @return True if at target RPM
     */
    public boolean isAtTargetRPM() {
        return Math.abs(currentRPM - targetRPM) <= TOLERANCE_RPM;
    }
    
    /**
     * Stops the launcher
     */
    public void stop() {
        targetRPM = 0;
        launcherMotor.set(0);
    }
    
    /**
     * Updates the SmartDashboard with launcher information
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Launcher/Target RPM", targetRPM);
        SmartDashboard.putNumber("Launcher/Current RPM", currentRPM);
        SmartDashboard.putBoolean("Launcher/At Target", isAtTargetRPM());
    }
    
    // Getters and setters for ball parameters
    public void setBallMass(double mass) {
        this.ballMass = Math.max(0.1, mass); // Prevent zero or negative mass
    }
    
    public void setBallRadius(double radius) {
        this.ballRadius = Math.max(0.01, radius); // Prevent zero or negative radius
    }
    
    public double getBallMass() {
        return ballMass;
    }
    
    public double getBallRadius() {
        return ballRadius;
    }
}