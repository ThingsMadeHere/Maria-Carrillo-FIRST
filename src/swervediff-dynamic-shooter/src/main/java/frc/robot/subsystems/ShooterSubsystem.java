package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SoundPlayer;

public class ShooterSubsystem extends SubsystemBase {
    private final LauncherPhysicsSubsystem launcher;
    private final SoundPlayer soundPlayer;
    
    public ShooterSubsystem(LauncherPhysicsSubsystem launcher) {
        this.launcher = launcher;
        this.soundPlayer = SoundPlayer.getInstance();
    }
    
    /**
     * Prepares the shooter by spinning up the launcher to the target RPM
     * @param targetRPM The target RPM for the launcher
     */
    public void prepareShooter(double targetRPM) {
        launcher.setTargetRPM(targetRPM);
    }
    
    /**
     * Checks if the shooter is ready to fire
     * @return True if the launcher is at the target RPM
     */
    public boolean isReadyToFire() {
        return launcher.isAtTargetRPM();
    }
    
    /**
     * Fires the shooter
     * @return True if the shot was fired successfully
     */
    public boolean fire() {
        if (isReadyToFire()) {
            // Play a random sound effect
            soundPlayer.playRandomSound();
            
            // TODO: Add code here to trigger ball feed mechanism
            // For example: feederSubsystem.feedBall();
            
            return true;
        }
        return false;
    }
    
    /**
     * Stops the shooter
     */
    public void stop() {
        launcher.stop();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
