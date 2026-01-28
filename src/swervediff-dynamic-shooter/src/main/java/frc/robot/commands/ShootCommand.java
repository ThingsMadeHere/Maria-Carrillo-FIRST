package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that shoots a ball when a button is pressed.
 * This command will:
 * 1. Spin up the launcher to target RPM
 * 2. Play a random sound effect when firing
 * 3. (Optional) Trigger a mechanism to feed the ball into the launcher
 */
public class ShootCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double targetRPM;
    private boolean hasFired = false;
    
    /**
     * Creates a new ShootCommand
     * @param shooter The shooter subsystem to use
     * @param targetRPM The target RPM for the launcher
     */
    public ShootCommand(ShooterSubsystem shooter, double targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        
        // Require the shooter subsystem
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        // Reset the hasFired flag
        hasFired = false;
        
        // Prepare the shooter
        shooter.prepareShooter(targetRPM);
    }
    
    @Override
    public void execute() {
        // If we haven't fired yet and the shooter is ready
        if (!hasFired && shooter.isReadyToFire()) {
            // Fire the shooter and update the hasFired flag
            hasFired = shooter.fire();
        }
    }
    
    @Override
    public boolean isFinished() {
        // The command finishes once we've fired
        return hasFired;
    }
    
    @Override
    public void end(boolean interrupted) {
        // If the command is interrupted, stop the shooter
        if (interrupted) {
            shooter.stop();
        }
    }
}
