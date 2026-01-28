package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DifferentialDriveSubsystem;

/**
 * Command for controlling the robot's differential drive with arcade drive.
 */
public class DifferentialDriveCommand extends Command {
    private final DifferentialDriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier turnSupplier;
    private final boolean squareInputs;

    /**
     * Creates a new DifferentialDriveCommand.
     * 
     * @param driveSubsystem The drive subsystem this command will run on
     * @param forwardSupplier Supplier for forward/backward input (-1.0 to 1.0)
     * @param turnSupplier Supplier for rotation input (-1.0 to 1.0)
     * @param squareInputs Whether to square the input values for better low-speed control
     */
    public DifferentialDriveCommand(DifferentialDriveSubsystem driveSubsystem,
                                  DoubleSupplier forwardSupplier,
                                  DoubleSupplier turnSupplier,
                                  boolean squareInputs) {
        this.driveSubsystem = driveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;
        this.squareInputs = squareInputs;
        
        // Add requirements
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Optional: Reset any necessary state when the command starts
    }

    @Override
    public void execute() {
        // Get the forward and turn values from the suppliers
        double forward = forwardSupplier.getAsDouble();
        double turn = turnSupplier.getAsDouble();
        
        // Apply deadband to avoid unwanted movement
        forward = applyDeadband(forward, 0.1);
        turn = applyDeadband(turn, 0.1);
        
        // Apply square inputs if enabled
        if (squareInputs) {
            forward = Math.copySign(forward * forward, forward);
            turn = Math.copySign(turn * turn, turn);
        }
        
        // Drive the robot
        driveSubsystem.arcadeDrive(forward, turn);
    }
    
    /**
     * Applies a deadband to the input value.
     * 
     * @param value The input value to apply the deadband to
     * @param deadband The deadband threshold
     * @return The value after applying the deadband
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive when the command ends
        driveSubsystem.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // This command should run continuously
        return false;
    }
}
