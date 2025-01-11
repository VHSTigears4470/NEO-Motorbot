package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Default drive command for differential drive. */
public class AutoForward extends Command {
    private final DriveSubsystem sub;

    /**
     * Creates a new ArcadeDrive command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoForward(DriveSubsystem sub) {
        this.sub = sub;
        addRequirements(sub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sub.arcadeDrive(0, 1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        sub.stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}