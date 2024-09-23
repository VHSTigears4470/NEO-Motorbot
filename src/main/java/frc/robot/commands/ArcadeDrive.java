package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Default drive command for differential drive. */
public class ArcadeDrive extends Command {
    private final DriveSubsystem sub;
    private final CommandXboxController xbox;

    /**
     * Creates a new ArcadeDrive command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArcadeDrive(DriveSubsystem sub, CommandXboxController xbox) {
        this.sub = sub;
        this.xbox = xbox;
        addRequirements(sub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sub.arcadeDrive(-1 * xbox.getLeftY(), -1 * xbox.getLeftX());
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