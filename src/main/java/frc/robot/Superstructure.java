package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class Superstructure {
    public final Intake intake_;
    public final Indexer indexer_;

    public Superstructure(Intake intake, Indexer indexer) {
        intake_ = intake;
        indexer_ = indexer;
    }

    // Roller jog
    public Command jogRoller(double percent) {
        return new StartEndCommand(
            () -> intake_.setRollerPercent(percent),
            () -> intake_.stopRoller(),
            intake_);
    }

    // Extension jog
    public Command jogExtension(double percent) {
        return new StartEndCommand(
            () -> intake_.setExtensionPercent(percent),
            () -> intake_.stopExtension(),
            intake_);
    }

    // Indexer jog
    public Command jogIndexer(double percent) {
        return new StartEndCommand(
            () -> indexer_.setPercent(percent),
            () -> indexer_.stop(),
            indexer_);
    }
    
}
