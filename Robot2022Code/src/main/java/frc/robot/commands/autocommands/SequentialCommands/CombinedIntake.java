package frc.robot.commands.autocommands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IndexerMotorCommand;
import frc.robot.commands.Ramsete930Command;
import frc.robot.commands.intakecommands.intakePistonCommands.EngageIntakePistonsCommand;
import frc.robot.commands.intakecommands.intakemotorcommands.RunIntakeMotorsCommand;
import frc.robot.subsystems.IndexerMotorSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.IntakePistonSubsystem;

public class CombinedIntake extends ParallelRaceGroup{

    public CombinedIntake(
        IntakePistonSubsystem intakePistonSubsystem,
        IntakeMotorSubsystem intakeMotorSubsystem,
        IndexerMotorSubsystem indexerMotorSubsystem,
        Ramsete930Command path){

        addCommands(
            new EngageIntakePistonsCommand(intakePistonSubsystem),
            new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
            new IndexerMotorCommand(indexerMotorSubsystem, false),
            path
        );
    }

    public CombinedIntake(
        IntakePistonSubsystem intakePistonSubsystem,
        IntakeMotorSubsystem intakeMotorSubsystem,
        IndexerMotorSubsystem indexerMotorSubsystem){

        addCommands(
            new EngageIntakePistonsCommand(intakePistonSubsystem),
            new RunIntakeMotorsCommand(intakeMotorSubsystem, false),
            new IndexerMotorCommand(indexerMotorSubsystem, false)
        );
    }
}
