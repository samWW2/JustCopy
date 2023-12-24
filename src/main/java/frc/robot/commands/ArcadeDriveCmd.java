package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction)
    {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get() ;
        double realTimeTurn = turnFunction.get() ;
        
       
        driveSubsystem.arcadeDrive(realTimeSpeed, -realTimeTurn);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
