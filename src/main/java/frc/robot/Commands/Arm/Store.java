package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;

public class Store extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Arm m_Arm;
    private ArmStates m_ArmStates;
    private double m_Speed;
    public Store(Arm subsystem) {
        m_Arm = subsystem;
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {        
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_Arm.setStates(ArmStates.STORE);
      }

      @Override
        public void end(boolean interrupted) {
            // m_Arm.idleSpeed();
        
        }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

//limelight procedure
//1. Have driver select with button what to target: amp, speaker or source  
//2. Based on alliance and target, pick apriltag id number
//3. Look for the apriltag, and retrieve the distance and maybe angle

}