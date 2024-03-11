package frc.robot.Commands.Arm;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class AutoArm extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Arm m_Arm;
  private final Shooter m_Shooter;
  private double m_targetAngle;
  
  private boolean armReachedTarget = false;
  private double[] autoTargets;
  private int shots;

  public AutoArm(Arm subsystem, Shooter shooter_subsystem) {//, double target
    m_Arm = subsystem;
    m_Shooter = shooter_subsystem;
    // m_targetAngle = target;
    // m_Speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, shooter_subsystem);
  }

  @Override
  public void initialize() {
    m_Arm.setTargetAngle(m_targetAngle);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("Finished moving arm");
    // m_Arm.setStates(ArmStates.STORE);
    SmartDashboard.putBoolean("Arm on Target", m_Arm.onTarget());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.autoOnTarget();
    /*
     * if(!m_BreakBeam.get() && m_ArmStates == ArmStates.INTAKE){
     * return true;
     * }else{
     * return false;
     * }
     */
  }

  // limelight procedure
  // 1. Have driver select with button what to target: amp, speaker or source
  // 2. Based on alliance and target, pick apriltag id number
  // 3. Look for the apriltag, and retrieve the distance and maybe angle

}
