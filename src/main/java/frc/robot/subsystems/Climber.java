package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberDirection;

public class Climber extends SubsystemBase {
    private CANSparkFlex leftClimberFlex;
    private CANSparkFlex rightClimberFlex;
    private boolean leftAtBottom = false;
    private boolean rightAtBottom = false;
    private boolean leftAtTop = false;
    private boolean rightAtTop = false;

    private ClimberDirection m_ClimberDirection;

    public Climber() {
        // Spark Flex Instantiations
        leftClimberFlex = new CANSparkFlex(ClimberConstants.LeftClimberMotor, MotorType.kBrushless);
        rightClimberFlex = new CANSparkFlex(ClimberConstants.RightClimberMotor, MotorType.kBrushless);

        setMotorMode(IdleMode.kBrake);

        resetPosition();
    }

    public void setMotorMode(IdleMode mode) {
        leftClimberFlex.setIdleMode(mode);
        rightClimberFlex.setIdleMode(mode);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Left Climber Current",
        // leftClimberFlex.getOutputCurrent());
        // SmartDashboard.putNumber("Right Climber Current",
        // rightClimberFlex.getOutputCurrent());

        // SmartDashboard.putBoolean("left at Bottom", leftAtBottom);
        // SmartDashboard.putBoolean("right at Bottom", rightAtBottom);

        // SmartDashboard.putNumber("Left Climber Ticks", getLeftPosition());
        // SmartDashboard.putNumber("Right Climber Ticks", getRightPosition());

        // SmartDashboard.putBoolean("Left at Top", leftAtTop);
        // SmartDashboard.putBoolean("Right at Top", rightAtTop);
        // SmartDashboard.putNumber("Left Voltage", leftClimberFlex.getBusVoltage());
        // SmartDashboard.putNumber("Right Voltage", rightClimberFlex.getBusVoltage());

        // left max is -172.2
        // right max is -177.5
    }

    public void runMotors(ClimberDirection direction) {
        m_ClimberDirection = direction;

        switch (m_ClimberDirection) {
            case UP:
                if (getLeftPosition() >= ClimberConstants.LeftClimberTopPos) { // Stop the Motors when it reaches the
                                                                               // top position
                    leftClimberFlex.set(0);
                } else {
                    leftClimberFlex.set(ClimberConstants.ClimberUpSpeed);
                }
                if (getRightPosition() >= ClimberConstants.RightClimberTopPos) {
                    rightClimberFlex.set(0);
                } else {
                    rightClimberFlex.set((ClimberConstants.ClimberUpSpeed));
                }
                break;
            case DOWN:
                if (leftClimberFlex.getOutputCurrent() > 60 && getRightPosition() < 20) { // Check if the Motor is
                                                                                          // getting stalled and if it's
                                                                                          // at a low position to stop
                                                                                          // the climber
                    leftAtBottom = true;
                }
                if (rightClimberFlex.getOutputCurrent() > 60 && getRightPosition() < 20) {
                    rightAtBottom = true;
                }
                if (leftAtBottom) {
                    leftClimberFlex.set(0);
                } else {
                    leftClimberFlex.set(ClimberConstants.ClimberDownSpeed);
                }
                if (rightAtBottom) {
                    rightClimberFlex.set(0);
                } else {
                    rightClimberFlex.set(ClimberConstants.ClimberDownSpeed);
                }
                break;

        }
    }

    public void stopMotors() {
        leftClimberFlex.set(0);
        rightClimberFlex.set(0);
    }

    public void resetLeft() {
        leftAtBottom = false;
        leftAtTop = false;
    }

    public void resetRight() {
        rightAtBottom = false;
        rightAtTop = false;
    }

    public double getLeftPosition() {
        return leftClimberFlex.getEncoder().getPosition();
    }

    public double getRightPosition() {
        return rightClimberFlex.getEncoder().getPosition();
    }

    public void resetPosition() {
        leftClimberFlex.getEncoder().setPosition(0);
        rightClimberFlex.getEncoder().setPosition(0);
    }

    public boolean atEndPos(ClimberDirection dir) {
        if (dir == ClimberDirection.UP) {
            return rightAtTop && leftAtTop;
        } else {
            return rightAtBottom && leftAtBottom;
        }

    }

}