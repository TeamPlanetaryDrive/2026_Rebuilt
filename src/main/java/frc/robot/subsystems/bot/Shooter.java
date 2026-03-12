package frc.robot.subsystems.bot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX motor1; 
    private final TalonFX motor2; 
    private final TalonFX motor3; 

    public Shooter(){
        int placeholderCANID = 0;
        motor1 = new TalonFX(placeholderCANID);
        motor2 = new TalonFX(placeholderCANID);
        motor3 = new TalonFX(placeholderCANID);
    }
}
