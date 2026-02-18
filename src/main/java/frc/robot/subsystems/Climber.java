package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public SparkMax motor1;

    public Climber() {
    motor1 = new SparkMax(10, MotorType.kBrushed);
    }

    public void climb(double speed){
        motor1.set(-1);
    }
    public void lower(double speed){
        motor1.set(1);
    }
    public void stop(double speed){
        motor1.set(0);
    }
}
