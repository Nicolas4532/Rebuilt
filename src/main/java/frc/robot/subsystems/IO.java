package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IO extends SubsystemBase {

    public SparkMax motor1;
    public SparkMax motor2; 

    public IO() {
        motor1 = new SparkMax(7, MotorType.kBrushed);
        motor2 = new SparkMax(8, MotorType.kBrushed);
    }
    
    public void shoot(double speed) {
        motor1.set(-speed);
        motor2.set(speed);
    }

    public void intake(double speed) {
        motor1.set(-speed);
        motor2.set(-speed);
    }

    public void outtake(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    public void stop(double speed) {
        motor1.set(0);
        motor2.set(0);
    }

}

