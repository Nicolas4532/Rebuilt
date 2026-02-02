package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    public SparkMax motor1;
    public SparkMax motor2; 
    public SparkMax motor3;

    public Turret() {
        //motor1 = new SparkMax(7, MotorType.kBrushless);
        //motor2 = new SparkMax(8, MotorType.kBrushless);
        motor3 = new SparkMax(9, MotorType.kBrushless);
    }

    public void rotate(double speed) {
        motor3.set(speed);
    }
    public void rotateLeft(double speed) {
        motor3.set(speed);
    }
    public void shoot(double speed){
        //motor2.set(1);
    }
    public void hoodUp(double speed){
        //motor3.set(speed);
    }
    public void hoodDown(double speed){
        //motor3.set(-speed);
    }
    public void stop(double speed){
        //motor1.set(0);
        //motor2.set(0);
        motor3.set(0);
    }
}