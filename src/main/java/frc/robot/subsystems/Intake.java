package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public SparkMax motor; // NEO en outtake

    public Intake() {
        motor = new SparkMax(6, MotorType.kBrushed);
    }

    public void shoot(double speed) {
        motor.set(speed);
    }

    public void reverse(double speed) {
        motor.set(-speed);
    }

    public void stop() {
        motor.set(0);
    }
}
