package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

        public SparkMax leftMotorLeader; // Drivetrain cims izquierdo 1
        public SparkMax leftMotorFollower; // Drivetrain cims izquierdo 2
        public SparkMax rightMotorLeader; // Drivetrain cims derecho 1
        public SparkMax rightMotorFollower; // Drivetrain cims derecho 2

        public DifferentialDrive differentialDrive;

        public Drivetrain() {

                leftMotorLeader = new SparkMax(1, MotorType.kBrushed);
                leftMotorFollower = new SparkMax(2, MotorType.kBrushed);
                rightMotorLeader = new SparkMax(3, MotorType.kBrushed);
                rightMotorFollower = new SparkMax(4, MotorType.kBrushed);

                SparkMaxConfig baseConfig = new SparkMaxConfig();
                SparkMaxConfig leftMotorFollowerConfig = new SparkMaxConfig();
                SparkMaxConfig rightMotorLeaderConfig = new SparkMaxConfig();
                SparkMaxConfig rightMotorFollowerConfig = new SparkMaxConfig();

                baseConfig.idleMode(IdleMode.kCoast); // aqui se configura si es coast o brake

                leftMotorFollowerConfig
                                .apply(baseConfig)
                                .follow(leftMotorLeader);

                rightMotorLeaderConfig
                                .apply(baseConfig)
                                .inverted(true);

                rightMotorFollowerConfig
                                .apply(baseConfig)
                                .follow(rightMotorLeader);

                leftMotorLeader.configure(baseConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
                leftMotorFollower.configure(leftMotorFollowerConfig,
                                com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
                rightMotorLeader.configure(rightMotorLeaderConfig,
                                com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
                rightMotorFollower.configure(rightMotorFollowerConfig,
                                com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                differentialDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);
        }

        public void drive(double speed, double rotation) {
                differentialDrive.arcadeDrive(speed, rotation);
        }
}