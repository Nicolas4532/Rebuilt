package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

  public Drivetrain drivetrain;
  public Shooter shooter;
  public XboxController controller;
  public Intake intake;
  public double startTime;



  // Control de velocidad del drivetrain
  private double drivelimit = 1.0;
  private int lastPOV = -1; // para detectar cambios en la cruceta
  private boolean Estop = false;

  public Robot() {}

  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    shooter = new Shooter();
    intake = new Intake();
    controller = new XboxController(0);
 

    
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp(); // Guarda el tiempo en el que inició autónomo
  }

  @Override
  public void autonomousPeriodic() {
    double timeElapsed = Timer.getFPGATimestamp() - startTime;

    if (timeElapsed < 3.0) {
      drivetrain.drive(-0.5, 0); // Avanza hacia adelante por 3 segundos
    } else {
      drivetrain.drive(0, 0); // Se detiene después
    }
  }

  @Override
  public void teleopInit() {
    if (Estop) {
      drivelimit = 0.0;
    }
  }

  @Override
  public void teleopPeriodic() {

    // ----- TOGGLE con la cruceta hacia arriba -----
    int currentPOV = controller.getPOV();

    if (currentPOV == 0 && lastPOV != 0) {
      // Toggle entre 1.0 y 0.5
      if (drivelimit == 1.0) {
        drivelimit = 0.5;
      } else {
        drivelimit = 1.0;
      }
      System.out.println("driveLimit ahora es " + drivelimit);
    }

    // ----- EMERGENCY STOP con la cruceta hacia abajo -----
    if (currentPOV == 180 && lastPOV != 180) {
      Estop = !Estop; // cambia de true a false o de false a true
      System.out.println("!!!!EMERGENCY STOP " + (Estop ? "ACTIVADO" : "DESACTIVADO") + "!!!");
    }

    lastPOV = currentPOV; // actualizar el último POV

    // Usar drivelimit en el drivetrain
    drivetrain.drive(controller.getRightY() * drivelimit, controller.getLeftX() * drivelimit);

    // --------- Shooter ---------
    boolean rightTriggerPressed = controller.getRawAxis(3) > 0.05;
    boolean leftTriggerPressed = controller.getRawAxis(2) > 0.05;
    boolean leftBumperPressed = controller.getRawButton(5);
    boolean rightBumperPressed = controller.getRawButton(6);

    if (rightTriggerPressed) {
      shooter.reverse(-1.0);
      intake.shoot(-1);
    } else if (leftTriggerPressed) {
      intake.shoot(-1.0);
      shooter.shoot(-1);
    } else if (leftBumperPressed) {
      shooter.reverse(-1);
      intake.reverse(-1.0);
    } else {  
      intake.stop();
      shooter.stop();
    }
  }
}
    
  


  

