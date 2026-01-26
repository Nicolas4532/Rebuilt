package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IO;
//import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {

  public Drivetrain drivetrain;
  //public Turret turret;
  public XboxController controller;
  public double startTime;
  public IO io;



  // Control de velocidad del drivetrain

  public Robot() {}

  @Override
  public void robotInit() {
    //turret = new Turret();
    drivetrain = new Drivetrain();
    io = new IO();
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
  }

  @Override
  public void teleopPeriodic() {

    drivetrain.drive(controller.getRightY(), controller.getLeftX());

    boolean rightTriggerPressed = controller.getRawAxis(3) > 0.05;
    boolean leftTriggerPressed = controller.getRawAxis(2) > 0.05;
    boolean leftBumperPressed = controller.getRawButton(5);
    boolean rightBumperPressed = controller.getRawButton(6);

    if (rightTriggerPressed) {
      io.shoot(1);
    } else if (leftTriggerPressed) {
      io.intake(1);
    } else if (controller.getBButton()) {
      io.outtake(1);
    } else {  
      io.stop(1);
    }

  }
}
