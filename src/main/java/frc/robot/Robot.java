package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.Turret;

//Variables Globales
public class Robot extends TimedRobot {

  public double homePosition;
  public double kP = 0.005;
  public NetworkTable limelight;
  public double kAim = 0.0015;   // ganancia para apuntar (se ajusta)
  public Pigeon2 pigeon;
  

  public Drivetrain drivetrain;
  public Turret turret;
  public XboxController controller;
  public double startTime;
  public IO io;
  public RelativeEncoder encoder;



  // Control de velocidad del drivetrain

  public Robot() {}

  @Override
  public void robotInit() {
    turret = new Turret();
    drivetrain = new Drivetrain();
    io = new IO();
    controller = new XboxController(0);

    encoder = turret.motor3.getEncoder(); // SIN RelativeEncoder adelante
    encoder.setPosition(0);
    homePosition = encoder.getPosition();

    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    pigeon = new Pigeon2(0); // CAN ID correcto
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

    pigeon.setYaw(0);

    drivetrain.drive(controller.getRightY(), controller.getLeftX());

    double RightStick = controller.getRightX();
    boolean RightStickActive = Math.abs(RightStick) > 0.07;

    boolean rightTriggerPressed = controller.getRawAxis(3) > 0.05;
    boolean leftTriggerPressed = controller.getRawAxis(2) > 0.05;
    boolean leftBumperPressed = controller.getRawButton(5);
    boolean autoAim = controller.getRightBumper();

    double tx = limelight.getEntry("tx").getDouble(0.0);
    double tv = limelight.getEntry("tv").getDouble(0.0);

    if (tv == 1) { // ve un AprilTag
    double error = tx;   // queremos tx = 0
    double output = kAim * error;

    // limitar velocidad
    output = Math.max(-0.3, Math.min(0.3, output));

    turret.rotate(output);
} else {
    turret.stop(0);
}

    if (rightTriggerPressed) {
      io.shoot(1);
    } else if (leftTriggerPressed) {
      io.intake(1);
    } else if (controller.getBButton()) {
      io.outtake(1);
    } else {  
      io.stop(1);
    }

if (autoAim && tv == 1) {
    double output = kAim * tx;
    output = Math.max(-0.3, Math.min(0.3, output));
    turret.rotate(output);
} else {
    turret.stop(0);
}

  }
}
