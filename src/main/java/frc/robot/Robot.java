package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IO;

public class Robot extends TimedRobot {
  
  // ==================== CONSTANTES PIGEON ================
  private boolean autoTurnActive = false;
  private double targetYaw = 0.0;
  private static final double kTurnKP = 0.01;
  private static final double kYawTolerance = 1.0;
  public double lastPrintTime = 0.0;
  // ==================== CONSTANTES LL ====================
  private static final double kTurretKP = 0.006;   // Limelight → torreta
  private static final double kHoldKP   = 0.003;    // encoder hold
  private static final double kDeadband = 0.02;      // grados Limelight
  private static final double kMaxTurretSpeed = 1;
  private static final double kTurretMinRot = -8.93;
  private static final double kTurretMaxRot =  8.93;

  // ==================== HARDWARE ====================
  private Drivetrain drivetrain;
  private Turret turret;
  private IO io;
  private XboxController controller;

  private NetworkTable limelight;
  private RelativeEncoder turretEncoder;
  private Pigeon2 pigeon;

  // ==================== ESTADO ====================
  private double holdPosition = 0;

  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    turret = new Turret();
    io = new IO();
    controller = new XboxController(0);

    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    turretEncoder = turret.motor3.getEncoder();
    turretEncoder.setPosition(0);
    holdPosition = turretEncoder.getPosition();

    pigeon = new Pigeon2(0);
    pigeon.setYaw(0);

    DriverStation.reportWarning("Robot iniciado correctamente", false);
  }

  @Override
  public void teleopPeriodic() {
    
    //===================== BOTONES ========================
    boolean leftBumper = controller.getLeftBumperPressed();
    boolean rightBumper = controller.getRightBumper();
    //===================== PIGEON ========================
    double yaw = pigeon.getYaw().getValueAsDouble();
    double pitch = pigeon.getPitch().getValueAsDouble();
    double roll = pigeon.getRoll().getValueAsDouble();

    // ==================== DRIVETRAIN ====================
    double forward = controller.getRightY();
    double turn = controller.getLeftX();
    drivetrain.drive(forward, turn);

    // ==================== INTAKE / SHOOT ====================
    if (controller.getRawAxis(3) > 0.05) {
      io.shoot(1);
    } else if (controller.getRawAxis(2) > 0.05) {
      io.intake(1);
    } else if (controller.getBButton()) {
      io.outtake(1);
    } else {
      io.stop(1);
    }

    // ==================== LIMELIGHT ====================
    double tx = limelight.getEntry("tx").getDouble(999);
    double tv = limelight.getEntry("tv").getDouble(0);
    double ty = limelight.getEntry("ty").getDouble(999);

    double turretCmd = 0;

    if (tv == 1) {
      // -------- TARGET VISIBLE --------
      if (Math.abs(tx) > kDeadband) {
        turretCmd = -kTurretKP * tx;
      }

      holdPosition = turretEncoder.getPosition();

    } else {
      // -------- TARGET PERDIDO → HOLD --------
      double error = holdPosition - turretEncoder.getPosition();
      turretCmd = kHoldKP * error;
    }
    
    turretCmd = MathUtil.clamp(turretCmd, -kMaxTurretSpeed, kMaxTurretSpeed);

    if (Math.abs(turretCmd) < 0.02) {
      turret.stop(1);
    } else {
      turret.rotate(turretCmd);
    }
    // ==================== DASHBOARD ====================
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight tv", tv);
    SmartDashboard.putNumber("Turret Encoder", turretEncoder.getPosition());
    SmartDashboard.putNumber("Turret Cmd", turretCmd);

    double now = Timer.getFPGATimestamp();
  if (now - lastPrintTime > 0.5) {
      lastPrintTime = now;

      DriverStation.reportWarning(String.format("Pigeon | Yaw: %.2f | Pitch: %.2f | Roll: %.2f",yaw, pitch, roll),false);
  }

  }
}