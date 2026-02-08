package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.Gyro;

public class Robot extends TimedRobot {
  
  // ==================== VARIABLES AUTÓNOMO ====================
  private int autoCommandIndex = 0; // Índice del comando actual
  private double commandStartTime = 0.0; // Tiempo de inicio del comando actual
  
  // ==================== CONSTANTES LL ====================
  private static final double kTurretKP = 0.006;
  private static final double kHoldKP = 0.003;
  private static final double kDeadband = 0.02;
  private static final double kMaxTurretSpeed = 1;
  
  // ==================== HARDWARE ====================
  private Drivetrain drivetrain;
  private Turret turret;
  private IO io;
  private Gyro gyro;
  private XboxController controller;
  
  private NetworkTable limelight;
  private RelativeEncoder turretEncoder;
  
  // ==================== ESTADO ====================
  private double holdPosition = 0;
  public double lastPrintTime = 0.0;
  
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();
    turret = new Turret();
    io = new IO();
    gyro = new Gyro();
    controller = new XboxController(0);
    
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    turretEncoder = turret.motor3.getEncoder();
    turretEncoder.setPosition(0);
    holdPosition = turretEncoder.getPosition();
    
    DriverStation.reportWarning("Robot iniciado correctamente", false);
  }
  
  @Override
  public void teleopPeriodic() {
    
    // ==================== BOTONES ====================
    boolean rightBumper = controller.getRightBumperPressed();
    boolean leftBumper = controller.getLeftBumperPressed();
    
    // ==================== AUTO-TURN ====================
    if (rightBumper) {
      gyro.rotate(90);
    }
    
    if (leftBumper) {
      gyro.rotate(-90);
    }
    
    // ==================== DRIVETRAIN ====================
    double forward = controller.getRightY();
    double turn = controller.getLeftX();
    
    if (gyro.isAutoTurnActive()) {
      turn = gyro.getAutoTurnCommand();
    }
    
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
    
    double turretCmd = 0;
    
    if (tv == 1) {
      if (Math.abs(tx) > kDeadband) {
        turretCmd = turret.getSmartRotationCommand(tx, kTurretKP);
      }
      holdPosition = turretEncoder.getPosition();
    } else {
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
    SmartDashboard.putNumber("Turret Angle", turret.getAngleDegrees());
    SmartDashboard.putNumber("Turret Cmd", turretCmd);
    
    double now = Timer.getFPGATimestamp();
    if (now - lastPrintTime > 0.5) {
      lastPrintTime = now;
      DriverStation.reportWarning(
        String.format("Gyro | Yaw: %.2f | Pitch: %.2f | Roll: %.2f", 
          gyro.getYaw(), gyro.getPitch(), gyro.getRoll()),
        false
      );
    }
  }
  
  // ================================================================
  // ==================== AUTÓNOMO SIMPLIFICADO ====================
  // ================================================================
  
@Override
public void autonomousInit() {
  gyro.resetYaw(); // El frente ACTUAL es ahora 0°
  autoCommandIndex = 0;
  commandStartTime = 0; // ✅ CAMBIO: Debe empezar en 0, no en el timestamp
  DriverStation.reportWarning("=== AUTÓNOMO INICIADO | Frente reseteado a 0° ===", false);
}
  
  @Override
  public void autonomousPeriodic() {
    
    // ╔═══════════════════════════════════════════════════════════╗
    // ║  MODIFICA TU AUTÓNOMO AQUÍ - ¡SUPER FÁCIL!                ║
    // ╚═══════════════════════════════════════════════════════════╝
    
    switch (autoCommandIndex) {
      
      case 0:
        rotate(45);        // Girar a 45° desde el frente
        break;
      
      case 1:
        wait(3.0);         // Esperar 1 segundo
        break;
      
      case 2:
        rotate(180);       // Girar a 180° (dar media vuelta desde el frente)
        break;
      
      case 3:
        wait(0.5);         // Esperar medio segundo
        break;
      
      case 4:
        rotate(0);         // Volver al frente (0°)
        break;
      
      // *** AGREGA MÁS COMANDOS AQUÍ SI QUIERES ***
      // case 5:
      //   rotate(-90);    // Girar 90° a la izquierda
      //   break;
      
      default:
        // Autónomo terminado
        drivetrain.drive(0, 0);
        break;
    }
    
    // ══════════════════════════════════════════════════════════
    // NO MODIFICAR DEBAJO DE ESTA LÍNEA
    // ══════════════════════════════════════════════════════════
    
    // Aplicar comandos del gyro
    double turn = gyro.getAutoTurnCommand();
    drivetrain.drive(0, turn);
    
    SmartDashboard.putNumber("Auto Command", autoCommandIndex);
  }
  
  // ==================== COMANDOS DE AUTÓNOMO ====================
  
  /**
   * Girar a un ángulo absoluto (relativo al frente inicial)
   * @param degrees Ángulo objetivo en grados
   */
  private void rotate(double degrees) {
    if (!commandStarted()) {
      gyro.rotateTo(degrees);
      commandStartTime = Timer.getFPGATimestamp();
      DriverStation.reportWarning(
        String.format("Comando %d: Girando a %.0f°", autoCommandIndex, degrees), 
        false
      );
    }
    
    if (gyro.isAutoTurnFinished()) {
      nextCommand();
    }
  }
  
  /**
   * Esperar un tiempo determinado
   * @param seconds Segundos a esperar
   */
  private void wait(double seconds) {
    if (!commandStarted()) {
      commandStartTime = Timer.getFPGATimestamp();
      DriverStation.reportWarning(
        String.format("Comando %d: Esperando %.1f segundos", autoCommandIndex, seconds), 
        false
      );
    }
    
    if (Timer.getFPGATimestamp() - commandStartTime >= seconds) {
      nextCommand();
    }
  }
  
  /**
   * Avanzar o retroceder durante un tiempo
   * @param speed Velocidad (-1.0 a 1.0, negativo = atrás)
   * @param seconds Duración en segundos
   */
  private void drive(double speed, double seconds) {
    if (!commandStarted()) {
      commandStartTime = Timer.getFPGATimestamp();
      DriverStation.reportWarning(
        String.format("Comando %d: Conduciendo a %.2f por %.1fs", autoCommandIndex, speed, seconds), 
        false
      );
    }
    
    if (Timer.getFPGATimestamp() - commandStartTime < seconds) {
      drivetrain.drive(speed, 0);
    } else {
      drivetrain.drive(0, 0);
      nextCommand();
    }
  }
  
  // ==================== HELPERS ====================
  
  private boolean commandStarted() {
    return commandStartTime > 0;
  }
  
  private void nextCommand() {
    autoCommandIndex++;
    commandStartTime = 0; // Marcar como no iniciado
    DriverStation.reportWarning(
      String.format("→ Comando completado. Siguiente: %d", autoCommandIndex), 
      false
    );
  }
}