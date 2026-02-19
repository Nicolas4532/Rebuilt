package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.IO;
import frc.robot.subsystems.Gyro;

public class Robot extends TimedRobot {
  
  // ==================== VARIABLES AUTÓNOMO ====================
  private int autoCommandIndex = 0;
  private double commandStartTime = 0.0;
  
  // ==================== CONSTANTES LL ====================
  private static final double kTurretKP = 0.006;
  private static final double kHoldKP = 0.003;
  private static final double kDeadband = 0.02;
  private static final double kMaxTurretSpeed = 1;
  
  // ==================== APRIL TAG IDs ====================
  private static final int RED_ALLIANCE_TAG = 1;
  private static final int BLUE_ALLIANCE_TAG = 26;
  private int targetTagID = RED_ALLIANCE_TAG;
  
  // ==================== HARDWARE ====================
  private Drivetrain drivetrain;
  private Turret turret;
  private IO io;
  private Gyro gyro;
  private XboxController controller;
  private Climber climber;
  
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
    climber = new Climber();
    
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    
    turretEncoder = turret.motor3.getEncoder();
    turretEncoder.setPosition(0);
    holdPosition = turretEncoder.getPosition();
    
    // ==================== SHUFFLEBOARD SETUP ====================
    ShuffleboardTab allianceTab = Shuffleboard.getTab("Alliance Select");
    
    // Botón para alianza ROJA
    
    allianceTab.add("🔴 RED Alliance (Tag 10)", new Command() {
        @Override
        public void initialize() {
            targetTagID = RED_ALLIANCE_TAG;
            DriverStation.reportWarning("🔴 Cambiado a ALIANZA ROJA - Tag " + RED_ALLIANCE_TAG, false);
        }
        
        @Override
        public boolean isFinished() {
            return true;
        }
    }).withSize(2, 2).withPosition(0, 0);
    
    // Botón para alianza AZUL
    allianceTab.add("🔵 BLUE Alliance (Tag 26)", new Command() {
        @Override
        public void initialize() {
            targetTagID = BLUE_ALLIANCE_TAG;
            DriverStation.reportWarning("🔵 Cambiado a ALIANZA AZUL - Tag " + BLUE_ALLIANCE_TAG, false);
        }
        
        @Override
        public boolean isFinished() {
            return true;
        }
    }).withSize(2, 2).withPosition(2, 0);
    
    // Mostrar tag actual
    allianceTab.addNumber("Current Target Tag", () -> targetTagID)
        .withSize(2, 1).withPosition(0, 2);
    
    allianceTab.addString("Current Alliance", () -> {
        if (targetTagID == RED_ALLIANCE_TAG) return "🔴 RED";
        else if (targetTagID == BLUE_ALLIANCE_TAG) return "🔵 BLUE";
        else return "⚪ CUSTOM";
    }).withSize(2, 1).withPosition(2, 2);
    
    // Tab de telemetría
    ShuffleboardTab telemetryTab = Shuffleboard.getTab("Telemetry");
    telemetryTab.addNumber("Limelight tx", () -> limelight.getEntry("tx").getDouble(999));
    telemetryTab.addNumber("Limelight tid", () -> limelight.getEntry("tid").getDouble(-1));
    telemetryTab.addBoolean("Correct Tag Visible", () -> {
        double tv = limelight.getEntry("tv").getDouble(0);
        double tid = limelight.getEntry("tid").getDouble(-1);
        return (tv == 1) && (tid == targetTagID);
    });
    telemetryTab.addNumber("Turret Angle", () -> turret.getAngleDegrees());
    telemetryTab.addBoolean("Turret Wrapping", () -> turret.isWrapping());
    
    // Intentar detectar alianza automáticamente del FMS
    updateAllianceTag();
    
    DriverStation.reportWarning("Robot iniciado correctamente", false);
  }
  
  /**
   * ✅ NUEVO - Detecta alianza automáticamente del FMS
   */
  private void updateAllianceTag() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        targetTagID = RED_ALLIANCE_TAG;
        DriverStation.reportWarning("🔴 FMS detectó ALIANZA ROJA - Tag " + RED_ALLIANCE_TAG, false);
      } else if (alliance.get() == Alliance.Blue) {
        targetTagID = BLUE_ALLIANCE_TAG;
        DriverStation.reportWarning("🔵 FMS detectó ALIANZA AZUL - Tag " + BLUE_ALLIANCE_TAG, false);
      }
    } else {
      DriverStation.reportWarning("⚠️ No se pudo detectar alianza del FMS - usando Tag " + targetTagID, false);
    }
    
    // Actualizar SmartDashboard
    SmartDashboard.putNumber("Target Tag ID", targetTagID);
  }
  
  /**
   * Lee el Target Tag ID desde SmartDashboard (permite cambio manual)
   */
  private void updateTargetFromDashboard() {
    int dashboardTag = (int) SmartDashboard.getNumber("Target Tag ID", targetTagID);
    if (dashboardTag != targetTagID) {
      targetTagID = dashboardTag;
      DriverStation.reportWarning("📝 Tag objetivo cambiado manualmente a: " + targetTagID, false);
    }
  }
  
  @Override
  public void teleopInit() {
    // Actualizar tag al inicio de teleop
    updateAllianceTag();
  }
  
  @Override
  public void autonomousInit() {
    gyro.resetYaw();
    autoCommandIndex = 0;
    commandStartTime = 0;
    
    // Actualizar tag al inicio del autónomo
    updateAllianceTag();
    
    DriverStation.reportWarning("=== AUTÓNOMO INICIADO | Frente reseteado a 0° ===", false);
  }
  
  @Override
  public void teleopPeriodic() {
    
    // ==================== BOTONES ====================
    boolean rightBumper = controller.getRightBumperPressed();
    boolean leftBumper = controller.getLeftBumperPressed();
    boolean Abutton = controller.getAButtonPressed();
    boolean Bbutton = controller.getBButtonPressed();
    boolean Xbutton = controller.getXButton();
    boolean Ybutton = controller.getYButton();
    double RightX = controller.getRightX();
    boolean RightXactive = Math.abs(RightX) > 0.7;

    // ==================== RESET TORRETA ====================
    if (Abutton) {
      turret.resetHome();
      DriverStation.reportWarning("🔄 Home de la torreta reseteado manualmente", false);
    }

    // ==================== CLIMBER ====================
    if (Ybutton) {
      climber.climb(.1);
    } else if (Xbutton) {
      climber.lower(.1);
    } else {
      climber.stop(1);
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
    
    // ==================== LIMELIGHT CON FILTRO DE TAG ====================
    double tx = limelight.getEntry("tx").getDouble(999);
    double tv = limelight.getEntry("tv").getDouble(0);
    double tid = limelight.getEntry("tid").getDouble(-1);
    
    // Solo considerar válido si es el tag correcto
    boolean targetVisible = (tv == 1) && (tid == targetTagID);
    
    double turretCmd = 0;

    // ==================== MANUAL OVERRIDE ====================
    if (leftBumper && RightXactive) {
      turretCmd = RightX * 0.5;
      
    } else {
      // ==================== MODO AUTOMÁTICO ====================
      
      if (!turret.isWrapping() && targetVisible) {
        turret.getSmartRotationCommand(tx, kTurretKP, targetVisible);
      }
      
      if (turret.isWrapping()) {
        turretCmd = turret.getWrapCommand(targetVisible);
        
      } else if (targetVisible) {
        if (Math.abs(tx) > kDeadband) {
          turretCmd = -kTurretKP * tx;
        }
        holdPosition = turretEncoder.getPosition();
        
      } else {
        double error = holdPosition - turretEncoder.getPosition();
        turretCmd = kHoldKP * error;
      }
    }

    // Aplicar límites
    turretCmd = MathUtil.clamp(turretCmd, -kMaxTurretSpeed, kMaxTurretSpeed);

    // Ejecutar el comando
    if (Math.abs(turretCmd) >= 0.02 || (leftBumper && RightXactive)) {
      turret.rotate(turretCmd);
    } else {
      turret.stop(1);
    }
    
    // ==================== DASHBOARD ====================
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight tv", tv);
    SmartDashboard.putNumber("Limelight tid", tid);
    SmartDashboard.putBoolean("Correct Tag Visible", targetVisible);
    SmartDashboard.putNumber("Target Tag ID", targetTagID);
    SmartDashboard.putNumber("Turret Encoder", turretEncoder.getPosition());
    SmartDashboard.putNumber("Turret Angle", turret.getAngleDegrees());
    SmartDashboard.putNumber("Turret Cmd", turretCmd);
  }
  
  @Override
  public void autonomousPeriodic() {
    switch (autoCommandIndex) {
      case 0:
        drive(1,4 );
        break;
      case 1:
        wait(3.0);
        break;
      case 2:
      rotate (-90);
      drive(1,3);
      break;
      case 3:
      rotate(-90);
      break;
      case 4:
      drive( 1,3);
      default:
        drivetrain.drive(0, 0);
        break;
    }
    
    double turn = gyro.getAutoTurnCommand();
    drivetrain.drive(0, turn);
    SmartDashboard.putNumber("Auto Command", autoCommandIndex);
  }
  
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
  
  private boolean commandStarted() {
    return commandStartTime > 0;
  }
  
  private void nextCommand() {
    autoCommandIndex++;
    commandStartTime = 0;
    DriverStation.reportWarning(
      String.format("→ Comando completado. Siguiente: %d", autoCommandIndex), 
      false
    );
  }
}