package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    public SparkMax motor1;
    public SparkMax motor2; 
    public SparkMax motor3;
    
    private RelativeEncoder encoder;
    
    // ==================== CONSTANTES ====================
    private static final double GEAR_RATIO = 250.0 / 14.0; // 17.857:1
    private static final double ROTATIONS_PER_360 = GEAR_RATIO;
    
    // Límites en rotaciones del encoder del NEO
    private static final double MAX_ROTATIONS = ROTATIONS_PER_360;  // +360°
    private static final double MIN_ROTATIONS = -ROTATIONS_PER_360; // -360°
    
    private static final double WRAP_TRIGGER_ANGLE = 330.0;  // A partir de 330° iniciar wrap
    private static final double WRAP_EXIT_ANGLE = 300.0;     // Hasta 300° seguir wrapping
    private static final double WRAP_SPEED = 0.5;            // Velocidad durante wrap
    
    private double homePosition = 0.0;
    private boolean isWrapping = false; // NUEVO - estado de wrapping
    private int wrapDirection = 0;      // NUEVO - dirección del wrap (-1 o 1)
    
    public Turret() {
        motor3 = new SparkMax(9, MotorType.kBrushless);
        
        encoder = motor3.getEncoder();
        encoder.setPosition(0);
        homePosition = 0.0;
        
        DriverStation.reportWarning("Torreta inicializada | Home position establecida", false);
    }

    /**
     * Gira la torreta aplicando soft limits
     * @param speed Velocidad de -1.0 a 1.0
     */
    public void rotate(double speed) {
        double currentPosition = encoder.getPosition();
        
        // Verificar límites duros
        if (isAtHardLimit(speed, currentPosition)) {
            motor3.set(0);
            
            if (Math.random() < 0.01) {
                DriverStation.reportWarning(
                    String.format("⚠️ Torreta en LÍMITE DURO | Pos: %.2f rot (%.1f°)", 
                        currentPosition, getAngleDegrees()),
                    false
                );
            }
        } else {
            motor3.set(speed);
        }
    }
    
    /**
     * Verifica si estamos en un límite duro y tratando de ir más allá
     */
    private boolean isAtHardLimit(double speed, double position) {
        if (speed > 0 && position >= MAX_ROTATIONS) {
            return true;
        }
        if (speed < 0 && position <= MIN_ROTATIONS) {
            return true;
        }
        return false;
    }
    
    /**
     * Calcula comando inteligente con wrapping automático
     * Esta es la función CLAVE que maneja todo el comportamiento
     */
    public double getSmartRotationCommand(double targetTx, double kP) {
        double currentAngle = getAngleDegrees();
        double absAngle = Math.abs(currentAngle);
        
        // ========== MODO WRAPPING ACTIVO ==========
        if (isWrapping) {
            // Seguir girando en la dirección del wrap hasta salir de la zona
            if (absAngle < WRAP_EXIT_ANGLE) {
                // Ya salimos de la zona crítica
                isWrapping = false;
                wrapDirection = 0;
                DriverStation.reportWarning(
                    String.format("✓ Wrap completado | Ángulo: %.1f°", currentAngle),
                    false
                );
                
                // Ahora sí usar el comando del Limelight
                return -kP * targetTx;
            } else {
                // Continuar wrapping
                return wrapDirection * WRAP_SPEED;
            }
        }
        
        // ========== DETECTAR SI DEBEMOS ENTRAR EN MODO WRAPPING ==========
        if (absAngle >= WRAP_TRIGGER_ANGLE) {
            // Estamos cerca del límite
            
            // Calcular hacia dónde quiere ir el Limelight
            double limelightCommand = -kP * targetTx;
            
            // Si el Limelight quiere seguir hacia el límite, ACTIVAR WRAP
            if ((currentAngle > 0 && limelightCommand > 0) ||  // Cerca de +360° y quiere seguir +
                (currentAngle < 0 && limelightCommand < 0)) {  // Cerca de -360° y quiere seguir -
                
                // ACTIVAR MODO WRAPPING
                isWrapping = true;
                wrapDirection = (currentAngle > 0) ? -1 : 1; // Invertir dirección
                
                DriverStation.reportWarning(
                    String.format("🔄 WRAP ACTIVADO | Ángulo: %.1f° | Dirección: %s", 
                        currentAngle, (wrapDirection > 0 ? "DERECHA" : "IZQUIERDA")),
                    false
                );
                
                return wrapDirection * WRAP_SPEED;
            }
        }
        
        // ========== MODO NORMAL ==========
        // No estamos en zona crítica, usar comando normal del Limelight
        return -kP * targetTx;
    }
    
    /**
     * Verifica si estamos cerca de un límite
     */
    public boolean isNearLimit() {
        double absAngle = Math.abs(getAngleDegrees());
        return absAngle >= WRAP_TRIGGER_ANGLE;
    }
    
    /**
     * Fuerza la salida del modo wrapping (útil si algo falla)
     */
    public void cancelWrapping() {
        isWrapping = false;
        wrapDirection = 0;
    }
    
    /**
     * Resetea la posición actual como "home"
     */
    public void resetHome() {
        encoder.setPosition(0);
        homePosition = 0.0;
        isWrapping = false;
        wrapDirection = 0;
        DriverStation.reportWarning("Torreta: Home reseteada a posición actual", false);
    }
    
    /**
     * Obtiene el ángulo actual de la torreta en grados (relativo al home)
     */
    public double getAngleDegrees() {
        return (encoder.getPosition() / GEAR_RATIO) * 360.0;
    }
    
    /**
     * Obtiene la posición actual del encoder en rotaciones
     */
    public double getEncoderPosition() {
        return encoder.getPosition();
    }
    
    public void rotateLeft(double speed) {
        rotate(-Math.abs(speed));
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
        motor3.set(0);
        // NO resetear isWrapping aquí - debe completarse
    }
    
    @Override
    public void periodic() {
        // Actualizar SmartDashboard
        SmartDashboard.putNumber("Turret Angle (deg)", getAngleDegrees());
        SmartDashboard.putNumber("Turret Encoder (rot)", encoder.getPosition());
        SmartDashboard.putBoolean("Turret Near Limit", isNearLimit());
        SmartDashboard.putBoolean("Turret Wrapping", isWrapping);
        SmartDashboard.putNumber("Turret Wrap Direction", wrapDirection);
    }
}