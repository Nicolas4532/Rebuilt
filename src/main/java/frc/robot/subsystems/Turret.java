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
    private static final double GEAR_RATIO = 250.0 / 14.0;
    private static final double ROTATIONS_PER_360 = GEAR_RATIO;
    
    private static final double MAX_ROTATIONS = ROTATIONS_PER_360;
    private static final double MIN_ROTATIONS = -ROTATIONS_PER_360;
    
    private static final double WRAP_TRIGGER_ANGLE = 345.0;
    private static final double WRAP_TARGET_TRAVEL = 360.0;
    private static final double WRAP_SEARCH_START = 100.0;
    private static final double WRAP_SPEED = 0.25;
    
    // ==================== VELOCIDAD DE SEGUIMIENTO ====================
    private double trackingSpeedMultiplier = 2.0;  // ✅ NUEVO - multiplicador de velocidad (1.0 = 100%)
    
    private double homePosition = 0.0;
    private boolean isWrapping = false;
    private int wrapDirection = 0;
    private double wrapStartAngle = 0.0;
    private double totalWrapTravel = 0.0;
    
    public Turret() {
        motor3 = new SparkMax(9, MotorType.kBrushless);
        
        encoder = motor3.getEncoder();
        encoder.setPosition(0);
        homePosition = 0.0;
        
        DriverStation.reportWarning("Torreta inicializada | Home position establecida", false);
    }
    
    /**
     * ✅ NUEVO - Establece la velocidad de seguimiento del April Tag
     * @param speed Multiplicador de velocidad (0.0 a 2.0, donde 1.0 = velocidad normal)
     */
    public void setTrackingSpeed(double speed) {
        trackingSpeedMultiplier = Math.max(0.1, Math.min(2.0, speed)); // Limitar entre 0.1 y 2.0
        SmartDashboard.putNumber("Turret Tracking Speed", trackingSpeedMultiplier);
    }
    
    /**
     * ✅ NUEVO - Obtiene la velocidad de seguimiento actual
     * @return Multiplicador de velocidad actual
     */
    public double getTrackingSpeed() {
        return trackingSpeedMultiplier;
    }
    
    public void rotate(double speed) {
        double currentPosition = encoder.getPosition();
        
        if (isAtHardLimit(speed, currentPosition)) {
            motor3.set(0);
        } else {
            motor3.set(speed);
        }
    }

    public double getWrapCommand(boolean targetVisible) {
        if (!isWrapping) {
            return 0.0;
        }
        
        double currentAngle = getAngleDegrees();
        totalWrapTravel = Math.abs(currentAngle - wrapStartAngle);
        double command = wrapDirection * WRAP_SPEED;
        
        // FASE 1: 0° - 100° → Solo girar, ignorar Limelight
        if (totalWrapTravel < WRAP_SEARCH_START) {
            SmartDashboard.putString("Turret Status", 
                String.format("WRAPPING FASE 1: %.1f° / %.0f° (ignorando target)", 
                    totalWrapTravel, WRAP_SEARCH_START));
            return command;
        }
        
        // FASE 2: 100° - 300° → Girar Y buscar target
        if (targetVisible) {
            isWrapping = false;
            wrapDirection = 0;
            DriverStation.reportWarning(
                String.format("✓ Wrap completado (TARGET ENCONTRADO) | %.1f° → %.1f° (viajó %.1f°)", 
                    wrapStartAngle, currentAngle, totalWrapTravel),
                false
            );
            totalWrapTravel = 0.0;
            return 0.0;
        }
        
        if (totalWrapTravel >= WRAP_TARGET_TRAVEL) {
            isWrapping = false;
            wrapDirection = 0;
            DriverStation.reportWarning(
                String.format("✓ Wrap completado (DISTANCIA MÁXIMA) | %.1f° → %.1f° (viajó %.1f°)", 
                    wrapStartAngle, currentAngle, totalWrapTravel),
                false
            );
            totalWrapTravel = 0.0;
            return 0.0;
        }
        
        SmartDashboard.putString("Turret Status", 
            String.format("WRAPPING FASE 2: %.1f° / %.0f° (buscando target: %s)", 
                totalWrapTravel, WRAP_TARGET_TRAVEL, targetVisible ? "SÍ" : "NO"));
        
        return command;
    }
    
    public boolean isInSearchPhase() {
        if (!isWrapping) {
            return false;
        }
        return totalWrapTravel >= WRAP_SEARCH_START;
    }
    
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
     * ✅ MODIFICADO - Ahora aplica el multiplicador de velocidad
     */
    public double getSmartRotationCommand(double targetTx, double kP, boolean targetVisible) {
        double currentAngle = getAngleDegrees();
        double absAngle = Math.abs(currentAngle);
        
        if (!isWrapping && absAngle >= WRAP_TRIGGER_ANGLE) {
            double limelightCommand = -kP * targetTx * trackingSpeedMultiplier; // ✅ Aplicar multiplicador
            
            if (targetVisible && 
                ((currentAngle > 0 && limelightCommand > 0) ||  
                 (currentAngle < 0 && limelightCommand < 0))) {
                
                isWrapping = true;
                wrapStartAngle = currentAngle;
                totalWrapTravel = 0.0;
                wrapDirection = (currentAngle > 0) ? -1 : 1;
                
                DriverStation.reportWarning(
                    String.format("🔄 WRAP ACTIVADO | Desde: %.1f° | Dirección: %s", 
                        currentAngle, 
                        (wrapDirection > 0 ? "DERECHA (+)" : "IZQUIERDA (-)")),
                    false
                );
                
                return 0.0;
            }
        }
        
        SmartDashboard.putString("Turret Status", 
            String.format("NORMAL: %.1f° | tx: %.2f | tv: %s | speed: %.1f%%", 
                currentAngle, targetTx, targetVisible ? "SÍ" : "NO", 
                trackingSpeedMultiplier * 100)); // ✅ Mostrar velocidad en SmartDashboard
        
        return -kP * targetTx * trackingSpeedMultiplier; // ✅ Aplicar multiplicador
    }
    
    public boolean isNearLimit() {
        double absAngle = Math.abs(getAngleDegrees());
        return absAngle >= WRAP_TRIGGER_ANGLE;
    }
    
    public boolean isWrapping() {
        return isWrapping;
    }
    
    public void cancelWrapping() {
        isWrapping = false;
        wrapDirection = 0;
        wrapStartAngle = 0.0;
        totalWrapTravel = 0.0;
        DriverStation.reportWarning("Wrapping cancelado manualmente", false);
    }
    
    public void resetHome() {
        encoder.setPosition(0);
        homePosition = 0.0;
        isWrapping = false;
        wrapDirection = 0;
        wrapStartAngle = 0.0;
        totalWrapTravel = 0.0;
        DriverStation.reportWarning("Torreta: Home reseteada a posición actual", false);
    }
    
    public double getAngleDegrees() {
        return (encoder.getPosition() / GEAR_RATIO) * 360.0;
    }
    
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
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle (deg)", getAngleDegrees());
        SmartDashboard.putNumber("Turret Encoder (rot)", encoder.getPosition());
        SmartDashboard.putBoolean("Turret Near Limit", isNearLimit());
        SmartDashboard.putBoolean("Turret Wrapping", isWrapping);
        SmartDashboard.putNumber("Turret Wrap Direction", wrapDirection);
        SmartDashboard.putBoolean("Turret In Search Phase", isInSearchPhase());
        SmartDashboard.putNumber("Turret Tracking Speed", trackingSpeedMultiplier); // ✅ NUEVO
        
        if (isWrapping) {
            SmartDashboard.putNumber("Turret Wrap Travel", totalWrapTravel);
            SmartDashboard.putNumber("Turret Wrap Start", wrapStartAngle);
        }
    }
}