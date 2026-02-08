package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    
    // ==================== HARDWARE ====================
    private final Pigeon2 pigeon;
    
    // ==================== CONSTANTES AUTO-TURN ====================
    private static final double kTurnSpeed = 0.5;
    private static final double kTurnSlowSpeed = 0.2;
    private static final double kSlowdownAngle = 25.0;
    private static final double kYawTolerance = 2.0;
    private static final double kStopTolerance = 6.0;
    private static final double kSettleTime = 0.15;
    private static final double kMaxOscillations = 3;
    private static final double kMaxOscillationTime = 2.0;
    
    // ==================== ESTADO ====================
    private boolean autoTurnActive = false;
    private double targetYaw = 0.0;
    private double settleTimer = 0.0;
    private double oscillationTimer = 0.0;
    private double lastErrorSign = 0.0;
    private int oscillationCount = 0;
    private double lastError = 0.0;
    
    public Gyro() {
        pigeon = new Pigeon2(0);
        pigeon.setYaw(0);
        DriverStation.reportWarning("Gyro inicializado correctamente", false);
    }
    
    /**
     * Inicia un giro automático relativo desde la posición actual
     * Solo se activa si NO hay un auto-turn en progreso
     * @param degrees Grados a girar (+ derecha, - izquierda)
     */
    public void rotate(double degrees) {
        if (autoTurnActive) {
            return;
        }
        
        autoTurnActive = true;
        targetYaw = getYaw() + degrees;
        settleTimer = 0.0;
        oscillationCount = 0;
        oscillationTimer = 0.0;
        lastErrorSign = 0.0;
        lastError = 999;
        
        DriverStation.reportWarning(
            String.format("Auto-turn iniciado | Actual: %.2f° | Target: %.2f° | Girando: %.2f°", 
                getYaw(), targetYaw, degrees), 
            false
        );
    }
    
    /**
     * Gira a un ángulo absoluto específico
     * @param absoluteYaw Ángulo objetivo absoluto
     */
    public void rotateTo(double absoluteYaw) {
        if (autoTurnActive) {
            return;
        }
        
        autoTurnActive = true;
        targetYaw = absoluteYaw;
        settleTimer = 0.0;
        oscillationCount = 0;
        oscillationTimer = 0.0;
        lastErrorSign = 0.0;
        lastError = 999;
        
        DriverStation.reportWarning(
            String.format("Auto-turn a ángulo absoluto | Target: %.2f°", absoluteYaw), 
            false
        );
    }
    
    /**
     * Calcula el comando de giro para auto-turn
     * @return Comando de giro (-1.0 a 1.0), o 0 si auto-turn no está activo
     */
    public double getAutoTurnCommand() {
        if (!autoTurnActive) {
            return 0.0;
        }
        
        double yaw = getYaw();
        double error = targetYaw - yaw;
        
        // Normalizar error (-180 a 180)
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        double absError = Math.abs(error);
        double turn = 0.0;
        
        // Detectar oscilación por cambio de signo
        if (lastErrorSign != 0 && Math.signum(error) != lastErrorSign) {
            oscillationCount++;
            DriverStation.reportWarning(
                String.format("Oscilación #%d | Error: %.2f°", oscillationCount, error),
                false
            );
        }
        lastErrorSign = Math.signum(error);
        
        // Detectar oscilación por falta de progreso
        if (Math.abs(lastError - absError) < 0.5) {
            oscillationTimer += 0.02;
        } else {
            oscillationTimer = 0.0;
        }
        lastError = absError;
        
        // Cancelar si oscilamos demasiado O si llevamos mucho tiempo sin progresar
        if (oscillationCount >= kMaxOscillations || oscillationTimer >= kMaxOscillationTime) {
            autoTurnActive = false;
            DriverStation.reportWarning(
                String.format("⚠️ Auto-turn CANCELADO | Oscilaciones: %d | Tiempo estancado: %.2fs | Error: %.2f°", 
                    oscillationCount, oscillationTimer, error),
                false
            );
            return 0.0;
        }
        
        // Zona de parada completa
        if (absError < kStopTolerance) {
            turn = 0.0;
            
            if (absError < kYawTolerance) {
                settleTimer += 0.02;
                
                if (settleTimer >= kSettleTime) {
                    autoTurnActive = false;
                    DriverStation.reportWarning(
                        String.format("✓ Auto-turn completado | Error: %.2f°", error), 
                        false
                    );
                }
            } else {
                settleTimer = 0.0;
            }
        }
        // Fase de frenado gradual
        else if (absError <= kSlowdownAngle) {
            double speedFactor = absError / kSlowdownAngle;
            turn = Math.copySign(kTurnSlowSpeed + (kTurnSpeed - kTurnSlowSpeed) * speedFactor, error);
            settleTimer = 0.0;
        }
        // Fase de velocidad máxima
        else {
            turn = Math.copySign(kTurnSpeed, error);
            settleTimer = 0.0;
        }
        
        return turn;
    }
    
    /**
     * Cancela el auto-turn actual
     */
    public void cancelAutoTurn() {
        autoTurnActive = false;
        settleTimer = 0.0;
        oscillationCount = 0;
        oscillationTimer = 0.0;
        DriverStation.reportWarning("Auto-turn cancelado manualmente", false);
    }
    
    /**
     * @return true si el auto-turn está activo
     */
    public boolean isAutoTurnActive() {
        return autoTurnActive;
    }
    
    /**
     * Verifica si el auto-turn ha terminado (ya sea exitoso o cancelado)
     * @return true si terminó o nunca estuvo activo
     */
    public boolean isAutoTurnFinished() {
        return !autoTurnActive;
    }
    
    /**
     * @return Ángulo yaw actual en grados
     */
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }
    
    /**
     * @return Ángulo pitch actual en grados
     */
    public double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }
    
    /**
     * @return Ángulo roll actual en grados
     */
    public double getRoll() {
        return pigeon.getRoll().getValueAsDouble();
    }
    
    /**
     * Resetea el yaw a 0 (o al ángulo especificado)
     */
    public void resetYaw(double angle) {
        pigeon.setYaw(angle);
    }
    
    public void resetYaw() {
        resetYaw(0);
    }
    
    @Override
    public void periodic() {
        // Actualizar SmartDashboard
        SmartDashboard.putNumber("Gyro Yaw", getYaw());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumber("Gyro Roll", getRoll());
        SmartDashboard.putBoolean("Auto-turn Active", autoTurnActive);
        
        if (autoTurnActive) {
            double error = targetYaw - getYaw();
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            SmartDashboard.putNumber("Auto-turn Error", error);
            SmartDashboard.putNumber("Auto-turn Target", targetYaw);
            SmartDashboard.putNumber("Auto-turn Settle Timer", settleTimer);
            SmartDashboard.putNumber("Auto-turn Oscillations", oscillationCount);
            SmartDashboard.putNumber("Auto-turn Stall Timer", oscillationTimer);
        }
    }
}