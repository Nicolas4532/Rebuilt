package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IO extends SubsystemBase {

    public SparkMax motor1;
    public SparkMax motor2;
    public SparkMax motor3; 
    
    // ==================== VARIABLES PARA SECUENCIA DE DISPARO ====================
    private boolean isShootSequenceActive = false;
    private double shootSequenceStartTime = 0.0;
    private static final double SHOOTER_SPINUP_TIME = 1.5;  // 2.5 segundos
    
    public IO() {
        motor1 = new SparkMax(7, MotorType.kBrushless);
        motor2 = new SparkMax(8, MotorType.kBrushed);
        motor3 = new SparkMax(11, MotorType.kBrushless);
        
        // ==================== CONFIGURACIÓN SHOOTER (MOTOR 3) ====================
        SparkMaxConfig motor3Config = new SparkMaxConfig();
        
        motor3Config
            .closedLoopRampRate(0.0)
            .openLoopRampRate(0.0)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        
        motor3.configure(motor3Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        DriverStation.reportWarning("✅ Shooter (motor3) configurado para aceleración rápida", false);
    }
    
    /**
     * Inicia la secuencia de disparo
     */
    public void startShootSequence(double speed) {
        if (!isShootSequenceActive) {
            isShootSequenceActive = true;
            shootSequenceStartTime = Timer.getFPGATimestamp();
            DriverStation.reportWarning("🔫 Secuencia de disparo iniciada", false);
        }
        
        // Calcular tiempo transcurrido
        double elapsedTime = Timer.getFPGATimestamp() - shootSequenceStartTime;
        
        if (elapsedTime < SHOOTER_SPINUP_TIME) {
            // FASE 1: Solo motor3 por 2.5 segundos
            motor1.set(0);
            motor2.set(0);
            motor3.set(speed);  // Máxima velocidad
        } else {
            // FASE 2: Después de 2.5 segundos, activar motor1 y motor2 también
            motor1.set(speed);
            motor2.set(speed);
            motor3.set(speed);  // Motor3 sigue girando
        }
    }
    
    /**
     * Método tradicional de disparo (sin secuencia)
     */
    public void shoot(double speed) {
        motor1.set(speed);
        motor2.set(speed);
        motor3.set(speed);
    }

    public void intake(double speed) {
        isShootSequenceActive = false;  // Cancelar secuencia si estaba activa
        motor1.set(speed);
        motor2.set(-speed);
        motor3.set(speed * 0.5);
    }

    public void outtake(double speed) {
        isShootSequenceActive = false;  // Cancelar secuencia
        motor1.set(-speed);
        motor2.set(speed);
        motor3.set(-speed);
    }

    public void stop(double speed) {
        isShootSequenceActive = false;  // Resetear secuencia
        motor1.set(0);
        motor2.set(0);
        motor3.set(0);
    }
    
    /**
     * @return true si la secuencia está en fase 2 (todos los motores girando)
     */
    public boolean isReadyToFeed() {
        if (!isShootSequenceActive) return false;
        double elapsedTime = Timer.getFPGATimestamp() - shootSequenceStartTime;
        return elapsedTime >= SHOOTER_SPINUP_TIME;
    }
    
    /**
     * @return tiempo restante en la fase de spin-up
     */
    public double getRemainingSpinupTime() {
        if (!isShootSequenceActive) return 0;
        double elapsedTime = Timer.getFPGATimestamp() - shootSequenceStartTime;
        return Math.max(0, SHOOTER_SPINUP_TIME - elapsedTime);
    }

}