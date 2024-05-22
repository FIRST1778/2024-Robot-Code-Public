package org.chillout1778.lib

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import kotlin.math.abs

class Util {
    companion object {
        fun neo(id: Int): CANSparkMax {
            val motor = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless)
            // Restore everything so that we aren't messed up by previous
            // code using this motor.
            motor.restoreFactoryDefaults()
            return motor
        }

        fun clamp(n: Double, range: Double) = n.coerceIn(-range, range)
        fun deadband(n: Double) = if (abs(n) < 0.1) 0.0 else n
        fun wrapAngle(n: Double): Double {
            var n2 = n % (2.0*Math.PI)
            if (n2 < 0.0)
                n2 += 2.0*Math.PI
            if (n2 > Math.PI)
                n2 -= 2*Math.PI
            if (n2 < -Math.PI || n2 > Math.PI) {
                println("Something is very wrong: $n2")
            }
            return n2
        }
    }
}
