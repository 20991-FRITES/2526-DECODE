package math

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale

class Distance
/** Creates a new default Distance object at 0  */ @JvmOverloads constructor(
    private val unit: DistanceUnit = DistanceUnit.MM,
    private val value: Double = 0.0
) {
    /**
     * Creates a new Distance object.
     *
     * @param unit the distance unit of the value
     * @param value the distance value
     */

    /**
     * Gets the distance value in the desired distance unit
     *
     * @param unit the desired distance unit
     * @return the value member converted to the desired distance unit
     */
    fun getValue(unit: DistanceUnit): Double {
        return unit.fromUnit(this.unit, value)
    }

    /** Gets the distance value converted to millimeters  */
    fun toMillimeters(): Double {
        return getValue(DistanceUnit.MM)
    }

    /** Gets the distance value converted to inches  */
    fun toInches(): Double {
        return getValue(DistanceUnit.INCH)
    }

    /** Gets the distance value converted to meters  */
    fun toMeters(): Double {
        return getValue(DistanceUnit.METER)
    }

    val isZero: Boolean
        /** Check if this Distance is zero.  */
        get() = value == 0.0

    /** Negates this Distance and returns the result as a new Distance object.  */
    fun negate(): Distance {
        return Distance(unit, -value)
    }

    /** Adds another Distance to this Distance and returns the result as a new Distance object.  */
    fun add(other: Distance): Distance {
        val sumInMM = toMillimeters() + other.toMillimeters()
        return fromMillimeters(sumInMM)
    }

    /**
     * Subtracts another Distance from this Distance and returns the result as a new Distance
     * object.
     */
    fun subtract(other: Distance): Distance {
        val differenceInMM = toMillimeters() - other.toMillimeters()
        return fromMillimeters(differenceInMM)
    }

    /** Multiplies this Distance by a scalar and returns the result as a new Distance object.  */
    fun multiply(scalar: Double): Distance {
        val productInMM = toMillimeters() * scalar
        return fromMillimeters(productInMM)
    }

    /** Divides this Distance by a scalar and returns the result as a new Distance object.  */
    fun divide(scalar: Double): Distance {
        val quotientInMM = toMillimeters() / scalar
        return fromMillimeters(quotientInMM)
    }

    /**
     * Divides this Distance by another Distance and returns the result as a new Distance object.
     */
    fun divide(other: Distance): Distance {
        val quotient = toMillimeters() / other.toMillimeters()
        return fromMillimeters(quotient)
    }

    /** Halves this Distance and returns the result as a new Distance object.  */
    fun halve(): Distance {
        return divide(2.0)
    }

    /** Divides this Distance by another Distance and returns the result as a unitless ratio.  */
    fun ratio(other: Distance): Double {
        return toMillimeters() / other.toMillimeters()
    }

    /** Whether this Distance is greater or equal to another Distance.  */
    fun geq(other: Distance): Boolean {
        return toMillimeters() >= other.toMillimeters()
    }

    /** Whether this Distance is less than or equal to another Distance.  */
    fun leq(other: Distance): Boolean {
        return toMillimeters() <= other.toMillimeters()
    }

    /**
     * Returns a string representation of the object in a human readable format for debugging
     * purposes.
     *
     * @return a string representation of the object
     */
    override fun toString(): String {
        return String.format(
            Locale.ENGLISH, "%.3f %s", getValue(DistanceUnit.METER), DistanceUnit.METER
        )
    }

    companion object {
        /** Creates a new Distance object from millimeters input  */
        fun fromMillimeters(millimeters: Double): Distance {
            return Distance(DistanceUnit.MM, millimeters)
        }

        /** Creates a new Distance object from meters input  */
        fun fromCentimeters(centimeters: Double): Distance {
            return Distance(DistanceUnit.CM, centimeters)
        }

        /** Creates a new Distance object from meters input  */
        fun fromMeters(meters: Double): Distance {
            return Distance(DistanceUnit.METER, meters)
        }

        /** Creates a new Distance object from inches input  */
        fun fromInches(inches: Double): Distance {
            return Distance(DistanceUnit.INCH, inches)
        }
    }
}