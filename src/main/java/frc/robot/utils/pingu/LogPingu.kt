package frc.robot.utils.pingu

import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.StructSerializable
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.TEST_MODE
import org.littletonrobotics.junction.Logger.recordMetadata
import org.littletonrobotics.junction.Logger.recordOutput

/**
 * Type alias for a pair consisting of a log message and any associated data.
 */
typealias Log = Pair<String, Any>

/**
 * Utility class for logging. Provides methods to update PID
 * values, retrieve double values, create pairs, and perform test logging.
 */
object LogPingu {
    private val capturedLogs = mutableListOf<Log>()

    /**
     * Logs a key-value pair.
     *
     * @param string The key associated with the value to log.
     * @param value The value to log.
     */
    @JvmStatic
    fun log(
        string: String,
        value: Any,
    ) {
        capturedLogs.add(Log(string, value))
    }

    /**
     * Logs multiple values within the provided block context.
     *
     * This function captures logs generated within the block and logs them all at once
     * after the block has been executed. It uses a custom `LogContext` to temporarily
     * override the log method to capture logs instead of immediately logging them.
     *
     * @param block The block of code within which logs will be captured.
     */
    @JvmStatic
    fun logs(block: Runnable) {
        capturedLogs.clear()

        block.run()

        logs(*capturedLogs.toTypedArray())
    }

    /**
     * Logs multiple values with their respective keys based on the type of each value.
     *
     * @param logs Vararg parameter of pairs where the first element is the key and the second element is the value to log.
     */
    @JvmStatic
    @SafeVarargs
    fun logs(vararg logs: Log) =
        logs.forEach { (key, value) ->
            when (value) {
                is Double -> logs(key, value)
                is Int -> logs(key, value)
                is Boolean -> logs(key, value)
                is String -> logs(key, value)
                is WPISerializable -> logs(key, value)
                else -> println("Unsupported log type for key $key")
            }
        }

    /**
     * Logs a double value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The double value to log.
     */
    @JvmStatic
    fun logs(
        key: String?,
        value: Double,
    ) {
        if (TEST_MODE) {
            recordOutput(key, value)
        }
    }

    /**
     * Logs an integer value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The integer value to log.
     */
    @JvmStatic
    fun logs(
        key: String?,
        value: Int,
    ) {
        if (TEST_MODE) {
            recordOutput(key, value)
        }
    }

    /**
     * Logs a boolean value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The boolean value to log.
     */
    @JvmStatic
    fun logs(
        key: String?,
        value: Boolean,
    ) {
        if (TEST_MODE) {
            recordOutput(key, value)
        }
    }

    /**
     * Logs a String value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The String value to log.
     */
    @JvmStatic
    fun logs(
        key: String?,
        value: String?,
    ) {
        if (TEST_MODE) {
            recordOutput(key, value)
        }
    }

    /**
     * Logs a WPISerializable value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The WPISerializable value to log.
     */
    @JvmStatic
    fun <T : WPISerializable?> logs(
        key: String?,
        value: T,
    ) {
        if (TEST_MODE) {
            recordOutput(key, value)
        }
    }

    /**
     * Logs a StructSerializable value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The SwerveModuleState value to log.
     */
    @JvmStatic
    fun <T : StructSerializable?> logs(
        key: String?,
        vararg value: T,
    ) {
        if (TEST_MODE) {
            recordOutput(key, *value)
        }
    }

    /**
     * Logs a meta data value with a specified key if the system is in test mode.
     *
     * @param key The key associated with the value to log.
     * @param value The string value to log.
     */
    @JvmStatic
    fun metaLogs(
        key: String?,
        value: String?,
    ) {
        if (TEST_MODE) {
            recordMetadata(key, value)
        }
    }
}
