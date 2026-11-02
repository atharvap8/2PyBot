package `in`.twopybot.app.ui

enum class AllowedOrientation { PORTRAIT, LANDSCAPE, REVERSE_LANDSCAPE }

object OrientationPolicy {
    /** Stable windows give hysteresis; reverse portrait is deliberately ignored. */
    fun candidate(degrees: Int): AllowedOrientation? = when (degrees) {
        in 0..30, in 330..359 -> AllowedOrientation.PORTRAIT
        in 60..120 -> AllowedOrientation.REVERSE_LANDSCAPE
        in 240..300 -> AllowedOrientation.LANDSCAPE
        else -> null
    }
    fun target(degrees: Int, previous: AllowedOrientation): AllowedOrientation = candidate(degrees) ?: previous
}

/** Elapsed-time policy, independent of sensor event rate and wall-clock changes. */
class HeldOrientationPolicy(private val holdMs: Long = 2_000L) {
    var pending: AllowedOrientation? = null
        private set
    private var since = 0L

    fun observe(degrees: Int, current: AllowedOrientation, now: Long) {
        val target = OrientationPolicy.candidate(degrees)
        if (target == null || target == current) reset()
        else if (target != pending) { pending = target; since = now }
    }
    fun remaining(now: Long): Long = (holdMs - (now - since)).coerceAtLeast(0)
    fun ready(now: Long): AllowedOrientation? {
        val result = pending ?: return null
        if (remaining(now) > 0) return null
        reset()
        return result
    }
    fun reset() { pending = null; since = 0L }
}
