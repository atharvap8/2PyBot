package `in`.twopybot.app.ui

enum class AllowedOrientation { PORTRAIT, LANDSCAPE, REVERSE_LANDSCAPE }

object OrientationPolicy {
    /** Stable windows give hysteresis; reverse portrait is deliberately ignored. */
    fun target(degrees: Int, previous: AllowedOrientation): AllowedOrientation = when (degrees) {
        in 0..30, in 330..359 -> AllowedOrientation.PORTRAIT
        in 60..120 -> AllowedOrientation.REVERSE_LANDSCAPE
        in 240..300 -> AllowedOrientation.LANDSCAPE
        else -> previous
    }
}
