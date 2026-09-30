package `in`.twopybot.app.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

// An instrument panel, not a Material demo: near-black ground, one cyan accent
// for live data, violet for control, amber for "changed but unsaved", and red
// reserved exclusively for stop and danger so it never loses its meaning.
val Ink       = Color(0xFF0A0D13)
val Panel     = Color(0xFF141A24)
val PanelSoft = Color(0xFF1B2230)
val Line      = Color(0xFF283041)
val TextHi    = Color(0xFFE9EDF5)
val TextDim   = Color(0xFF8892A6)
val Cyan      = Color(0xFF35E0D8)
val Violet    = Color(0xFFA78BFA)
val Green     = Color(0xFF4ADE80)
val Amber     = Color(0xFFFBBF24)
val Red       = Color(0xFFF43F5E)

private val Scheme = darkColorScheme(
    primary = Cyan, onPrimary = Ink,
    secondary = Violet, onSecondary = Ink,
    tertiary = Green, onTertiary = Ink,
    background = Ink, onBackground = TextHi,
    surface = Panel, onSurface = TextHi,
    surfaceVariant = PanelSoft, onSurfaceVariant = TextDim,
    error = Red, onError = Color.White,
    outline = Line,
)

// Live numbers are read at a glance on a moving robot: monospace so the
// digits don't jitter sideways every time a value changes.
val Mono = TextStyle(fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Medium)

private val Typo = Typography(
    titleLarge  = Typography().titleLarge.copy(fontWeight = FontWeight.SemiBold, letterSpacing = 0.3.sp),
    titleMedium = Typography().titleMedium.copy(fontWeight = FontWeight.SemiBold),
    labelSmall  = Typography().labelSmall.copy(letterSpacing = 1.0.sp),
)

@Composable
fun TwoPyBotTheme(dark: Boolean = isSystemInDarkTheme(), content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = Scheme, typography = Typo, content = content)
}
