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
val Ink       = Color(0xFF090C10)
val Panel     = Color(0xFF12171E)
val PanelSoft = Color(0xFF1C2530)
val Line      = Color(0xFF29333F)
val TextHi    = Color(0xFFF1F4F8)
val TextDim   = Color(0xFF9AA7B7)
val Cyan      = Color(0xFF78D4F5)
val Violet    = Color(0xFFA7B4F6)
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
    titleLarge  = Typography().titleLarge.copy(fontSize=23.sp,fontWeight = FontWeight.SemiBold, letterSpacing = (-0.4).sp),
    titleMedium = Typography().titleMedium.copy(fontSize=16.sp,fontWeight = FontWeight.SemiBold),
    bodyMedium = Typography().bodyMedium.copy(fontSize=13.sp,lineHeight=19.sp),
    labelLarge = Typography().labelLarge.copy(fontSize=13.sp,fontWeight=FontWeight.Medium),
    labelSmall  = Typography().labelSmall.copy(letterSpacing = 1.0.sp),
)

@Composable
fun TwoPyBotTheme(dark: Boolean = isSystemInDarkTheme(), content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = Scheme, typography = Typo, content = content)
}
