package `in`.twopybot.app.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

// Same design system as the bot's System map web page: warm near-black ground,
// one orange accent, green = OK, amber = changed/warning, red only for stop.
// (Name "Cyan" is kept for source compatibility: it is the accent colour.)
val Ink       = Color(0xFF141210)
val Panel     = Color(0xFF1B1815)
val PanelSoft = Color(0xFF221E1A)
val Line      = Color(0xFF332C26)
val TextHi    = Color(0xFFEFE9E3)
val TextDim   = Color(0xFFABA196)
val TextMute  = Color(0xFF73695F)
val Cyan      = Color(0xFFF08A4B)
val Accent    = Cyan
val AccentBg  = Color(0xFF2A211B)
val Violet    = Color(0xFFA78BFA)
val Green     = Color(0xFF34D399)
val Amber     = Color(0xFFF0B84B)
val Red       = Color(0xFFFF4D6D)

private val Scheme = darkColorScheme(
    primary = Cyan, onPrimary = Color(0xFF1B1210), primaryContainer = AccentBg, onPrimaryContainer = Cyan,
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

private val Shapes = androidx.compose.material3.Shapes(
    extraSmall = androidx.compose.foundation.shape.RoundedCornerShape(4.dp),
    small = androidx.compose.foundation.shape.RoundedCornerShape(6.dp),
    medium = androidx.compose.foundation.shape.RoundedCornerShape(10.dp),
    large = androidx.compose.foundation.shape.RoundedCornerShape(10.dp),
    extraLarge = androidx.compose.foundation.shape.RoundedCornerShape(14.dp),
)

private val Typo = Typography(
    titleLarge  = Typography().titleLarge.copy(fontSize=23.sp,fontWeight = FontWeight.SemiBold, letterSpacing = (-0.4).sp),
    titleMedium = Typography().titleMedium.copy(fontSize=16.sp,fontWeight = FontWeight.SemiBold),
    bodyMedium = Typography().bodyMedium.copy(fontSize=13.sp,lineHeight=19.sp),
    labelLarge = Typography().labelLarge.copy(fontSize=13.sp,fontWeight=FontWeight.Medium),
    labelSmall  = Typography().labelSmall.copy(letterSpacing = 1.0.sp),
)

@Composable
fun TwoPyBotTheme(dark: Boolean = isSystemInDarkTheme(), content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = Scheme, typography = Typo, shapes = Shapes, content = content)
}
