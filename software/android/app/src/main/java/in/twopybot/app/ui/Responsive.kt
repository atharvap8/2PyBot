package `in`.twopybot.app.ui

import android.content.res.Configuration
import androidx.compose.runtime.Composable
import androidx.compose.runtime.ReadOnlyComposable
import androidx.compose.ui.platform.LocalConfiguration

/**
 * Layout mode. A phone held sideways for driving has ~360 dp of height and
 * plenty of width, so the whole UI flips from "scroll a column" to
 * "two panes side by side" — the video never has to share vertical space
 * with the numbers.
 */
enum class Pane { PORTRAIT, LANDSCAPE, WIDE }

@Composable
@ReadOnlyComposable
fun pane(): Pane {
    val c = LocalConfiguration.current
    return when {
        c.orientation != Configuration.ORIENTATION_LANDSCAPE -> Pane.PORTRAIT
        c.screenWidthDp >= 840 -> Pane.WIDE          // tablet / foldable open
        else -> Pane.LANDSCAPE
    }
}

@Composable
@ReadOnlyComposable
fun isLandscape(): Boolean = pane() != Pane.PORTRAIT
