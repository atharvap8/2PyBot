package `in`.twopybot.app.ui

import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import `in`.twopybot.app.BuildConfig
import `in`.twopybot.app.R
import `in`.twopybot.app.ui.theme.Accent
import `in`.twopybot.app.ui.theme.Ink
import `in`.twopybot.app.ui.theme.Line
import `in`.twopybot.app.ui.theme.TextDim

/**
 * Launch screen: the 2PyBot wordmark on top, the real front photo of the robot
 * in the middle, and a loading bar along the bottom. Same black ground and
 * single orange accent as the rest of the app. Shown once at start while the
 * first screen and the connection warm up, then handed off to [RobotApp].
 */
@Composable
fun SplashScreen(onDone: () -> Unit) {
    val progress = remember { Animatable(0f) }
    LaunchedEffect(Unit) {
        progress.animateTo(1f, animationSpec = tween(durationMillis = 2000, easing = LinearEasing))
        onDone()
    }
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Ink)
            .windowInsetsPadding(WindowInsets.safeDrawing)
            .padding(horizontal = 36.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Spacer(Modifier.weight(0.16f))
        Image(
            painter = painterResource(R.drawable.logo_wordmark),
            contentDescription = "2PyBot",
            modifier = Modifier.height(40.dp),
        )
        Spacer(Modifier.height(28.dp))
        Image(
            painter = painterResource(R.drawable.bot_front),
            contentDescription = "2PyBot robot, front view",
            contentScale = ContentScale.Fit,
            modifier = Modifier.weight(1f).fillMaxWidth(),
        )
        Spacer(Modifier.height(28.dp))
        LinearProgressIndicator(
            progress = progress.value,
            modifier = Modifier.fillMaxWidth().height(4.dp),
            color = Accent,
            trackColor = Line,
        )
        Spacer(Modifier.height(12.dp))
        Text("v${BuildConfig.VERSION_NAME}", color = TextDim, fontSize = 11.sp, fontWeight = FontWeight.Medium)
        Spacer(Modifier.height(24.dp))
    }
}
