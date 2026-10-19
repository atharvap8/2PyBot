package `in`.twopybot.app

import android.os.Bundle
import android.content.pm.ActivityInfo
import android.view.OrientationEventListener
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.Alignment
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import `in`.twopybot.app.ui.Pane
import `in`.twopybot.app.ui.AllowedOrientation
import `in`.twopybot.app.ui.OrientationPolicy
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.pane
import `in`.twopybot.app.ui.screens.*
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.flow.collectLatest

private data class Dest(val id: String, val label: String, val icon: ImageVector)

private val DESTS = listOf(
    Dest("drive", "Drive", Icons.Filled.DirectionsCar),
    Dest("live", "Live", Icons.Filled.Videocam),
    Dest("map", "Map", Icons.Filled.Map),
    Dest("tune", "Tune", Icons.Filled.Tune),
    Dest("auto", "Auto", Icons.Filled.AutoFixHigh),
    Dest("cam", "Camera", Icons.Filled.CameraAlt),
    Dest("data", "Data", Icons.Filled.FiberManualRecord),
    Dest("set", "Setup", Icons.Filled.Settings),
)

@OptIn(ExperimentalMaterial3Api::class)
class MainActivity : ComponentActivity() {
    private var allowedOrientation = AllowedOrientation.PORTRAIT
    private lateinit var orientationListener: OrientationEventListener
    override fun onResume() {
        super.onResume()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        if (::orientationListener.isInitialized && orientationListener.canDetectOrientation()) orientationListener.enable()
    }

    override fun onPause() {
        if (::orientationListener.isInitialized) orientationListener.disable()
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        super.onPause()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_PORTRAIT
        orientationListener = object : OrientationEventListener(this) {
            override fun onOrientationChanged(orientation: Int) {
                val target = OrientationPolicy.target(orientation, allowedOrientation)
                if (target != allowedOrientation) {
                    allowedOrientation = target
                    requestedOrientation = when (target) {
                        AllowedOrientation.PORTRAIT -> ActivityInfo.SCREEN_ORIENTATION_PORTRAIT
                        AllowedOrientation.LANDSCAPE -> ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
                        AllowedOrientation.REVERSE_LANDSCAPE -> ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE
                    }
                }
            }
        }
        enableEdgeToEdge()
        setContent {
            TwoPyBotTheme {
                `in`.twopybot.app.ui.RobotApp()
            }
        }
    }
}
