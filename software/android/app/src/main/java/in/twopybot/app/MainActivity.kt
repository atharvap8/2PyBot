package `in`.twopybot.app

import android.os.Bundle
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
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.pane
import `in`.twopybot.app.ui.screens.*
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.flow.collectLatest

private data class Dest(val id: String, val label: String, val icon: ImageVector)

private val DESTS = listOf(
    Dest("live", "Live", Icons.Filled.Videocam),
    Dest("map", "Map", Icons.Filled.Map),
    Dest("tune", "Tune", Icons.Filled.Tune),
    Dest("auto", "Auto", Icons.Filled.AutoFixHigh),
    Dest("cam", "Camera", Icons.Filled.CameraAlt),
    Dest("set", "Setup", Icons.Filled.Settings),
)

@OptIn(ExperimentalMaterial3Api::class)
class MainActivity : ComponentActivity() {
    override fun onResume() {
        super.onResume()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onPause() {
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        super.onPause()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            TwoPyBotTheme {
                val vm: BotViewModel = viewModel()
                val snackbar = remember { SnackbarHostState() }
                var dest by rememberSaveable { mutableStateOf("live") }

                LaunchedEffect(Unit) {
                    vm.snack.collectLatest { m ->
                        if (m != null) { snackbar.showSnackbar(m); vm.clearToast() }
                    }
                }

                val land = isLandscape()

                Scaffold(
                    containerColor = Ink,
                    snackbarHost = { SnackbarHost(snackbar) },
                    // In landscape the top bar is dead weight: it costs ~15% of
                    // the screen height, which is exactly what the video wants.
                    topBar = {
                        if (!land) CenterAlignedTopAppBar(
                            title = { Text("2PyBot", style = MaterialTheme.typography.titleLarge, color = TextHi) },
                            actions = {
                                TextButton(onClick = { vm.estop() }) {
                                    Text("STOP", color = Red, style = Mono.copy(fontSize = 14.sp))
                                }
                            },
                            colors = TopAppBarDefaults.centerAlignedTopAppBarColors(
                                containerColor = Ink, titleContentColor = TextHi)
                        )
                    },
                    bottomBar = {
                        if (!land) NavigationBar(containerColor = Panel, tonalElevation = 0.dp) {
                            DESTS.forEach { d ->
                                NavigationBarItem(
                                    selected = dest == d.id,
                                    onClick = { dest = d.id },
                                    icon = { Icon(d.icon, d.label, modifier = Modifier.size(20.dp)) },
                                    label = { Text(d.label, fontSize = 9.5.sp) },
                                    colors = NavigationBarItemDefaults.colors(
                                        selectedIconColor = Ink, selectedTextColor = Cyan,
                                        indicatorColor = Cyan,
                                        unselectedIconColor = TextDim, unselectedTextColor = TextDim)
                                )
                            }
                        }
                    }
                ) { padding ->
                    Row(Modifier.padding(padding).fillMaxSize()) {
                        if (land) {
                            // Rail instead of a bottom bar: vertical space is the
                            // scarce resource when the phone is sideways.
                            Column(Modifier.width(80.dp).fillMaxHeight().background(Panel),
                                horizontalAlignment = Alignment.CenterHorizontally) {
                                    Spacer(Modifier.height(4.dp))
                                    FilledIconButton(
                                        onClick = { vm.estop() },
                                        colors = IconButtonDefaults.filledIconButtonColors(
                                            containerColor = Red, contentColor = Color.White),
                                        modifier = Modifier.size(44.dp)
                                    ) { Icon(Icons.Filled.Stop, "E-stop") }
                                    Spacer(Modifier.height(4.dp))
                                BoxWithConstraints(Modifier.weight(1f).fillMaxWidth()) {
                                    val height = (maxHeight / DESTS.size).coerceIn(40.dp, 64.dp)
                                    Column(Modifier.fillMaxWidth().verticalScroll(rememberScrollState())) {
                                        DESTS.forEach { d ->
                                            Column(Modifier.fillMaxWidth().height(height)
                                                .background(if (dest == d.id) Cyan.copy(alpha=.16f) else Color.Transparent)
                                                .clickable { dest = d.id },
                                                horizontalAlignment=Alignment.CenterHorizontally,
                                                verticalArrangement=Arrangement.Center) {
                                                Icon(d.icon,d.label,Modifier.size(20.dp),tint=if (dest==d.id) Cyan else TextDim)
                                                Text(d.label,fontSize=10.sp,color=if (dest==d.id) Cyan else TextDim)
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        Box(Modifier.weight(1f)) {
                            when (dest) {
                                "live" -> LiveScreen(vm)
                                "map"  -> MapScreen(vm)
                                "tune" -> TuneScreen(vm)
                                "auto" -> AutoScreen(vm)
                                "cam"  -> CameraScreen(vm)
                                else   -> SettingsScreen(vm)
                            }
                        }
                    }
                }
            }
        }
    }
}
