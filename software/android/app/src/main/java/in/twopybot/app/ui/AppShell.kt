package `in`.twopybot.app.ui

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import `in`.twopybot.app.BuildConfig
import `in`.twopybot.app.ui.screens.*
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.flow.collectLatest

private data class Destination(val id: String, val title: String, val detail: String, val icon: ImageVector)

private val destinations = listOf(
    Destination("drive", "Drive", "Robot and instruments", Icons.Default.DirectionsCar),
    Destination("live", "Live", "Camera and telemetry", Icons.Default.Videocam),
    Destination("maps", "Maps", "Lidar SLAM, 2D and 3D", Icons.Default.Layers),
    Destination("system", "System", "Live health of every part", Icons.Default.Memory),
    Destination("auto", "Routes", "Teach and replay", Icons.Default.Route),
    Destination("map", "Trail", "Wheel odometry trail", Icons.Default.Map),
    Destination("tune", "Tune", "Controller parameters", Icons.Default.Tune),
    Destination("cam", "Camera", "Optics and perception", Icons.Default.CameraAlt),
    Destination("data", "Sessions", "Record and export", Icons.Default.FolderOpen),
    Destination("set", "Setup", "Connection and firmware", Icons.Default.Settings),
)
private val primary = destinations.take(4)
private val moreItem = Destination("more", "More", "All workspaces", Icons.Default.GridView)

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun RobotApp(vm: BotViewModel = viewModel()) {
    var selected by rememberSaveable { mutableStateOf("drive") }
    var more by rememberSaveable { mutableStateOf(false) }
    val snackbar = remember { SnackbarHostState() }
    val current = destinations.first { it.id == selected }
    LaunchedEffect(vm) { vm.snack.collectLatest { if (it != null) { snackbar.showSnackbar(it); vm.clearToast() } } }
    BackHandler(selected != "drive" && !more) { selected = "drive" }
    val goDrive = { selected = "drive" }
    // Only the selected workspace is composed, so the video, the 3D view and the
    // web pages never run at the same time.
    val workspace = remember(vm) {
        movableContentOf<String> { destination ->
            when (destination) {
                "drive" -> DrivingScreen(vm)
                "live" -> LiveScreen(vm)
                "maps" -> CalibrationScreen(vm, page = "/slam/", backLabel = "← Drive", onBack = goDrive)
                "system" -> CalibrationScreen(vm, page = "/sysmap/", backLabel = "← Drive", onBack = goDrive)
                "auto" -> AutoScreen(vm)
                "map" -> MapScreen(vm)
                "tune" -> TuneScreen(vm)
                "cam" -> CameraScreen(vm)
                "data" -> RecorderScreen(vm)
                else -> SettingsScreen(vm)
            }
        }
    }
    val landscape = isLandscape()
    BoxWithConstraints(Modifier.fillMaxSize().background(Ink).windowInsetsPadding(WindowInsets.safeDrawing)) {
        val rail = landscape || maxWidth >= 600.dp
        Scaffold(
            containerColor = Ink, contentWindowInsets = WindowInsets(0, 0, 0, 0),
            snackbarHost = { SnackbarHost(snackbar) },
            topBar = { if (!rail) TopBar(vm, current.title) },
            bottomBar = {
                if (!rail) Column {
                    HorizontalDivider(color = Line, thickness = 1.dp)
                    NavigationBar(containerColor = Ink, tonalElevation = 0.dp,
                        modifier = Modifier.height(64.dp), windowInsets = WindowInsets(0, 0, 0, 0)) {
                        (primary + moreItem).forEach { d ->
                            val active = if (d.id == "more") selected !in primary.map { it.id } else selected == d.id
                            NavigationBarItem(selected = active,
                                onClick = { if (d.id == "more") more = true else selected = d.id },
                                icon = { Icon(d.icon, d.title, Modifier.size(22.dp)) },
                                label = { Text(d.title, fontSize = 11.sp, maxLines = 1) },
                                colors = NavigationBarItemDefaults.colors(selectedIconColor = Accent, selectedTextColor = Accent,
                                    indicatorColor = AccentBg, unselectedIconColor = TextDim, unselectedTextColor = TextDim))
                        }
                    }
                }
            }) { padding ->
            Row(Modifier.fillMaxSize().padding(padding)) {
                if (rail) SideRail(vm, selected, onSelect = { selected = it }, onMore = { more = true })
                Box(Modifier.weight(1f).fillMaxHeight()) { workspace(selected) }
            }
        }
    }
    if (more) ModalBottomSheet(onDismissRequest = { more = false }, containerColor = Panel) {
        Column(Modifier.fillMaxWidth().heightIn(max = 600.dp).verticalScroll(rememberScrollState()).padding(horizontal = 18.dp)) {
            Text("Workspaces", style = MaterialTheme.typography.titleLarge, color = TextHi)
            Text("2PyBot · ${BuildConfig.VERSION_NAME}", fontSize = 12.sp, color = TextDim,
                modifier = Modifier.padding(top = 2.dp, bottom = 14.dp))
            destinations.chunked(2).forEach { pair ->
                Row(Modifier.fillMaxWidth().padding(bottom = 10.dp), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                    pair.forEach { d ->
                        val active = selected == d.id
                        Column(Modifier.weight(1f).clip(RoundedCornerShape(10.dp))
                            .background(if (active) AccentBg else PanelSoft)
                            .border(1.dp, if (active) Accent else Line, RoundedCornerShape(10.dp))
                            .clickable { selected = d.id; more = false }.padding(14.dp)) {
                            Icon(d.icon, null, tint = Accent, modifier = Modifier.size(22.dp))
                            Text(d.title, color = TextHi, fontWeight = FontWeight.SemiBold, modifier = Modifier.padding(top = 10.dp))
                            Text(d.detail, color = TextDim, fontSize = 11.sp, maxLines = 1, overflow = TextOverflow.Ellipsis)
                        }
                    }
                    if (pair.size == 1) Spacer(Modifier.weight(1f))
                }
            }
            Spacer(Modifier.height(20.dp))
        }
    }
}

/** Connection state; collects telemetry itself so only this text recomposes at 10 Hz. */
@Composable
private fun rememberLink(vm: BotViewModel): Pair<String, Color> {
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val connected by vm.repo.connected.collectAsStateWithLifecycle()
    val online = connected && !tel.stale
    val text = when {
        !online -> "OFFLINE"
        tel.selfRighting -> "SELF-UPRIGHTING"
        tel.balancing -> "BALANCING"
        else -> "IDLE"
    }
    val color = when { !online -> Red; tel.balancing || tel.selfRighting -> Green; else -> Amber }
    return text to color
}

@Composable
private fun TopBar(vm: BotViewModel, title: String) {
    val unsaved by vm.repo.unsaved.collectAsStateWithLifecycle()
    Column {
        Row(Modifier.fillMaxWidth().height(56.dp).padding(horizontal = 16.dp),
            verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            BrandMark()
            Column(Modifier.weight(1f)) {
                Text(title, fontSize = 17.sp, fontWeight = FontWeight.SemiBold, color = TextHi, maxLines = 1)
                LinkLine(vm)
            }
            if (unsaved) Text("UNSAVED", color = Amber, fontSize = 10.sp, fontWeight = FontWeight.Bold, letterSpacing = .6.sp)
            StopButton(vm)
        }
        HorizontalDivider(color = Line, thickness = 1.dp)
    }
}

@Composable
private fun LinkLine(vm: BotViewModel) {
    val (text, color) = rememberLink(vm)
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(Modifier.size(7.dp).clip(CircleShape).background(color))
        Text(text, fontSize = 10.5.sp, letterSpacing = .8.sp, color = TextDim, modifier = Modifier.padding(start = 6.dp))
    }
}

@Composable
private fun BrandMark() {
    Box(Modifier.size(30.dp).clip(CircleShape).border(2.dp, Accent, CircleShape), contentAlignment = Alignment.Center) {
        Box(Modifier.size(10.dp).clip(CircleShape).background(Accent))
    }
}

@Composable
private fun StopButton(vm: BotViewModel) {
    Button(onClick = { vm.estop() }, contentPadding = PaddingValues(horizontal = 14.dp),
        modifier = Modifier.height(40.dp), shape = RoundedCornerShape(6.dp),
        colors = ButtonDefaults.buttonColors(containerColor = Red, contentColor = Color.White)) {
        Text("STOP", fontSize = 13.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
    }
}

@Composable
private fun SideRail(vm: BotViewModel, selected: String, onSelect: (String) -> Unit, onMore: () -> Unit) {
    val (_, linkColor) = rememberLink(vm)
    Row {
        Column(Modifier.width(76.dp).fillMaxHeight().background(Ink).padding(horizontal = 6.dp),
            horizontalAlignment = Alignment.CenterHorizontally) {
            Box(Modifier.padding(vertical = 12.dp)) { BrandMark() }
            Column(Modifier.weight(1f).verticalScroll(rememberScrollState()), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                (primary + moreItem).forEach { d ->
                    val active = if (d.id == "more") selected !in primary.map { it.id } else selected == d.id
                    Column(Modifier.fillMaxWidth().heightIn(min = 56.dp).clip(RoundedCornerShape(10.dp))
                        .background(if (active) AccentBg else Color.Transparent)
                        .clickable { if (d.id == "more") onMore() else onSelect(d.id) }.padding(vertical = 7.dp),
                        horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(3.dp)) {
                        Icon(d.icon, d.title, Modifier.size(21.dp), tint = if (active) Accent else TextDim)
                        Text(d.title, fontSize = 10.5.sp, color = if (active) Accent else TextDim)
                    }
                }
            }
            Box(Modifier.size(8.dp).clip(CircleShape).background(linkColor))
            Spacer(Modifier.height(8.dp))
            Column(Modifier.fillMaxWidth().height(52.dp).clip(RoundedCornerShape(6.dp))
                .background(Red).clickable { vm.estop() },
                horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.Center) {
                Text("STOP", fontSize = 12.sp, color = Color.White, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
            }
            Spacer(Modifier.height(8.dp))
        }
        VerticalDivider(color = Line, thickness = 1.dp)
    }
}
