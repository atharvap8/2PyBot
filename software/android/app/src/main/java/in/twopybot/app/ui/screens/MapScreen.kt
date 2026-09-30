package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.*
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.delay

@Composable
fun MapScreen(vm: BotViewModel) {
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val trail by vm.trail.collectAsStateWithLifecycle()

    LaunchedEffect(Unit) {
        while (true) { vm.refreshTrail(); delay(1000) }
    }

    val map: @Composable (Modifier) -> Unit = { m ->
        TrailMap(trail, tel.x, tel.y, tel.heading, m)
    }
    val stats: @Composable ColumnScope.() -> Unit = {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Tile("x", "%.2f m".format(tel.x), Cyan, Modifier.weight(1f))
            Tile("y", "%.2f m".format(tel.y), Cyan, Modifier.weight(1f))
        }
        Spacer(Modifier.height(8.dp))
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Tile("heading", "%.0f°".format(tel.heading), Violet, Modifier.weight(1f))
            Tile("compass", "%.0f°".format(tel.yaw), Green, Modifier.weight(1f))
        }
        Spacer(Modifier.height(8.dp))
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Tile("speed", "%.2f".format(tel.v), Cyan, Modifier.weight(1f))
            Tile("trail", "${trail.size} pts", TextDim, Modifier.weight(1f))
        }
        Spacer(Modifier.height(10.dp))
        OutlinedButton(onClick = { vm.resetPose() }, modifier = Modifier.fillMaxWidth()) {
            Text("Reset pose and clear trail")
        }
        Text("Dead-reckoned from the encoders — not SLAM. Drift accumulates with " +
             "every wheel slip; reset when you reposition the robot.",
             fontSize = 11.sp, color = TextDim, modifier = Modifier.padding(top = 8.dp))
    }

    if (isLandscape()) {
        Row(Modifier.fillMaxSize().padding(10.dp), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            map(Modifier.weight(1f).fillMaxHeight())
            Column(Modifier.width(230.dp).verticalScroll(rememberScrollState())) { stats() }
        }
    } else {
        Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp)) {
            Section("Odometry") { map(Modifier.fillMaxWidth().aspectRatio(1f)) }
            stats()
            Spacer(Modifier.height(70.dp))
        }
    }
}
