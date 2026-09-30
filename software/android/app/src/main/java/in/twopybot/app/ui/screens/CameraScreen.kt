package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import `in`.twopybot.app.data.CamControl
import `in`.twopybot.app.data.CamPayload
import `in`.twopybot.app.ui.components.Section
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.launch

@Composable
fun CameraScreen(vm: BotViewModel) {
    val scope = rememberCoroutineScope()
    var cam by remember { mutableStateOf<CamPayload?>(null) }
    var busy by remember { mutableStateOf(false) }
    var calLog by remember { mutableStateOf("") }

    LaunchedEffect(Unit) { cam = vm.repo.cameraControls() }

    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp)) {

        Section("Low-light calibration",
            "Auto-exposure is why the frame rate collapses in a dim room — the sensor " +
            "lengthens exposure instead of admitting it is dark. This locks exposure to " +
            "manual, then walks exposure and gain to a target brightness. Exposure first " +
            "(free, but adds motion blur), gain second (keeps the frame rate, adds noise).") {
            Button(
                onClick = {
                    busy = true; calLog = "calibrating, ~15 s…"
                    scope.launch {
                        val r = vm.repo.autoCalibrate(110)
                        calLog = r.getOrNull()?.take(900) ?: "failed: ${r.exceptionOrNull()?.message}"
                        busy = false
                        cam = vm.repo.cameraControls()
                    }
                },
                enabled = !busy, modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(containerColor = Cyan, contentColor = Ink)
            ) { Text(if (busy) "calibrating…" else "Auto-calibrate for current light") }

            if (calLog.isNotEmpty()) {
                Spacer(Modifier.height(8.dp))
                Text(calLog, style = Mono.copy(fontSize = 10.sp), color = TextDim, lineHeight = 14.sp)
            }
        }

        cam?.let { c ->
            Section("Profiles", "Presets matched to this camera's real control ranges.") {
                c.profiles.chunked(3).forEach { row ->
                    Row(Modifier.fillMaxWidth().padding(vertical = 3.dp),
                        horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        row.forEach { p ->
                            OutlinedButton(
                                onClick = { scope.launch { vm.repo.camProfile(p); cam = vm.repo.cameraControls(); vm.toast("$p applied") } },
                                modifier = Modifier.weight(1f),
                                contentPadding = PaddingValues(horizontal = 4.dp, vertical = 8.dp)
                            ) { Text(p, fontSize = 11.sp, maxLines = 1) }
                        }
                        repeat(3 - row.size) { Spacer(Modifier.weight(1f)) }
                    }
                }
            }

            Section("Sensor controls", "Enumerated live from v4l2 — these are the controls " +
                    "your camera actually exposes, not a guess.") {
                c.controls.filter { it.type in listOf("int", "bool", "menu") }.forEach { ctl ->
                    CamRow(ctl, c.values[ctl.name] ?: ctl.default) { v ->
                        scope.launch { vm.repo.setCamera(ctl.name, v) }
                    }
                }
            }
        } ?: Text("reading camera…", color = TextDim, fontSize = 13.sp)

        Spacer(Modifier.height(70.dp))
    }
}

@Composable
private fun CamRow(ctl: CamControl, initial: Int, onCommit: (Int) -> Unit) {
    var v by remember(ctl.name, initial) { mutableStateOf(initial.toFloat()) }
    Column(Modifier.fillMaxWidth().padding(vertical = 4.dp)) {
        Row {
            Text(ctl.name, style = Mono.copy(fontSize = 12.sp), color = TextHi,
                 modifier = Modifier.weight(1f))
            Text(v.toInt().toString(), style = Mono.copy(fontSize = 12.sp), color = Cyan)
        }
        Slider(
            value = v.coerceIn(ctl.min.toFloat(), ctl.max.toFloat()),
            onValueChange = { v = it },
            onValueChangeFinished = { onCommit(v.toInt()) },
            valueRange = ctl.min.toFloat()..ctl.max.toFloat(),
            colors = SliderDefaults.colors(thumbColor = Cyan,
                activeTrackColor = Cyan.copy(alpha = .6f), inactiveTrackColor = Line),
            modifier = Modifier.height(26.dp)
        )
    }
}
