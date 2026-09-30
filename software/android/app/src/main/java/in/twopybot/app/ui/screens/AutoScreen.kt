package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel

private data class Routine(
    val cmd: String, val title: String, val blurb: String,
    val danger: Boolean = false, val needsBalancing: Boolean = true,
)

private val ROUTINES = listOf(
    Routine("wobble", "Fix the wobble",
        "Measures how much the pitch is actually oscillating, then walks K3 and K4 " +
        "down 7%/12% a step until it settles. Stops at the first calm window, or at " +
        "40% of the starting gains — if it reaches that floor the wobble is " +
        "mechanical (loose mount, soft tyre), not a gain problem."),
    Routine("trim", "Find the balance point",
        "The pitch the robot actually holds IS the trim error. Averages it over 4 s, " +
        "folds it into TRIM, repeats three times. Do this after moving anything heavy."),
    Routine("radius", "Measure wheel radius",
        "Start this, push the robot exactly 2.00 m by hand, then press Done. Compares " +
        "what the encoders reported against the real 2.00 m and corrects WHEELR. This " +
        "is the loaded rolling radius — the number that scales the whole control loop.",
        needsBalancing = false),
    Routine("stall", "Find the speed ceiling",
        "WHEELS OFF THE GROUND. Ramps the motors until the encoders stop following the " +
        "command, then sets MAXSPD to 80% of that. This is the honest stall point at " +
        "your current motor current and battery voltage.",
        danger = true, needsBalancing = false),
)

@Composable
fun AutoScreen(vm: BotViewModel) {
    val log by vm.repo.autoLog.collectAsStateWithLifecycle()
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val busy = log.lastOrNull()?.startsWith("start") == true ||
               log.lastOrNull()?.contains(",rms,") == true
    var confirm by remember { mutableStateOf<Routine?>(null) }

    val cards: @Composable ColumnScope.() -> Unit = {
        ROUTINES.forEach { r ->
            Column(
                Modifier.fillMaxWidth().padding(vertical = 5.dp)
                    .clip(RoundedCornerShape(16.dp)).background(Panel)
                    .border(1.dp, if (r.danger) Red.copy(alpha = .45f) else Line,
                            RoundedCornerShape(16.dp))
                    .padding(13.dp)
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(r.title, style = MaterialTheme.typography.titleMedium,
                         color = if (r.danger) Red else TextHi, modifier = Modifier.weight(1f))
                    if (r.needsBalancing && !tel.balancing)
                        Text("needs balancing", fontSize = 10.sp, color = Amber)
                }
                Spacer(Modifier.height(4.dp))
                Text(r.blurb, fontSize = 12.sp, color = TextDim, lineHeight = 16.sp)
                Spacer(Modifier.height(9.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = { if (r.danger) confirm = r else vm.autoTune(r.cmd) },
                        enabled = !busy && (!r.needsBalancing || tel.balancing),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = if (r.danger) Red else Cyan,
                            contentColor = if (r.danger) androidx.compose.ui.graphics.Color.White else Ink)
                    ) { Text("Run", fontWeight = FontWeight.SemiBold) }

                    if (r.cmd == "radius")
                        OutlinedButton(onClick = { vm.autoTune("done") }) { Text("Done — I pushed 2 m") }
                }
            }
        }

        Spacer(Modifier.height(4.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = { vm.autoTune("stop") }, modifier = Modifier.weight(1f)) {
                Text("Abort")
            }
            Button(onClick = { vm.paramAction("save") }, modifier = Modifier.weight(1f),
                colors = ButtonDefaults.buttonColors(containerColor = Green, contentColor = Ink)) {
                Text("Keep result (Save)")
            }
            OutlinedButton(onClick = { vm.paramAction("load") }, modifier = Modifier.weight(1f)) {
                Text("Discard")
            }
        }
        Text("Routines change values in RAM only. Nothing survives a reboot until you Save.",
             fontSize = 11.sp, color = TextDim, modifier = Modifier.padding(top = 6.dp))
    }

    val logPane: @Composable ColumnScope.() -> Unit = {
        Text("PROGRESS", fontSize = 9.5.sp, letterSpacing = 1.sp, color = TextDim)
        Spacer(Modifier.height(6.dp))
        Column(
            Modifier.fillMaxWidth().weight(1f, fill = false)
                .heightIn(min = 120.dp)
                .clip(RoundedCornerShape(12.dp)).background(Ink)
                .border(1.dp, Line, RoundedCornerShape(12.dp))
                .padding(10.dp).verticalScroll(rememberScrollState())
        ) {
            if (log.isEmpty())
                Text("idle — pick a routine", color = TextDim, style = Mono.copy(fontSize = 11.sp))
            log.takeLast(60).forEach { line ->
                val c = when {
                    line.startsWith("done")   -> Green
                    line.startsWith("abort")  -> Red
                    line.startsWith("refused")-> Amber
                    line.startsWith("start")  -> Cyan
                    else -> TextDim
                }
                Text(line, style = Mono.copy(fontSize = 11.sp), color = c, lineHeight = 15.sp)
            }
        }
    }

    if (isLandscape()) {
        Row(Modifier.fillMaxSize().padding(10.dp), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            Column(Modifier.weight(1f).verticalScroll(rememberScrollState())) { cards() }
            Column(Modifier.weight(1f)) { logPane() }
        }
    } else {
        Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp)) {
            cards()
            Spacer(Modifier.height(12.dp))
            logPane()
            Spacer(Modifier.height(70.dp))
        }
    }

    confirm?.let { r ->
        AlertDialog(
            onDismissRequest = { confirm = null },
            containerColor = Panel,
            title = { Text("Wheels off the ground?", color = Red) },
            text = {
                Text("This routine spins the motors up until they stall. If the robot is " +
                     "on the floor it will drive away from you at full speed.\n\n" +
                     "Lift it onto a stand first.", color = TextDim, fontSize = 13.sp)
            },
            confirmButton = {
                TextButton(onClick = { vm.autoTune(r.cmd); confirm = null }) {
                    Text("They're off the ground — run it", color = Red)
                }
            },
            dismissButton = { TextButton(onClick = { confirm = null }) { Text("Cancel") } }
        )
    }
}
