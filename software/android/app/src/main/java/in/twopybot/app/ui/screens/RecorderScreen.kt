package `in`.twopybot.app.ui.screens

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.Section
import `in`.twopybot.app.ui.components.WorkspaceColumns
import `in`.twopybot.app.ui.components.WorkspaceTitle
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

@Composable
fun RecorderScreen(vm: BotViewModel) {
    val recording by vm.recorder.state.collectAsStateWithLifecycle()
    val sessions by vm.recorder.sessions.collectAsStateWithLifecycle()
    val connected by vm.repo.connected.collectAsStateWithLifecycle()
    var terrain by rememberSaveable { mutableStateOf("indoor-flat") }
    var note by rememberSaveable { mutableStateOf("") }
    var exportId by rememberSaveable { mutableStateOf("") }
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    val export = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("application/zip")) { uri ->
        if (uri != null) scope.launch {
            runCatching { withContext(Dispatchers.IO) {
                context.contentResolver.openOutputStream(uri, "w")?.use { vm.recorder.export(exportId, it) }
                    ?: error("Cannot open export destination")
            } }.fold({ vm.toast("Session ZIP exported") }, { vm.toast("Export failed: ${it.message}") })
        }
    }
    WorkspaceColumns(first={
        WorkspaceTitle("Session recorder","Capture, annotate & export")
        Section("Terrain recorder", "Captures all received robot JSON, depth frames, debug fields and app commands. " +
            "Saved locally across screen changes. Camera video is separate.") {
            Text(if (connected) "Robot connected" else "Disconnected — connection gaps are recorded", color = if (connected) Green else Amber)
            OutlinedTextField(terrain, { terrain = it }, label = { Text("Terrain label") }, modifier = Modifier.fillMaxWidth(), singleLine = true)
            OutlinedTextField(note, { note = it }, label = { Text("Annotation / outcome") }, modifier = Modifier.fillMaxWidth())
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { if (recording.active) vm.stopRecording() else vm.startRecording(terrain) }) {
                    Text(if (recording.active) "Stop and save" else "Start recording")
                }
                OutlinedButton(onClick = { vm.labelRecording(terrain, note); note = "" }, enabled = recording.active) { Text("Mark terrain") }
            }
            Text("${if (recording.active) "RECORDING" else "Idle"} · ${recording.rows} events · ${recording.samples} state samples", style = Mono)
            Text("%.1f s · %.2f m · peak %.2f m/s · pitch RMS %.2f°".format(recording.elapsedMs / 1000.0,
                recording.distanceM, recording.peakSpeed, recording.pitchRms), style = Mono)
            if (recording.error.isNotEmpty()) Text(recording.error, color = Red)
        }
    },second={
        Section("Saved sessions", "ZIP contains raw JSONL, telemetry CSV, parameter metadata and diagnostics. " +
            "Labels are human annotations; rewards and terminal states remain open for offline RL preparation.") {
            if (sessions.isEmpty()) Text("No sessions yet", color = TextDim)
            sessions.forEach { id ->
                OutlinedButton(onClick = { exportId = id; export.launch("2PyBot-session-$id.zip") },
                    enabled = !recording.active || recording.id != id, modifier = Modifier.fillMaxWidth()) {
                    Text(if (recording.active && recording.id == id) "$id · active" else "Export $id")
                }
            }
        }
    })
}
