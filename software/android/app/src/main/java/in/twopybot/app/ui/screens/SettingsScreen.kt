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
import `in`.twopybot.app.ui.components.Section
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel

@Composable
fun SettingsScreen(vm: BotViewModel) {
    val host by vm.repo.host.collectAsStateWithLifecycle()
    val stream by vm.repo.stream.collectAsStateWithLifecycle()
    val log by vm.repo.logLines.collectAsStateWithLifecycle()
    var field by remember(host) { mutableStateOf(host) }

    Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp)) {

        Section("Robot address",
            "10.42.0.1 is the Radxa's own hotspot gateway. Use the LAN address instead " +
            "if you are on shared Wi-Fi.") {
            OutlinedTextField(
                value = field, onValueChange = { field = it },
                singleLine = true, label = { Text("host or IP") },
                textStyle = Mono, modifier = Modifier.fillMaxWidth()
            )
            Spacer(Modifier.height(8.dp))
            Button(onClick = { vm.setHost(field) }, modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(containerColor = Cyan, contentColor = Ink)) {
                Text("Connect")
            }
        }

        Section("Stream") {
            Text("encoder  ${stream.encoder.ifEmpty { "—" }}", style = Mono.copy(fontSize = 12.sp), color = TextDim)
            Text("bitrate  ${stream.bitrate.ifEmpty { "—" }}", style = Mono.copy(fontSize = 12.sp), color = TextDim)
            Text("running  ${stream.running}", style = Mono.copy(fontSize = 12.sp), color = TextDim)
            if (stream.error.isNotEmpty())
                Text(stream.error, style = Mono.copy(fontSize = 11.sp), color = Red)
        }

        Section("Safety",
            "The gamepad is paired to the ESP32 over Bluetooth and stays that way. This app " +
            "mirrors pad state for display and can tune parameters, but it cannot drive. " +
            "The console also refuses remote ARM on purpose: a remote link may STOP the " +
            "robot, never START it.") {
            Button(onClick = { vm.estop() }, modifier = Modifier.fillMaxWidth(),
                colors = ButtonDefaults.buttonColors(containerColor = Red, contentColor = androidx.compose.ui.graphics.Color.White)) {
                Text("E-STOP")
            }
        }

        Section("Robot log") {
            Text(log.takeLast(40).joinToString("\n").ifEmpty { "—" },
                 style = Mono.copy(fontSize = 10.sp), color = TextDim, lineHeight = 14.sp)
        }

        Spacer(Modifier.height(70.dp))
    }
}
