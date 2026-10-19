package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.Section
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlin.math.roundToInt

@Composable
fun FirmwareSection(vm: BotViewModel) {
    val firmware by vm.repo.firmware.collectAsStateWithLifecycle()
    val error by vm.repo.firmwareError.collectAsStateWithLifecycle()
    val starting by vm.firmwareStarting.collectAsStateWithLifecycle()
    val telemetry by vm.repo.telemetry.collectAsStateWithLifecycle()
    Section("Controller firmware", "Reflash the latest successfully compiled build published to this bot. " +
        "Keep the robot still during restart/calibration. Gamepad START is required afterwards.") {
        val build=firmware.latest
        if(build!=null) {
            Text("Latest build  ${build.version}", color=TextHi)
            Text("${build.sizeBytes / 1024} KiB · ${build.builtAt}",style=Mono.copy(fontSize=11.sp),color=TextDim)
            Text("SHA-256  ${build.sha256.take(16)}…",style=Mono.copy(fontSize=11.sp),color=TextDim)
            if(build.sourceDirty)Text("Local build · source contains uncommitted work",style=Mono.copy(fontSize=10.sp),color=TextDim)
        } else Text("No compiled build available",color=TextDim)
        firmware.installed?.let { Text("Last verified installed  ${it.version}",style=Mono.copy(fontSize=11.sp),color=TextDim) }
        Spacer(Modifier.height(8.dp))
        firmware.job?.let { job ->
            Text("${job.status.replace('_',' ')} · ${job.stage}",color=if(job.status=="succeeded") Cyan else TextHi)
            LinearProgressIndicator(progress={ (job.progress/100f).coerceIn(0f,1f) },modifier=Modifier.fillMaxWidth())
            Text("${job.progress.roundToInt()}%  ${job.message}",style=Mono.copy(fontSize=11.sp),color=TextDim)
            if(job.error.isNotBlank())Text(job.error,color=Red,style=Mono.copy(fontSize=11.sp))
            if(job.rolledBack)Text("Previous firmware restored",color=Cyan)
            if(job.status=="succeeded")Text("Verified · ${job.paramsPreserved} tuning values preserved",color=Cyan)
            Spacer(Modifier.height(8.dp))
        }
        if(error.isNotBlank())Text("Connection: $error. An accepted job continues on the bot.",color=Red,style=Mono.copy(fontSize=11.sp))
        if(firmware.blockedReason.isNotBlank())Text(firmware.blockedReason,color=TextDim,style=Mono.copy(fontSize=11.sp))
        Button(onClick={vm.reflashLatest()},enabled=firmware.canRequestFlash() && !starting &&
            !telemetry.balancing && !telemetry.stale && telemetry.startupReady,
            modifier=Modifier.fillMaxWidth(),colors=ButtonDefaults.buttonColors(containerColor=Cyan,contentColor=Ink)) {
            Text(if(starting) "Requesting update…" else if(firmware.busy) "Firmware update in progress" else "Reflash latest compiled firmware")
        }
        OutlinedButton(onClick={vm.refreshFirmware()},modifier=Modifier.fillMaxWidth()) { Text("Refresh firmware status") }
    }
}
