package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.net.WhepClient
import `in`.twopybot.app.ui.components.*
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.launch
import org.webrtc.RendererCommon
import org.webrtc.SurfaceViewRenderer

@Composable
fun LiveScreen(vm: BotViewModel) {
    val ctx = LocalContext.current
    val scope = rememberCoroutineScope()
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val dbg by vm.repo.debug.collectAsStateWithLifecycle()
    val pad by vm.repo.pad.collectAsStateWithLifecycle()
    val conn by vm.repo.connected.collectAsStateWithLifecycle()
    val unsaved by vm.repo.unsaved.collectAsStateWithLifecycle()

    val whep = remember { WhepClient(ctx) }
    var videoState by remember { mutableStateOf("idle") }
    var renderer by remember { mutableStateOf<SurfaceViewRenderer?>(null) }
    var showControls by rememberSaveable { mutableStateOf(false) }

    DisposableEffect(Unit) {
        whep.onState = { videoState = it }
        onDispose { renderer?.release(); whep.release() }
    }

    // ---- pieces, so portrait and landscape can arrange the same content ----
    // Keep the native video renderer alive when rotation moves it between panes.
    val videoPane = remember(whep) { movableContentOf<Modifier> { mod ->
        Box(mod.clip(RoundedCornerShape(16.dp)).background(Ink)
               .border(1.dp, Line, RoundedCornerShape(16.dp))) {
            AndroidView(
                modifier = Modifier.fillMaxSize(),
                factory = { c ->
                    SurfaceViewRenderer(c).apply {
                        init(whep.egl().eglBaseContext, null)
                        setScalingType(RendererCommon.ScalingType.SCALE_ASPECT_FIT)
                        setEnableHardwareScaler(true)
                        renderer = this
                    }
                }
            )
            if (videoState != "connected")
                Text(videoState, color = TextDim, style = Mono.copy(fontSize = 13.sp),
                     modifier = Modifier.align(Alignment.Center))
            Row(Modifier.align(Alignment.TopStart).padding(6.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                Text("LIVE / WebRTC · 16:9", style = Mono.copy(fontSize = 10.sp), color = TextHi,
                     modifier = Modifier.clip(RoundedCornerShape(6.dp))
                        .background(Ink.copy(alpha = .65f)).padding(horizontal = 6.dp, vertical = 2.dp))
            }
            // Controls float ON the video in landscape so they cost no height
            Row(Modifier.align(Alignment.BottomStart).padding(8.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                FilledTonalButton(
                    onClick = { scope.launch { renderer?.let { whep.connect(vm.repo.whepUrl(), it) } } },
                    contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
                    colors = ButtonDefaults.filledTonalButtonColors(
                        containerColor = Cyan.copy(alpha = .85f), contentColor = Ink)
                ) { Text(if (videoState == "connected") "Reconnect" else "Connect", fontSize = 12.sp) }
                FilledTonalButton(
                    onClick = { whep.close(); videoState = "stopped" },
                    contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
                    colors = ButtonDefaults.filledTonalButtonColors(
                        containerColor = Panel.copy(alpha = .9f), contentColor = TextHi)
                ) { Text("Stop video", fontSize = 12.sp) }
            }
        }
    } }

    val gauges: @Composable (Modifier) -> Unit = { mod ->
        Row(mod, horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            Column(Modifier.weight(1f)) {
                Text("ATTITUDE", fontSize = 9.sp, letterSpacing = 1.sp, color = TextDim)
                AttitudeGauge(tel.pitch, dbg["w"] ?: 0f, Modifier.fillMaxWidth().height(96.dp))
            }
            Column(Modifier.weight(1f)) {
                Text("PAD (MIRROR)", fontSize = 9.sp, letterSpacing = 1.sp, color = TextDim)
                StickMirror(pad, Modifier.fillMaxWidth().height(96.dp))
            }
        }
    }

    val numbers: @Composable ColumnScope.() -> Unit = {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            Tile("pitch", "%.1f°".format(tel.pitch), Cyan, Modifier.weight(1f))
            Tile("rate", "%.0f°/s".format(dbg["w"] ?: 0f), Violet, Modifier.weight(1f))
            Tile("speed", "%.2f".format(tel.v), Green, Modifier.weight(1f))
        }
        Spacer(Modifier.height(6.dp))
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            Tile("pos err", dbg["ex"]?.let { "%.1fcm".format(it * 100) } ?: "—",
                 if ((dbg["ex"] ?: 0f) > 0.05f) Amber else Cyan, Modifier.weight(1f))
            Tile("sat V/A", "${if (pad.satV) 1 else 0}/${if (pad.satA) 1 else 0}",
                 if (pad.satV || pad.satA) Amber else TextDim, Modifier.weight(1f))
            Tile("loop", "%.1fms".format(pad.loopMaxMs),
                 if (pad.loopMaxMs > 7f) Red else TextDim, Modifier.weight(1f))
        }
    }

    val gestures: @Composable ColumnScope.() -> Unit = {
        FlowRowCompat(listOf(
            "nod yes" to "G,yes", "nod no" to "G,no", "spin" to "G,spin",
            "dance" to "G,dance", "stiff on" to "H,1", "stiff off" to "H,0",
            "climb on" to "H,2", "climb off" to "H,0", "centre cam" to "A,0",
        )) { vm.command(it) }
    }

    AdaptiveWorkspace(portraitRatio=16f/9f,primary=videoPane,tools={
        WorkspaceTitle("Live view","Camera-first control workspace")
        StatusStrip(tel,conn,unsaved)
        Spacer(Modifier.height(4.dp))
        numbers()
        InstrumentRow("Heading","%.0f°".format(tel.heading))
        InstrumentRow("Camera link",videoState,if(videoState=="connected") Green else TextDim)
        TextButton(onClick={showControls=!showControls},modifier=Modifier.fillMaxWidth()) {
            Text(if(showControls) "Hide instruments & actions" else "Instruments & actions",fontSize=12.sp)
        }
        if(showControls) {
            gauges(Modifier.fillMaxWidth())
            Section("Robot actions") {gestures()}
        }
    })
}

@Composable
fun FlowRowCompat(items: List<Pair<String, String>>, onClick: (String) -> Unit) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        items.chunked(3).forEach { row ->
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                row.forEach { (label, cmd) ->
                    OutlinedButton(onClick = { onClick(cmd) }, modifier = Modifier.weight(1f),
                        contentPadding = PaddingValues(horizontal = 4.dp, vertical = 8.dp)) {
                        Text(label, fontSize = 12.sp, maxLines = 1)
                    }
                }
                repeat(3 - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}
