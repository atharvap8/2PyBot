package `in`.twopybot.app.ui.screens

import android.graphics.Bitmap
import android.graphics.PointF
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.*
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.delay
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import `in`.twopybot.app.BuildConfig
import `in`.twopybot.app.export.RobotReportExporter

@Composable
fun MapScreen(vm: BotViewModel) {
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val trail by vm.trail.collectAsStateWithLifecycle()
    val debug by vm.repo.debug.collectAsStateWithLifecycle()
    val host by vm.repo.host.collectAsStateWithLifecycle()
    val context = LocalContext.current
    val scope = rememberCoroutineScope()
    var includeTelemetry by remember { mutableStateOf(true) }
    var exportImage by remember { mutableStateOf<Bitmap?>(null) }
    fun captureMap(): Bitmap {
        val stats = linkedMapOf("speed m/s" to "%.3f".format(tel.v), "pitch deg" to "%.2f".format(tel.pitch),
            "x m" to "%.3f".format(tel.x), "y m" to "%.3f".format(tel.y), "heading deg" to "%.1f".format(tel.heading),
            "balancing" to tel.balancing.toString(), "stale" to tel.stale.toString())
        debug.toSortedMap().forEach { (key, value) -> stats[key] = "%.4f".format(value) }
        return RobotReportExporter.renderTrail(trail.map { PointF(it.first, it.second) },tel.x,tel.y,tel.heading,stats,includeTelemetry)
    }
    val pdfExport = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("application/pdf")) { uri ->
        val image = exportImage
        if (uri != null && image != null) scope.launch {
            runCatching { withContext(Dispatchers.IO) {
                context.contentResolver.openOutputStream(uri,"w")?.use { RobotReportExporter.writeMapPdf(it,image,host,BuildConfig.VERSION_NAME) }
                    ?: error("Cannot open destination")
            } }.fold({ vm.toast("Trail PDF exported") }, { vm.toast("Export failed: ${it.message}") })
            image.recycle(); exportImage = null
        } else { image?.recycle(); exportImage = null }
    }
    val pngExport = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("image/png")) { uri ->
        val image = exportImage
        if (uri != null && image != null) scope.launch {
            runCatching { withContext(Dispatchers.IO) {
                context.contentResolver.openOutputStream(uri,"w")?.use { check(image.compress(Bitmap.CompressFormat.PNG,100,it)) }
                    ?: error("Cannot open destination")
            } }.fold({ vm.toast("Trail PNG exported") }, { vm.toast("Export failed: ${it.message}") })
            image.recycle(); exportImage = null
        } else { image?.recycle(); exportImage = null }
    }

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
        Row {
            Checkbox(checked=includeTelemetry,onCheckedChange={ includeTelemetry=it })
            Text("Include telemetry in export",fontSize=12.sp,modifier=Modifier.padding(top=12.dp))
        }
        Row(horizontalArrangement=Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick={ exportImage?.recycle(); exportImage=captureMap(); pdfExport.launch("2PyBot-trail-${System.currentTimeMillis()}.pdf") }) { Text("PDF") }
            OutlinedButton(onClick={ exportImage?.recycle(); exportImage=captureMap(); pngExport.launch("2PyBot-trail-${System.currentTimeMillis()}.png") }) { Text("PNG") }
        }
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
