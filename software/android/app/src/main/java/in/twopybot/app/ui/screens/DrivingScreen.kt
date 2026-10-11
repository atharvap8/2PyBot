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
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.RobotScene
import `in`.twopybot.app.ui.components.TrailMap
import `in`.twopybot.app.ui.components.SupportEvidence
import `in`.twopybot.app.ui.components.supportEvidence
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.delay

@Composable
fun DrivingScreen(vm: BotViewModel) {
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val tof by vm.repo.tof.collectAsStateWithLifecycle()
    val debug by vm.repo.debug.collectAsStateWithLifecycle()
    val trail by vm.trail.collectAsStateWithLifecycle()
    val connected by vm.repo.connected.collectAsStateWithLifecycle()
    val params by vm.repo.params.collectAsStateWithLifecycle()
    val pitchSign = if((params.firstOrNull { it.key == "PITCHSGN" }?.value ?: -1f) > 0f) 1f else -1f
    val support = supportEvidence(tel,debug,connected)
    var follow by rememberSaveable { mutableStateOf(true) }
    var reset by remember { mutableIntStateOf(0) }
    var showMap by rememberSaveable { mutableStateOf(false) }
    var details by rememberSaveable { mutableStateOf(false) }
    var guides by rememberSaveable { mutableStateOf(true) }
    var calibrating by rememberSaveable { mutableStateOf(false) }
    var reference by rememberSaveable { mutableStateOf("0.50") }
    LaunchedEffect(Unit) { while(true) { vm.refreshTrail(); delay(1000) } }
    val numbers: @Composable ColumnScope.() -> Unit = {
        Text("LIVE INSTRUMENTS",color=TextDim,fontSize=9.sp,letterSpacing=1.sp)
        Spacer(Modifier.height(8.dp))
        Text(if(!connected || tel.stale) "NO LINK" else if(tel.balancing) "BALANCING" else "IDLE",
            color=if(!connected || tel.stale) Amber else if(tel.balancing) Green else TextDim,fontSize=11.sp)
        Text("Pitch %.1f°".format(tel.pitch*pitchSign),color=TextHi,fontSize=12.sp)
        Text("Heading %.0f°".format(tel.heading),color=TextDim,fontSize=11.sp)
        Text("Loop %.1f ms".format((debug["loopMax"] ?: 0f)/1000),color=TextDim,fontSize=11.sp)
        Text("Encoder X %.2f · Y %.2f m".format(tel.x,tel.y),color=TextDim,fontSize=10.sp)
        Spacer(Modifier.height(8.dp))
        Text(support.label,color=if(support in listOf(SupportEvidence.AIR,SupportEvidence.UNLOADED)) Amber else TextDim,fontSize=11.sp)
        debug["accelMag"]?.let { Text("%.2f g".format(it),color=TextDim,fontSize=10.sp) }
        if(support.symbolicLift > 0f) Text("Lift cue · height unmeasured",color=TextDim,fontSize=9.sp)
        HorizontalDivider(Modifier.padding(vertical=12.dp),color=Line)
        Text("DEPTH",color=TextDim,fontSize=9.sp,letterSpacing=1.sp)
        Text("${tof.validCount}/64 usable · ${(tof.coverage*100).toInt()}%",color=TextHi,fontSize=11.sp)
        Text(tof.nearest?.let { "Obstacle %.2f m".format(it) } ?: "Depth unknown",color=Amber,fontSize=12.sp)
        Row(verticalAlignment=Alignment.CenterVertically) {
            Checkbox(tof.guardEnabled,{ vm.setToFGuard(it) },enabled=tof.guardSupported && connected)
            Text("Guard",color=TextHi,fontSize=12.sp)
        }
        Text(if(tof.guardEnabled) tof.guardReason else "Guard off",color=TextDim,fontSize=10.sp)
        tof.forwardLimit?.let { Text("Cap %.2f m/s".format(it),color=Cyan,fontSize=11.sp) }
        if(tof.error.isNotEmpty()) Text(tof.error,color=Amber,fontSize=10.sp)
        Row(verticalAlignment=Alignment.CenterVertically) {
            Checkbox(guides,{ guides=it })
            Text("Reference guides",color=TextHi,fontSize=11.sp)
        }
        Text(if(tof.rangeCalibrated) "Range bias %+.0f mm".format(tof.rangeOffset*1000) else "Range uncalibrated",color=TextDim,fontSize=10.sp)
        TextButton(onClick={ calibrating=!calibrating },contentPadding=PaddingValues(0.dp)) { Text("Range calibration",fontSize=10.sp) }
        if(calibrating) {
            Text("Flat target distance from sensor face (m)",color=TextDim,fontSize=9.sp)
            OutlinedTextField(reference,{ reference=it },singleLine=true,textStyle=Mono.copy(fontSize=12.sp),modifier=Modifier.fillMaxWidth())
            val measured = reference.toFloatOrNull()?.takeIf { it.isFinite() && it in .20f..3f }
            Button(onClick={ measured?.let { vm.calibrateToF(it) } },enabled=connected && tof.calibrationReady && measured!=null,
                contentPadding=PaddingValues(horizontal=8.dp,vertical=4.dp)) { Text("Save bias",fontSize=11.sp) }
            if(!tof.calibrationReady) Text("Waiting for stable central returns",color=Amber,fontSize=9.sp)
            if(tof.calibrationError.isNotEmpty()) Text(tof.calibrationError,color=Amber,fontSize=9.sp)
        }
    }
    BoxWithConstraints(Modifier.fillMaxSize().background(Ink)) {
        val panelHeight=(maxHeight-92.dp).coerceAtLeast(100.dp)
        RobotScene(tel,tof,Modifier.fillMaxSize(),trail,follow,reset,{ follow=false },debug,connected,pitchSign,guides)
        Column(Modifier.align(Alignment.TopStart).padding(16.dp)) {
            Text("DRIVE",color=TextDim,fontSize=9.sp,letterSpacing=2.sp)
            Text("%.2f".format(tel.v),style=Mono.copy(fontSize=48.sp,fontWeight=FontWeight.Light),color=TextHi)
            Text("m/s",color=TextDim,fontSize=11.sp)
            Spacer(Modifier.height(8.dp))
            Text(when {
                !connected || tel.stale -> "OFFLINE"
                tel.v > .025f -> "FORWARD"
                tel.v < -.025f -> "REVERSE"
                tel.balancing -> "HOLD"
                else -> "READY"
            },color=if(!connected || tel.stale) Amber else Cyan,fontSize=10.sp,letterSpacing=1.sp)
        }
        Column(Modifier.align(Alignment.TopEnd).padding(14.dp).clip(RoundedCornerShape(12.dp))
            .background(Panel.copy(alpha=.82f)).padding(10.dp),horizontalAlignment=Alignment.End) {
            Text(if(follow) "CHASE VIEW" else "EXPLORE",color=TextHi,fontSize=10.sp)
            Text(if(tof.stale) "DEPTH OFFLINE" else if(tof.validCount==0) "NO DEPTH RETURNS" else "${tof.validCount} DEPTH RETURNS",
                color=if(tof.stale || tof.validCount==0) Amber else Green,fontSize=9.sp)
            tof.nearest?.let { Text("%.2f m nearest".format(it),color=TextHi,fontSize=11.sp) }
        }
        if(showMap) {
            Column(Modifier.align(Alignment.BottomStart).padding(start=12.dp,bottom=76.dp)
                .width(132.dp).height(132.dp).clip(RoundedCornerShape(14.dp)).background(Panel.copy(alpha=.9f))
                .border(1.dp,Line,RoundedCornerShape(14.dp)).padding(8.dp)) {
                Text("ENCODER TRAIL",color=TextDim,fontSize=8.sp)
                TrailMap(trail,tel.x,tel.y,tel.heading,Modifier.weight(1f).fillMaxWidth())
            }
        }
        if(details) {
            Column(Modifier.align(Alignment.BottomEnd).padding(end=10.dp,bottom=76.dp)
                .width(204.dp).heightIn(max=panelHeight).clip(RoundedCornerShape(14.dp))
                .background(Panel.copy(alpha=.96f)).border(1.dp,Line,RoundedCornerShape(14.dp))
                .verticalScroll(rememberScrollState()).padding(12.dp)) { numbers() }
        }
        Column(Modifier.align(Alignment.BottomCenter).padding(bottom=8.dp),horizontalAlignment=Alignment.CenterHorizontally) {
            Row(Modifier.clip(RoundedCornerShape(18.dp)).background(Panel.copy(alpha=.94f)).padding(horizontal=4.dp),
                horizontalArrangement=Arrangement.spacedBy(2.dp)) {
                TextButton(onClick={ follow=!follow },contentPadding=PaddingValues(8.dp)) {
                    Text(if(follow) "Follow" else "Explore",color=if(follow) Cyan else TextHi,fontSize=11.sp)
                }
                TextButton(onClick={ reset++; follow=true },contentPadding=PaddingValues(8.dp)) { Text("Center",fontSize=11.sp) }
                TextButton(onClick={ showMap=!showMap },contentPadding=PaddingValues(8.dp)) { Text("Map",color=if(showMap) Cyan else TextDim,fontSize=11.sp) }
                TextButton(onClick={ details=!details },contentPadding=PaddingValues(8.dp)) { Text("Details",color=if(details) Cyan else TextDim,fontSize=11.sp) }
            }
            Text("Reference guidance · measured depth only",color=TextDim,fontSize=8.sp,modifier=Modifier.padding(top=3.dp))
        }
    }
}
