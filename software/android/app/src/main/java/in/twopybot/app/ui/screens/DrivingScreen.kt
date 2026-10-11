package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.components.RobotScene
import `in`.twopybot.app.ui.components.TrailMap
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.delay

private val DriveInk=Color(0xff4c555e)
private val DrivePanel=Color(0xffe8eaed)

@Composable
fun DrivingScreen(vm: BotViewModel) {
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val tof by vm.repo.tof.collectAsStateWithLifecycle()
    val debug by vm.repo.debug.collectAsStateWithLifecycle()
    val trail by vm.trail.collectAsStateWithLifecycle()
    val connected by vm.repo.connected.collectAsStateWithLifecycle()
    LaunchedEffect(Unit) { while(true) { vm.refreshTrail(); delay(1000) } }
    val numbers: @Composable () -> Unit = {
        Text("%.2f".format(tel.v),fontSize=38.sp,color=DriveInk)
        Text("m/s",color=DriveInk,fontSize=12.sp)
        Text(if(!connected || tel.stale) "NO LINK" else if(tel.balancing) "BALANCING" else "IDLE",color=DriveInk)
        Text("Pitch %.1f°".format(tel.pitch),color=DriveInk,fontSize=12.sp)
        Text("Loop %.1f ms".format((debug["loopMax"] ?: 0f)/1000),color=DriveInk,fontSize=12.sp)
        HorizontalDivider(Modifier.padding(vertical=8.dp))
        Text("${tof.validCount}/64 usable · ${(tof.coverage*100).toInt()}%",color=DriveInk,fontSize=12.sp)
        Text(tof.nearest?.let { "Corridor %.2f m".format(it) } ?: "Depth unknown",color=DriveInk,fontSize=12.sp)
        Row {
            Checkbox(tof.guardEnabled,{ vm.setToFGuard(it) },enabled=tof.guardSupported && connected)
            Text("Guard",color=DriveInk,modifier=Modifier.padding(top=12.dp))
        }
        Text(if(tof.guardEnabled) tof.guardReason else "Guard off",color=DriveInk,fontSize=11.sp)
        tof.forwardLimit?.let { Text("Cap %.2f m/s".format(it),color=DriveInk,fontSize=11.sp) }
        if(tof.error.isNotEmpty()) Text(tof.error,color=DriveInk,fontSize=11.sp)
        Spacer(Modifier.height(8.dp))
        Text("Partial coverage reduces speed. Unknown/stale depth holds forward requests. " +
            "Reverse, steering and balance recovery stay available.",color=DriveInk,fontSize=10.sp)
    }
    val map: @Composable (Modifier) -> Unit = { modifier ->
        Column(modifier.clip(RoundedCornerShape(12.dp)).background(DrivePanel).padding(8.dp)) {
            Text("TRAIL",color=DriveInk,fontSize=12.sp)
            TrailMap(trail,tel.x,tel.y,tel.heading,Modifier.weight(1f).fillMaxWidth())
            Text("Encoder dead reckoning",color=DriveInk,fontSize=10.sp)
        }
    }
    if(isLandscape()) {
        Row(Modifier.fillMaxSize().background(Color(0xffdfe2e5)).padding(8.dp),horizontalArrangement=Arrangement.spacedBy(8.dp)) {
            map(Modifier.weight(.9f).fillMaxHeight())
            RobotScene(tel,tof,Modifier.weight(2.2f).fillMaxHeight().clip(RoundedCornerShape(12.dp)))
            Column(Modifier.width(150.dp).fillMaxHeight().background(DrivePanel).verticalScroll(rememberScrollState()).padding(10.dp)) { numbers() }
        }
    } else {
        Column(Modifier.fillMaxSize().background(Color(0xffdfe2e5)).verticalScroll(rememberScrollState()).padding(10.dp)) {
            RobotScene(tel,tof,Modifier.fillMaxWidth().height(350.dp).clip(RoundedCornerShape(12.dp)))
            Row(Modifier.fillMaxWidth().height(340.dp).padding(top=10.dp),horizontalArrangement=Arrangement.spacedBy(10.dp)) {
                map(Modifier.weight(1f).fillMaxHeight())
                Column(Modifier.weight(1f).fillMaxHeight().verticalScroll(rememberScrollState()).padding(8.dp)) { numbers() }
            }
        }
    }
}
