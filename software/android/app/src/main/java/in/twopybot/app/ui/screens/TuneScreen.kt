package `in`.twopybot.app.ui.screens

import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Search
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import `in`.twopybot.app.ui.isLandscape
import `in`.twopybot.app.ui.components.ParamSlider
import `in`.twopybot.app.ui.components.Section
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel

/**
 * Tuning. The group list is NOT hardcoded — it comes from the firmware's
 * parameter dump, so adding a tunable to params.h makes it appear here with
 * no app change. Same contract the web console uses.
 */
private val HINTS = mapOf(
    "BALANCE" to "LQR/LQI gains. K4 is gyro-rate damping and its SIGN is the most " +
        "dangerous value on the robot. Change one gain at a time and watch the attitude gauge.",
    "MOTOR" to "M0601 BLDC geometry, common/turn current loops and speed limits. " +
        "ISGN and RPMSGN are fixed at -1. Stop before saving or changing motor mode/direction.",
    "DRIVE"   to "Speed scaling and stick shaping. SNAPIN is the cone around straight-ahead " +
        "where steering is forced to zero.",
    "YAW"     to "Differential steering and heading hold. YAWKP/YAWKD act on encoder counts, " +
        "so they need rescaling whenever the wheel radius changes.",
    "SAFETY"  to "Arming window, fall cutoffs, link timeouts.",
    "IMU"     to "Filter cutoff, Mahony gains, sign conventions. Verify a sign by tilting the " +
        "robot and watching the gauge before you trust it.",
    "CLIMB"   to "Ramp mode (D-pad DOWN). C1-C5 are a SEPARATE gain set used only " +
        "while climbing — the always-on integral absorbs the slope's shifted " +
        "equilibrium, which is why uphill tracking has no steady-state error.",
    "LED"     to "Ring brightness cap and which LED faces forward.",
    "PAYLOAD" to "Pan/zoom servo rates and travel. Torch duty is hard-capped in firmware.",
)

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun TuneScreen(vm: BotViewModel) {
    val params by vm.repo.params.collectAsStateWithLifecycle()
    val derived by vm.repo.derived.collectAsStateWithLifecycle()
    val unsaved by vm.repo.unsaved.collectAsStateWithLifecycle()

    val groups = remember(params) { params.map { it.group }.distinct() }
    var sel by remember(groups) { mutableStateOf(groups.firstOrNull() ?: "") }
    var query by rememberSaveable { mutableStateOf("") }
    val shown = params.filter { (query.isNotBlank() || it.group==sel) &&
        (query.isBlank() || "${it.key} ${it.desc} ${it.group}".contains(query,ignoreCase=true)) }

    Column(Modifier.fillMaxSize()) {

        if (params.isEmpty()) {
            Box(Modifier.fillMaxSize(), contentAlignment = androidx.compose.ui.Alignment.Center) {
                Text("waiting for the robot's parameter dump…", color = TextDim, fontSize = 13.sp)
            }
            return
        }

        val land = isLandscape()
        OutlinedTextField(query,{query=it},modifier=Modifier.fillMaxWidth().padding(horizontal=12.dp,vertical=6.dp),
            singleLine=true,placeholder={Text("Find a parameter across all groups",fontSize=12.sp)},
            leadingIcon={Icon(Icons.Default.Search,null,Modifier.size(20.dp))},
            trailingIcon={Text("${shown.size}",fontSize=12.sp,color=TextDim,modifier=Modifier.padding(end=12.dp))},
            textStyle=Mono.copy(fontSize=13.sp))

        // The group list is NOT hardcoded: it is whatever the firmware's P?
        // dump reported. Add a tunable in params.h and it shows up here.
        val groupList: @Composable () -> Unit = {
            groups.forEach { g ->
                val n = params.count { it.group == g }
                FilterChip(
                    selected = g == sel, onClick = { sel = g },
                    label = { Text(if (land) "$g  $n" else g, fontSize = 12.sp) },
                    modifier = if (land) Modifier.fillMaxWidth().padding(vertical = 2.dp) else Modifier,
                    colors = FilterChipDefaults.filterChipColors(
                        selectedContainerColor = Cyan.copy(alpha = 0.18f),
                        selectedLabelColor = Cyan))
            }
        }

        val sliders: @Composable () -> Unit = {
            LazyColumn(Modifier.fillMaxSize().padding(horizontal = 12.dp)) {
                item {
                    Section(sel, HINTS[sel]) {
                        if (derived.isNotEmpty() && (sel == "MOTOR" || sel == "BALANCE")) {
                            Text(derived.entries.joinToString("   ") {
                                "${it.key} ${"%.1f".format(it.value)}" },
                                style = Mono.copy(fontSize = 11.sp), color = Cyan)
                            Spacer(Modifier.height(8.dp))
                        }
                    }
                }
                items(shown.size,key={shown[it].key}) { index ->
                    val p=shown[index]
                    ParamSlider(p) {v->vm.setParam(p.key,v)}
                    HorizontalDivider(color=Line.copy(alpha=.5f))
                }
                item { Spacer(Modifier.height(14.dp)) }
            }
        }

        if (land) {
            Row(Modifier.weight(1f)) {
                Column(
                    Modifier.width(132.dp).fillMaxHeight()
                        .verticalScroll(rememberScrollState())
                        .padding(horizontal = 8.dp, vertical = 8.dp)
                ) { groupList() }
                Box(Modifier.weight(1f)) { sliders() }
            }
        } else {
            Row(Modifier.fillMaxWidth().horizontalScroll(rememberScrollState())
                    .padding(horizontal = 10.dp, vertical = 8.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp)) { groupList() }
            Box(Modifier.weight(1f)) { sliders() }
        }

        // persistence bar — nothing survives a reboot until Save is pressed
        Surface(color = Panel, tonalElevation = 3.dp) {
            Row(Modifier.fillMaxWidth().padding(10.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { vm.paramAction("save") }, modifier = Modifier.weight(1.4f),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (unsaved) Green else Line,
                        contentColor = if (unsaved) Ink else TextDim)) {
                    Text(if (unsaved) "SAVE TO ROBOT" else "saved", fontSize = 13.sp)
                }
                OutlinedButton(onClick = { vm.paramAction("load") }, modifier = Modifier.weight(1f),
                    contentPadding = PaddingValues(4.dp)) { Text("Reload", fontSize = 12.sp) }
                OutlinedButton(onClick = { vm.paramAction("defaults") }, modifier = Modifier.weight(1f),
                    contentPadding = PaddingValues(4.dp)) { Text("Defaults", fontSize = 12.sp) }
            }
        }
    }
}
