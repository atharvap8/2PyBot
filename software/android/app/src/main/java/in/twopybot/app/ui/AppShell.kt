package `in`.twopybot.app.ui

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import `in`.twopybot.app.BuildConfig
import `in`.twopybot.app.ui.screens.*
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import kotlinx.coroutines.flow.collectLatest

private data class Destination(val id:String,val title:String,val detail:String,val icon:ImageVector)
private val destinations=listOf(
    Destination("drive","Drive","Robot & instruments",Icons.Default.DirectionsCar),
    Destination("live","Live","Camera & telemetry",Icons.Default.Videocam),
    Destination("auto","Routes","Teach & original-speed replay",Icons.Default.Route),
    Destination("map","Map","Odometry & trail",Icons.Default.Map),
    Destination("tune","Tune","Controller parameters",Icons.Default.Tune),
    Destination("cam","Camera","Optics & perception",Icons.Default.CameraAlt),
    Destination("data","Sessions","Record & export",Icons.Default.FolderOpen),
    Destination("set","Setup","Connection & firmware",Icons.Default.Settings))
private val primary=destinations.take(4)

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun RobotApp(vm:BotViewModel=viewModel()) {
    var selected by rememberSaveable { mutableStateOf("drive") }
    var more by rememberSaveable { mutableStateOf(false) }
    val snackbar=remember { SnackbarHostState() }
    val tel by vm.repo.telemetry.collectAsStateWithLifecycle()
    val connected by vm.repo.connected.collectAsStateWithLifecycle()
    val unsaved by vm.repo.unsaved.collectAsStateWithLifecycle()
    val current=destinations.first { it.id==selected }
    val online=connected && !tel.stale
    val linkColor=if(online) Green else TextDim
    val status=if(!online) "OFFLINE" else if(tel.balancing) "BALANCING" else "CONNECTED"
    LaunchedEffect(vm) { vm.snack.collectLatest { if(it!=null) { snackbar.showSnackbar(it);vm.clearToast() } } }
    BackHandler(selected!="drive" && !more) { selected="drive" }
    // Moving between adaptive layouts keeps the active native/WebView screen.
    val workspace=remember(vm) {movableContentOf<String> {destination ->
        when(destination) {
            "drive" -> DrivingScreen(vm)
            "live" -> LiveScreen(vm)
            "auto" -> AutoScreen(vm)
            "map" -> MapScreen(vm)
            "tune" -> TuneScreen(vm)
            "cam" -> CameraScreen(vm)
            "data" -> RecorderScreen(vm)
            else -> SettingsScreen(vm)
        }
    }}
    val landscape=isLandscape()
    BoxWithConstraints(Modifier.fillMaxSize().background(Ink).windowInsetsPadding(WindowInsets.safeDrawing)) {
        val rail=landscape || maxWidth>=600.dp
        Scaffold(containerColor=Ink,contentWindowInsets=WindowInsets(0,0,0,0),
            snackbarHost={SnackbarHost(snackbar)},
            topBar={ if(!rail) Row(Modifier.fillMaxWidth().height(54.dp).padding(horizontal=18.dp),
                verticalAlignment=Alignment.CenterVertically,horizontalArrangement=Arrangement.spacedBy(10.dp)) {
                Column(Modifier.weight(1f)) {
                    Text("2PyBot / ${current.title}",fontSize=17.sp,fontWeight=FontWeight.SemiBold,color=TextHi)
                    Text(status, fontSize=10.sp,letterSpacing=1.sp,color=linkColor)
                }
                if(unsaved) Text("UNSAVED",color=Amber,fontSize=10.sp)
                Button(onClick={vm.estop()},contentPadding=PaddingValues(horizontal=12.dp),
                    modifier=Modifier.height(40.dp),shape=RoundedCornerShape(10.dp),
                    colors=ButtonDefaults.buttonColors(containerColor=Red.copy(alpha=.16f),contentColor=Red)) {
                    Icon(Icons.Default.Stop,"Stop motors",Modifier.size(18.dp));Spacer(Modifier.width(4.dp));Text("STOP",fontSize=12.sp)
                }
            } },
            bottomBar={ if(!rail) NavigationBar(containerColor=Panel,tonalElevation=0.dp,
                modifier=Modifier.height(68.dp),windowInsets=WindowInsets(0,0,0,0)) {
                (primary+Destination("more","More","All workspaces",Icons.Default.GridView)).forEach { d ->
                    NavigationBarItem(selected=if(d.id=="more") selected !in primary.map { it.id } else selected==d.id,
                        onClick={if(d.id=="more") more=true else selected=d.id},
                        icon={Icon(d.icon,d.title,Modifier.size(22.dp))},label={Text(d.title,fontSize=11.sp)},
                        colors=NavigationBarItemDefaults.colors(selectedIconColor=Cyan,selectedTextColor=TextHi,
                            indicatorColor=PanelSoft,unselectedIconColor=TextDim,unselectedTextColor=TextDim))
                }
            } }) { padding ->
            Row(Modifier.fillMaxSize().padding(padding)) {
                if(rail) Column(Modifier.width(72.dp).fillMaxHeight().background(Panel).padding(horizontal=6.dp),
                    horizontalAlignment=Alignment.CenterHorizontally) {
                    Text("2P",fontSize=17.sp,fontWeight=FontWeight.Bold,color=TextHi,modifier=Modifier.padding(vertical=10.dp))
                    Column(Modifier.weight(1f).verticalScroll(rememberScrollState()),verticalArrangement=Arrangement.spacedBy(4.dp)) {
                        (primary+Destination("more","More","All workspaces",Icons.Default.GridView)).forEach { d ->
                            val active=if(d.id=="more") selected !in primary.map { it.id } else selected==d.id
                            Column(Modifier.fillMaxWidth().heightIn(min=54.dp).clip(RoundedCornerShape(12.dp))
                                .background(if(active) PanelSoft else Color.Transparent)
                                .clickable {if(d.id=="more") more=true else selected=d.id}.padding(vertical=7.dp),
                                horizontalAlignment=Alignment.CenterHorizontally,verticalArrangement=Arrangement.spacedBy(3.dp)) {
                                Icon(d.icon,d.title,Modifier.size(21.dp),tint=if(active) Cyan else TextDim)
                                Text(d.title,fontSize=10.sp,color=if(active) TextHi else TextDim)
                            }
                        }
                    }
                    Box(Modifier.size(6.dp).clip(RoundedCornerShape(3.dp)).background(linkColor))
                    Spacer(Modifier.height(8.dp))
                    Column(Modifier.fillMaxWidth().height(52.dp).clip(RoundedCornerShape(12.dp))
                        .background(Red.copy(alpha=.14f)).clickable {vm.estop()},
                        horizontalAlignment=Alignment.CenterHorizontally,verticalArrangement=Arrangement.Center) {
                        Icon(Icons.Default.Stop,"Stop motors",Modifier.size(21.dp),tint=Red)
                        Text("STOP",fontSize=10.sp,color=Red,fontWeight=FontWeight.Bold)
                    }
                    Spacer(Modifier.height(8.dp))
                }
                Box(Modifier.weight(1f).fillMaxHeight()) {workspace(selected)}
            }
        }
    }
    if(more) ModalBottomSheet(onDismissRequest={more=false},containerColor=Panel) {
        Column(Modifier.fillMaxWidth().heightIn(max=560.dp).verticalScroll(rememberScrollState()).padding(horizontal=20.dp)) {
            Text("Workspaces",style=MaterialTheme.typography.titleLarge,color=TextHi)
            Text("2PyBot · ${BuildConfig.VERSION_NAME}",fontSize=12.sp,color=TextDim,modifier=Modifier.padding(top=4.dp,bottom=14.dp))
            destinations.chunked(2).forEach { pair -> Row(Modifier.fillMaxWidth().padding(bottom=10.dp),horizontalArrangement=Arrangement.spacedBy(10.dp)) {
                pair.forEach { d ->
                    OutlinedCard(onClick={selected=d.id;more=false},modifier=Modifier.weight(1f),
                        colors=CardDefaults.outlinedCardColors(containerColor=if(selected==d.id) PanelSoft else Panel)) {
                        Column(Modifier.padding(16.dp)) {
                            Icon(d.icon,null,tint=Cyan,modifier=Modifier.size(24.dp))
                            Text(d.title,color=TextHi,fontWeight=FontWeight.Medium,modifier=Modifier.padding(top=12.dp))
                            Text(d.detail,color=TextDim,fontSize=11.sp,maxLines=1,overflow=TextOverflow.Ellipsis)
                        }
                    }
                }
            } }
            Spacer(Modifier.height(20.dp))
        }
    }
}
