package `in`.twopybot.app.ui.components

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.data.ToFPayload
import `in`.twopybot.app.ui.theme.Red

@Composable
fun RobotScene(tel: Telemetry, tof: ToFPayload, modifier: Modifier = Modifier,
               trail: List<Pair<Float,Float>> = emptyList(), follow: Boolean = true,
               reset: Int = 0, onExplore: () -> Unit = {},
               debug: Map<String,Float> = emptyMap(), connected: Boolean = true,
               pitchForwardSign: Float = -1f, showGuidance: Boolean = true) {
    var failure by remember { mutableStateOf("") }
    var view by remember { mutableStateOf<RobotWorldView?>(null) }
    val explore by rememberUpdatedState(onExplore)
    val lifecycle = LocalLifecycleOwner.current.lifecycle
    DisposableEffect(lifecycle,view) {
        val observer = LifecycleEventObserver { _, event ->
            when(event) {
                Lifecycle.Event.ON_RESUME -> view?.onResume()
                Lifecycle.Event.ON_PAUSE -> view?.onPause()
                else -> Unit
            }
        }
        lifecycle.addObserver(observer)
        onDispose { lifecycle.removeObserver(observer); view?.onPause() }
    }
    Box(modifier) {
        AndroidView(modifier=Modifier.fillMaxSize(),factory = { context ->
            RobotWorldView(context,{ failure = it },{ explore() }).also { view = it }
        },update = { it.update(WorldInput(tel,tof,trail,follow,debug,connected,pitchForwardSign,showGuidance),reset) })
        if(failure.isNotEmpty()) Text(failure,color=Red,modifier=Modifier.align(Alignment.Center).padding(16.dp))
    }
}
