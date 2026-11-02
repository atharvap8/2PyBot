package `in`.twopybot.app.net

import android.content.Context
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.first
import org.webrtc.VideoSink

/** ViewModel-owned playback. Layout/surface changes only rebind its video sink. */
class LiveVideoSession(context: Context, private val scope: CoroutineScope) {
    private val client = WhepClient(context.applicationContext)
    private val mutableState = MutableStateFlow("idle")
    val state: StateFlow<String> = mutableState
    private val mutableRequested = MutableStateFlow(false)
    val requested: StateFlow<Boolean> = mutableRequested
    private var requestId = 0L
    private var job: Job? = null

    init { client.onState = { mutableState.value = it } }
    fun egl() = client.egl()
    fun attach(sink: VideoSink) = client.attachSink(sink)
    fun detach(sink: VideoSink) = client.detachSink(sink)
    private fun failed(value: String) = value.startsWith("error:") || value in setOf("failed", "closed", "disconnected")

    fun start(url: String) {
        val epoch = ++requestId
        val previous = job
        previous?.cancel()
        client.close()
        mutableRequested.value = true
        mutableState.value = "connecting"
        job = scope.launch {
            previous?.join()
            while (isActive && epoch == requestId && mutableRequested.value) {
                try {
                    mutableState.value = "connecting"
                    withTimeout(15_000) { client.connect(url) }
                    val initial = withTimeout(15_000) { state.first { it == "connected" || failed(it) } }
                    if (initial == "connected") {
                        while (isActive) {
                            state.first { failed(it) }
                            delay(1_200) // permit a transient ICE disconnection to recover
                            if (state.value != "connected") break
                        }
                    }
                } catch (timeout: TimeoutCancellationException) {
                    mutableState.value = "error: connection timeout"
                } catch (cancelled: CancellationException) {
                    throw cancelled
                } catch (error: Exception) {
                    mutableState.value = "error: ${error.message}"
                }
                if (epoch != requestId || !mutableRequested.value) break
                client.close()
                mutableState.value = "reconnecting"
                delay(1_000)
            }
        }
    }

    fun stop() {
        requestId++
        mutableRequested.value = false
        job?.cancel()
        client.close()
        mutableState.value = "stopped"
    }

    fun release() { stop(); client.release() }
}
