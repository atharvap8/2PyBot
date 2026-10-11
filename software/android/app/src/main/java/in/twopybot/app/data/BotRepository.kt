package `in`.twopybot.app.data

import android.util.Log
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.builtins.MapSerializer
import kotlinx.serialization.builtins.serializer
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.contentOrNull
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import java.util.concurrent.TimeUnit
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException
import kotlin.coroutines.suspendCoroutine

// ============================================================
//  wire models — mirror console/server.py exactly
// ============================================================
@Serializable
data class Telemetry(
    val ms: Long = 0, val encL: Long = 0, val encR: Long = 0,
    val pitch: Float = 0f, val yaw: Float = 0f,
    @SerialName("x_m") val x: Float = 0f,
    @SerialName("y_m") val y: Float = 0f,
    @SerialName("v_ms") val v: Float = 0f,
    val heading: Float = 0f,
    val armed: Boolean = false, val balancing: Boolean = false,
    @SerialName("rx_hz") val rxHz: Float = 0f,
    val stale: Boolean = true,
)

@Serializable
data class Param(
    val key: String, val value: Float, val lo: Float, val hi: Float,
    val group: String, val desc: String, val dirty: Boolean = false,
)

@Serializable
data class ParamsPayload(
    val params: List<Param> = emptyList(),
    val derived: Map<String, Float> = emptyMap(),
    val complete: Boolean = false,
)

@Serializable
data class StreamStatus(
    val running: Boolean = false, val encoder: String = "",
    val bitrate: String = "", val error: String = "", val whep: String = "/cam/whep",
)

@Serializable
data class StatusPayload(
    val connected: Boolean = false,
    val telemetry: Telemetry = Telemetry(),
    val debug: Map<String, Float> = emptyMap(),
    val derived: Map<String, Float> = emptyMap(),
    val stream: StreamStatus = StreamStatus(),
    @SerialName("param_count") val paramCount: Int = 0,
    val tof: ToFPayload = ToFPayload(),
)

@Serializable
data class DepthPoint(
    @SerialName("x_m") val x: Float = 0f,
    @SerialName("y_m") val y: Float = 0f,
    @SerialName("z_m") val z: Float = 0f,
    val confidence: Float = 0f, val confirmed: Boolean = false,
    @SerialName("in_path") val inPath: Boolean = false,
)

@Serializable
data class ToFPayload(
    val state: String = "starting", val stale: Boolean = true, val error: String = "",
    @SerialName("valid_count") val validCount: Int = 0,
    val coverage: Float = 0f, val points: List<DepthPoint> = emptyList(),
    @SerialName("nearest_m") val nearest: Float? = null,
    @SerialName("guard_enabled") val guardEnabled: Boolean = false,
    @SerialName("guard_supported") val guardSupported: Boolean = false,
    @SerialName("guard_active") val guardActive: Boolean = false,
    @SerialName("guard_reason") val guardReason: String = "off",
    @SerialName("forward_limit_mps") val forwardLimit: Float? = null,
    @SerialName("stopping_distance_m") val stoppingDistance: Float = .20f,
    @SerialName("corridor_half_width_m") val halfWidth: Float = .12f,
    @SerialName("turn_envelope_curvature") val curvature: Float = 0f,
    @SerialName("range_calibrated") val rangeCalibrated: Boolean = false,
    @SerialName("range_offset_m") val rangeOffset: Float = 0f,
    @SerialName("calibration_ready") val calibrationReady: Boolean = false,
    @SerialName("calibration_error") val calibrationError: String = "",
)

@Serializable
data class PathPayload(val path: List<List<Float>> = emptyList())

@Serializable
data class RobotLogRow(val text: String = "", val t: Double = 0.0)
@Serializable
data class RobotLogPayload(val log: List<RobotLogRow> = emptyList())

@Serializable
data class CamControl(
    val name: String, val type: String,
    val min: Int = 0, val max: Int = 100, val step: Int = 1,
    val default: Int = 0, val value: Int = 0,
)

@Serializable
data class CamPayload(
    val device: String = "", val controls: List<CamControl> = emptyList(),
    val values: Map<String, Int?> = emptyMap(),
    val profiles: List<String> = emptyList(),
)

// what the app shows as "the gamepad", relayed from the ESP32 by the Cubie
data class PadMirror(
    val fwd: Float = 0f, val steer: Float = 0f,
    val speedHigh: Boolean = false, val vCmd: Float = 0f,
    val satV: Boolean = false, val satA: Boolean = false,
    val loopMaxMs: Float = 0f, val connected: Boolean = false,
)

// ============================================================
//  repository
// ============================================================
class BotRepository(initialHost: String, val recorder: SessionRecorder) {

    private val json = Json { ignoreUnknownKeys = true; isLenient = true; coerceInputValues = true }
    private val FloatMap = MapSerializer(String.serializer(), Float.serializer())
    private val http = OkHttpClient.Builder()
        .connectTimeout(4, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.MILLISECONDS)      // websocket needs no read timeout
        .pingInterval(10, TimeUnit.SECONDS)
        .build()

    private val _host = MutableStateFlow(initialHost)
    val host: StateFlow<String> = _host
    fun setHost(h: String) { _host.value = h.trim(); reconnect() }

    private fun base() = "http://${_host.value}:8080"
    fun whepUrl(path: String = stream.value.whep) = "http://${_host.value}:8889$path"

    val telemetry = MutableStateFlow(Telemetry())
    val debug = MutableStateFlow<Map<String, Float>>(emptyMap())
    val params = MutableStateFlow<List<Param>>(emptyList())
    val derived = MutableStateFlow<Map<String, Float>>(emptyMap())
    val connected = MutableStateFlow(false)
    val logLines = MutableStateFlow<List<String>>(emptyList())
    /** Auto-tune progress. The firmware emits "A,<phase>,<detail>" lines and the
     *  console forwards anything it does not recognise as a log line, so this is
     *  just a filtered view — no extra endpoint needed. */
    val autoLog = MutableStateFlow<List<String>>(emptyList())
    val unsaved = MutableStateFlow(false)
    val stream = MutableStateFlow(StreamStatus())
    val tof = MutableStateFlow(ToFPayload())

    val pad: StateFlow<PadMirror> = combine(debug, connected) { d, c ->
        PadMirror(
            fwd = d["fwd"] ?: 0f, steer = d["str"] ?: 0f,
            speedHigh = (d["spdHi"] ?: 0f) > 0.5f, vCmd = d["vCmd"] ?: 0f,
            satV = (d["satV"] ?: 0f) > 0.5f, satA = (d["satA"] ?: 0f) > 0.5f,
            loopMaxMs = (d["loopMax"] ?: 0f) / 1000f, connected = c,
        )
    }.stateIn(repoScope, SharingStarted.Eagerly, PadMirror())

    private val scope get() = repoScope   // used by the ws + http helpers

    // ---------- HTTP ----------
    private suspend fun req(r: Request): String = suspendCoroutine { cont ->
        val requestBody = okio.Buffer().also { r.body?.writeTo(it) }.readUtf8()
        recorder.record("http_request", """{"method":"${r.method}","path":${JsonPrimitive(r.url.encodedPath)},"body":${JsonPrimitive(requestBody)}}""")
        http.newCall(r).enqueue(object : Callback {
            override fun onFailure(call: Call, e: java.io.IOException) {
                recorder.record("http_error", JsonPrimitive("${r.url.encodedPath}: ${e.message}").toString())
                cont.resumeWithException(e)
            }
            override fun onResponse(call: Call, response: Response) {
                response.use {
                    val body = it.body?.string().orEmpty()
                    recorder.record("http_response", """{"path":${JsonPrimitive(r.url.encodedPath)},"status":${it.code},"body":${runCatching { json.parseToJsonElement(body) }.getOrElse { JsonPrimitive(body) }}}""")
                    if (it.isSuccessful) cont.resume(body)
                    else cont.resumeWithException(java.io.IOException("HTTP ${it.code}: $body"))
                }
            }
        })
    }

    private suspend fun get(path: String) = req(Request.Builder().url(base() + path).build())

    private suspend fun post(path: String, body: String = "{}") = req(
        Request.Builder().url(base() + path)
            .post(body.toRequestBody("application/json".toMediaType())).build()
    )

    suspend fun refreshParams() = withContext(Dispatchers.IO) {
        runCatching {
            val p = json.decodeFromString<ParamsPayload>(get("/api/params"))
            params.value = p.params
            derived.value = p.derived
        }
    }

    suspend fun refreshStatus() = withContext(Dispatchers.IO) {
        runCatching {
            val s = json.decodeFromString<StatusPayload>(get("/api/status"))
            connected.value = s.connected
            telemetry.value = s.telemetry
            derived.value = s.derived
            stream.value = s.stream
            debug.value = s.debug
            tof.value = s.tof
        }
    }

    suspend fun refreshLog() = withContext(Dispatchers.IO) {
        runCatching {
            logLines.value = json.decodeFromString<RobotLogPayload>(get("/api/log")).log.map { it.text }.takeLast(1000)
        }
    }

    suspend fun setParam(key: String, value: Float) = withContext(Dispatchers.IO) {
        runCatching {
            post("/api/params/set", """{"key":"$key","value":$value}""")
            params.value = params.value.map { if (it.key == key) it.copy(value = value, dirty = true) else it }
            unsaved.value = true
        }
    }

    suspend fun paramAction(action: String) = withContext(Dispatchers.IO) {
        runCatching {
            post("/api/params/$action")
            if (action == "save") { unsaved.value = false }
            delay(350); refreshParams()
        }
    }

    suspend fun command(line: String) = withContext(Dispatchers.IO) {
        runCatching { post("/api/command", """{"line":"${line.replace("\\", "\\\\").replace("\"", "\\\"")}"}""") }
    }

    /** Fire an auto-tune routine: wobble | trim | radius | done | stall | stop */
    suspend fun autoTune(routine: String) = withContext(Dispatchers.IO) {
        if (routine == "wobble" || routine == "trim" || routine == "stall")
            autoLog.value = emptyList()
        runCatching { post("/api/command", """{"line":"AT,$routine"}""") }
    }

    suspend fun estop() = withContext(Dispatchers.IO) { runCatching { post("/api/estop") } }

    suspend fun setToFGuard(enabled: Boolean) = withContext(Dispatchers.IO) {
        runCatching { tof.value = json.decodeFromString<ToFPayload>(post("/api/tof/guard", """{"enabled":$enabled}""")) }
    }

    suspend fun calibrateToF(reference: Float) = withContext(Dispatchers.IO) {
        runCatching { tof.value = json.decodeFromString<ToFPayload>(post("/api/tof/calibrate", """{"reference_m":$reference}""")) }
    }

    suspend fun resetPose() = withContext(Dispatchers.IO) { runCatching { post("/api/pose/reset") } }

    suspend fun trail(): List<Pair<Float, Float>> = withContext(Dispatchers.IO) {
        runCatching {
            json.decodeFromString<PathPayload>(get("/api/path")).path.map { it[0] to it[1] }
        }.getOrDefault(emptyList())
    }

    suspend fun cameraControls(): CamPayload? = withContext(Dispatchers.IO) {
        runCatching { json.decodeFromString<CamPayload>(get("/api/camera/controls")) }.getOrNull()
    }

    suspend fun setCamera(name: String, value: Int) = withContext(Dispatchers.IO) {
        runCatching { post("/api/camera/set", """{"name":"$name","value":$value}""") }
    }

    suspend fun camProfile(name: String) = withContext(Dispatchers.IO) {
        runCatching { post("/api/camera/profile/$name") }
    }

    suspend fun autoCalibrate(target: Int = 110) = withContext(Dispatchers.IO) {
        runCatching { post("/api/camera/autocalibrate?target=$target") }
    }

    suspend fun restartStream(width: Int, bitrate: String) = withContext(Dispatchers.IO) {
        runCatching { post("/api/stream/restart", """{"width":$width,"bitrate":"$bitrate"}""") }
    }

    // ---------- WebSocket ----------
    private var ws: WebSocket? = null
    private var wsJob: Job? = null

    fun start() {
        if (wsJob?.isActive == true) return
        wsJob = scope.launch { connectLoop() }
    }

    fun reconnect() { ws?.cancel(); ws = null }

    private suspend fun connectLoop() {
        while (currentCoroutineContext().isActive) {
            val url = "ws://${_host.value}:8080/ws"
            val latch = CompletableDeferred<Unit>()
            ws = http.newWebSocket(Request.Builder().url(url).build(), object : WebSocketListener() {
                override fun onOpen(webSocket: WebSocket, response: Response) {
                    recorder.record("connection", """{"state":"open","host":${JsonPrimitive(_host.value)}}""")
                    connected.value = true
                    scope.launch { refreshParams(); refreshStatus(); refreshLog() }
                }
                override fun onMessage(webSocket: WebSocket, text: String) = handle(text)
                override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                    recorder.record("connection", """{"state":"failed","reason":${JsonPrimitive(t.message)}}""")
                    connected.value = false
                    tof.value = tof.value.copy(stale = true)
                    Log.w("BotRepo", "ws failed: ${t.message}")
                    latch.complete(Unit)
                }
                override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
                    recorder.record("connection", """{"state":"closed","code":$code,"reason":${JsonPrimitive(reason)}}""")
                    connected.value = false; latch.complete(Unit)
                    tof.value = tof.value.copy(stale = true)
                }
            })
            latch.await()
            delay(1500)
        }
    }

    private fun handle(text: String) {
        recorder.record("ws", text)
        runCatching {
            val o = json.parseToJsonElement(text) as JsonObject
            when (o["type"]?.toString()?.trim('"')) {
                "hello" -> o["status"]?.let {
                    val status = json.decodeFromJsonElement(StatusPayload.serializer(), it)
                    telemetry.value = status.telemetry; debug.value = status.debug
                    tof.value = status.tof; derived.value = status.derived; stream.value = status.stream
                }
                "tof" -> o["tof"]?.let { tof.value = json.decodeFromJsonElement(ToFPayload.serializer(), it) }
                "telemetry" -> {
                    o["tel"]?.let { telemetry.value = json.decodeFromJsonElement(Telemetry.serializer(), it) }
                    o["debug"]?.let {
                        debug.value = json.decodeFromJsonElement(FloatMap, it)
                    }
                }
                "params" -> scope.launch { refreshParams() }
                "log", "param_msg" -> {
                    val t = o["text"]?.jsonPrimitive?.contentOrNull ?: return
                    logLines.value = (logLines.value + t).takeLast(1000)
                    if (t.startsWith("A,")) {
                        autoLog.value = (autoLog.value + t.removePrefix("A,")).takeLast(120)
                    }
                    Unit
                }
                else -> {}
            }
            Unit          // keep the lambda's type stable
        }
    }

    companion object {
        private val repoScope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    }
}
