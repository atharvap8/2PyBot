package `in`.twopybot.app.data

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.serialization.json.*
import java.io.BufferedWriter
import java.io.File
import java.io.OutputStream
import java.util.UUID
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sqrt

data class RecordingState(
    val active: Boolean = false, val id: String = "", val terrain: String = "unlabelled",
    val rows: Long = 0, val samples: Long = 0, val elapsedMs: Long = 0,
    val pitchRms: Double = 0.0, val peakSpeed: Double = 0.0, val distanceM: Double = 0.0,
    val error: String = "",
)

/** Raw, ordered on-disk capture. The repository calls this before decoding any data. */
class SessionRecorder(private val directory: File) {
    val state = MutableStateFlow(RecordingState())
    val sessions = MutableStateFlow<List<String>>(emptyList())
    private val json = Json { ignoreUnknownKeys = true }
    private var events: BufferedWriter? = null
    private var telemetry: BufferedWriter? = null
    private var folder: File? = null
    private var startNanos = 0L
    private var startWall = 0L
    private var rows = 0L
    private var samples = 0L
    private var sumPitchSquared = 0.0
    private var peakPitch = 0.0
    private var peakSpeed = 0.0
    private var distance = 0.0
    private var staleSamples = 0L
    private var saturatedSamples = 0L
    private var balancedSamples = 0L
    private var lastPose: Pair<Double, Double>? = null
    private var terrain = "unlabelled"
    private val counts = linkedMapOf<String, Long>()

    init { refreshSessions() }

    @Synchronized fun refreshSessions() {
        sessions.value = directory.listFiles()?.filter { it.isDirectory }
            ?.map { it.name }?.sortedDescending().orEmpty()
    }

    @Synchronized fun start(label: String, metadata: JsonObject): String {
        check(events == null) { "A recording is already active" }
        check(directory.isDirectory || directory.mkdirs()) { "Cannot create recording directory" }
        val id = "${System.currentTimeMillis()}-${UUID.randomUUID().toString().take(8)}"
        val target = File(directory, id)
        check(target.mkdir()) { "Cannot create session" }
        folder = target
        startNanos = System.nanoTime(); startWall = System.currentTimeMillis()
        rows = 0; samples = 0; sumPitchSquared = 0.0; peakPitch = 0.0; peakSpeed = 0.0
        distance = 0.0; staleSamples = 0; saturatedSamples = 0; balancedSamples = 0
        counts.clear(); lastPose = null; terrain = label.trim().ifEmpty { "unlabelled" }
        File(target, "metadata.json").writeText(buildJsonObject {
            put("schema", "2pybot-session/1"); put("session_id", id); put("started_wall_ms", startWall)
            put("initial_terrain", terrain); put("robot", metadata)
            put("timestamps", "UTC epoch milliseconds and monotonic nanoseconds at Android reception")
            put("capture", "All received WebSocket JSON and HTTP responses/requests; no encoded camera video")
            put("labels", "Human terrain annotations; reward and terminal are intentionally unset")
        }.toString())
        try {
            events = File(target, "events.jsonl").bufferedWriter()
            telemetry = File(target, "telemetry.csv").bufferedWriter().also {
                it.write("seq,elapsed_ms,wall_ms,terrain,firmware_ms,x_m,y_m,heading_deg,pitch_deg,speed_mps,armed,balancing,stale,fwd,steer,v_cmd_steps_s,sat_v,sat_a,loop_max_us,debug_json,reward,terminal\n")
            }
            state.value = RecordingState(active = true, id = id, terrain = terrain)
            record("session_start", metadata.toString())
            check(state.value.active) { state.value.error }
        } catch (e: Exception) {
            events?.close(); telemetry?.close(); events = null; telemetry = null
            state.value = RecordingState(id = id, error = e.message ?: "Recording failed")
            throw e
        }
        refreshSessions()
        return id
    }

    @Synchronized fun label(value: String, note: String = "") {
        terrain = value.trim().ifEmpty { "unlabelled" }
        record("annotation", buildJsonObject { put("terrain", terrain); put("note", note) }.toString())
    }

    @Synchronized fun record(kind: String, raw: String) {
        val writer = events ?: return
        try {
            val now = System.nanoTime()
            val elapsed = (now - startNanos) / 1_000_000
            val wall = System.currentTimeMillis()
            val payload = runCatching { json.parseToJsonElement(raw) }.getOrElse { JsonPrimitive(raw) }
            val type = if (kind == "ws") (payload as? JsonObject)?.get("type")?.jsonPrimitive?.contentOrNull ?: "unknown" else kind
            if (kind == "http_request" && (payload as? JsonObject)?.get("path")?.jsonPrimitive?.contentOrNull == "/api/pose/reset") lastPose = null
            counts[type] = (counts[type] ?: 0) + 1
            writer.write(buildJsonObject {
                put("seq", ++rows); put("elapsed_ms", elapsed); put("wall_ms", wall)
                put("mono_ns", now); put("terrain", terrain); put("source", kind)
                put("event", type); put("payload", payload)
            }.toString()); writer.newLine()
            if (kind == "ws" && type == "telemetry" && payload is JsonObject) {
                val tel = payload["tel"] as? JsonObject ?: JsonObject(emptyMap())
                val debug = payload["debug"] as? JsonObject ?: JsonObject(emptyMap())
                fun value(o: JsonObject, key: String) = (o[key] as? JsonPrimitive)?.contentOrNull.orEmpty()
                val pitch = value(tel, "pitch").toDoubleOrNull()?.takeIf { it.isFinite() } ?: 0.0
                val speed = value(tel, "v_ms").toDoubleOrNull()?.takeIf { it.isFinite() } ?: 0.0
                val stale = value(tel, "stale") != "false"
                ++samples
                if (stale) { ++staleSamples; lastPose = null } else {
                    sumPitchSquared += pitch * pitch; peakPitch = maxOf(peakPitch, abs(pitch))
                    peakSpeed = maxOf(peakSpeed, abs(speed))
                    val x = value(tel, "x_m").toDoubleOrNull(); val y = value(tel, "y_m").toDoubleOrNull()
                    if (x != null && y != null && x.isFinite() && y.isFinite()) {
                        lastPose?.let { distance += hypot(x - it.first, y - it.second) }
                        lastPose = x to y
                    }
                }
                if (value(tel, "balancing") == "true") ++balancedSamples
                if ((value(debug, "satV").toDoubleOrNull() ?: 0.0) > .5 ||
                    (value(debug, "satA").toDoubleOrNull() ?: 0.0) > .5) ++saturatedSamples
                val fields = listOf(rows.toString(), elapsed.toString(), wall.toString(), terrain) +
                    listOf("ms", "x_m", "y_m", "heading", "pitch", "v_ms", "armed", "balancing", "stale").map { value(tel, it) } +
                    listOf("fwd", "str", "vCmd", "satV", "satA", "loopMax").map { value(debug, it) } +
                    listOf(debug.toString(), "", "")
                telemetry?.write(fields.joinToString(",") { csv(it) } + "\n")
            }
            // At the console's 10 Hz rates this keeps interrupted sessions recoverable.
            writer.flush(); telemetry?.flush()
            state.value = state.value.copy(terrain = terrain, rows = rows, samples = samples,
                elapsedMs = elapsed, pitchRms = sqrt(sumPitchSquared / maxOf(1, samples - staleSamples)),
                peakSpeed = peakSpeed, distanceM = distance)
        } catch (e: Exception) {
            runCatching { events?.close() }; runCatching { telemetry?.close() }
            events = null; telemetry = null
            state.value = state.value.copy(active = false, error = e.message ?: "Recording I/O failed")
        }
    }

    @Synchronized fun stop() {
        if (events == null) return
        record("session_stop", "{}")
        val snapshot = state.value
        check(snapshot.error.isEmpty()) { snapshot.error }
        try {
            events?.close(); telemetry?.close()
            File(folder!!, "diagnostics.json").writeText(buildJsonObject {
                put("complete", true); put("events", rows); put("telemetry_samples", samples)
                put("elapsed_ms", snapshot.elapsedMs); put("stale_samples", staleSamples)
                put("pitch_rms_deg", snapshot.pitchRms); put("peak_abs_pitch_deg", peakPitch)
                put("peak_abs_speed_mps", peakSpeed); put("encoder_distance_m", distance)
                put("balanced_samples", balancedSamples); put("saturated_samples", saturatedSamples)
                put("event_counts", buildJsonObject { counts.forEach { (k, v) -> put(k, v) } })
                put("interpretation", "Reception-level 10 Hz diagnostics; encoder drift and transport gaps remain in raw events")
            }.toString())
            state.value = snapshot.copy(active = false)
        } finally { events = null; telemetry = null; refreshSessions() }
    }

    fun export(id: String, output: OutputStream) {
        require(id in sessions.value && id != state.value.takeIf { it.active }?.id) { "Stop the session before exporting" }
        val target = File(directory, id)
        require(target.parentFile?.canonicalFile == directory.canonicalFile)
        ZipOutputStream(output).use { zip ->
            target.listFiles()?.filter { it.isFile }?.sortedBy { it.name }?.forEach { file ->
                zip.putNextEntry(ZipEntry(file.name)); file.inputStream().use { it.copyTo(zip) }; zip.closeEntry()
            }
            zip.putNextEntry(ZipEntry("README.txt"))
            zip.write(("2PyBot labelled session\n" +
                "events.jsonl: ordered, lossless received JSON, timestamps, terrain and commands.\n" +
                "telemetry.csv: aligned state/action samples; debug_json preserves all debug fields.\n" +
                "metadata.json: host, app version and initial live parameters/derived values.\n" +
                "diagnostics.json exists only for a normally stopped capture.\n" +
                "Terrain annotations are user labels. Missing rewards/terminals are not inferred.\n" +
                "Camera video is not captured; sessions are suitable for offline feature/reward design.\n").toByteArray())
            zip.closeEntry()
        }
    }

    companion object { private fun csv(value: String) = "\"${value.replace("\"", "\"\"")}\"" }
}
