package `in`.twopybot.app.data

import kotlinx.serialization.json.*
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.ByteArrayOutputStream
import java.util.zip.ZipInputStream

class SessionRecorderTest {
    @get:Rule val temporary = TemporaryFolder()

    @Test fun rawEventsLabelsAndDiagnosticsSurviveExportAndReload() {
        val directory = temporary.newFolder()
        val recorder = SessionRecorder(directory)
        val id = recorder.start("flat,tiles", buildJsonObject { put("version", "test") })
        recorder.record("ws", """{"type":"telemetry","tel":{"pitch":3,"v_ms":0.4,"x_m":0,"y_m":0,"stale":false,"balancing":true},"debug":{"satV":1,"custom":7}}""")
        recorder.label("gravel", "slip \"left\"")
        recorder.record("ws", """{"type":"tof","tof":{"distance_mm":[600],"status":[5]}}""")
        recorder.record("ws", """{"type":"telemetry","tel":{"pitch":-4,"v_ms":0.2,"x_m":0.3,"y_m":0.4,"stale":false},"debug":{}}""")
        recorder.stop()
        assertFalse(recorder.state.value.active)
        assertEquals(2, recorder.state.value.samples.toInt())
        assertEquals(.5, recorder.state.value.distanceM, .0001)
        assertEquals(kotlin.math.sqrt(12.5), recorder.state.value.pitchRms, .0001)
        val output = ByteArrayOutputStream()
        SessionRecorder(directory).export(id, output)
        val files = mutableMapOf<String, String>()
        ZipInputStream(output.toByteArray().inputStream()).use { zip ->
            var entry = zip.nextEntry
            while (entry != null) { files[entry.name] = zip.readBytes().toString(Charsets.UTF_8); entry = zip.nextEntry }
        }
        val rows = files.getValue("events.jsonl").lines().filter { it.isNotEmpty() }.map { Json.parseToJsonElement(it).jsonObject }
        assertEquals("gravel", rows.first { it["event"]?.jsonPrimitive?.content == "tof" }["terrain"]?.jsonPrimitive?.content)
        assertTrue(files.getValue("telemetry.csv").contains("\"flat,tiles\""))
        assertTrue(files.getValue("telemetry.csv").contains("custom"))
        assertEquals("1", Json.parseToJsonElement(files.getValue("diagnostics.json")).jsonObject["saturated_samples"]?.jsonPrimitive?.content)
        assertEquals(1L, rows.first()["seq"]?.jsonPrimitive?.long)
    }

    @Test fun activeAndUnknownSessionsCannotBeExported() {
        val recorder = SessionRecorder(temporary.newFolder())
        val id = recorder.start("flat", buildJsonObject {})
        assertThrows(IllegalArgumentException::class.java) { recorder.export(id, ByteArrayOutputStream()) }
        assertThrows(IllegalArgumentException::class.java) { recorder.export("../other", ByteArrayOutputStream()) }
        recorder.stop()
    }
}
