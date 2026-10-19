package `in`.twopybot.app.data

import kotlinx.serialization.json.Json
import org.junit.Assert.*
import org.junit.Test

class FirmwareModelsTest {
    private val json=Json { ignoreUnknownKeys=true }
    @Test fun decodesPublishedBuildAndRunningJob() {
        val id="a".repeat(64)
        val value=json.decodeFromString<FirmwareStatus>("""{"latest":{"build_id":"$id","version":"3.12.0","sha256":"$id","size_bytes":850000,"built_at":"2026-10-18T10:00:00Z","source_dirty":true},"busy":true,"worker_ready":true,"can_flash":false,"job":{"id":"job","status":"running","stage":"writing","progress":52,"message":"Writing","write_started":true}}""")
        assertEquals("3.12.0",value.latest?.version)
        assertEquals(52f,value.job?.progress)
        assertFalse(value.canRequestFlash())
    }
    @Test fun allowsOnlyReadyPinnedBuildAndRetainsRollbackDetails() {
        val build=FirmwareBuild(buildId="b".repeat(64),version="3.12.0")
        val ready=FirmwareStatus(latest=build,workerReady=true,canFlash=true)
        assertTrue(ready.canRequestFlash())
        assertFalse(ready.copy(busy=true).canRequestFlash())
        assertFalse(ready.copy(latest=null).canRequestFlash())
        assertFalse(ready.copy(latest=build.copy(buildId="../bad")).canRequestFlash())
        val failed=json.decodeFromString<FirmwareStatus>("""{"job":{"status":"failed","error":"verification failed","rolled_back":true},"busy":false,"can_flash":false}""")
        assertTrue(failed.job?.rolledBack==true)
        assertFalse(failed.canRequestFlash())
    }
}
