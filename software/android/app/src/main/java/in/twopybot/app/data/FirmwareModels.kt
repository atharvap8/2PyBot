package `in`.twopybot.app.data

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class FirmwareBuild(
    @SerialName("build_id") val buildId: String = "",
    val version: String = "", val sha256: String = "",
    @SerialName("size_bytes") val sizeBytes: Long = 0,
    @SerialName("built_at") val builtAt: String = "",
    @SerialName("source_commit") val sourceCommit: String = "",
    @SerialName("source_dirty") val sourceDirty: Boolean = false,
)

@Serializable
data class FirmwareJob(
    val id: String = "", val version: String = "", val status: String = "",
    val stage: String = "", val progress: Float = 0f,
    val message: String = "", val error: String = "",
    @SerialName("rolled_back") val rolledBack: Boolean = false,
    @SerialName("params_preserved") val paramsPreserved: Int = 0,
)

@Serializable
data class FirmwareStatus(
    val latest: FirmwareBuild? = null, val installed: FirmwareBuild? = null,
    val job: FirmwareJob? = null, val busy: Boolean = false,
    @SerialName("worker_ready") val workerReady: Boolean = false,
    @SerialName("can_flash") val canFlash: Boolean = false,
    @SerialName("blocked_reason") val blockedReason: String = "Loading firmware status",
) {
    fun canRequestFlash(): Boolean = canFlash && !busy && workerReady &&
        latest?.buildId?.matches(Regex("[a-f0-9]{64}")) == true
}
