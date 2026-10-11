package `in`.twopybot.app.ui.components

import `in`.twopybot.app.data.Telemetry
import kotlin.math.*

enum class SupportEvidence(val label: String, val symbolicLift: Float = 0f) {
    UNKNOWN("Contact unobserved"), SURFACE("Surface · inferred"), ROUGH("Rough surface"),
    HANDLED("Handled / tilted"), AIR("Airborne · IMU", .07f),
    UNLOADED("Possible lift · low g", .035f), LANDING("Touchdown · IMU"),
}

fun supportEvidence(tel: Telemetry, debug: Map<String,Float>, connected: Boolean): SupportEvidence {
    if(!connected || tel.stale) return SupportEvidence.UNKNOWN
    return when(debug["terrainState"]?.toInt()) {
        2 -> SupportEvidence.AIR
        3 -> SupportEvidence.LANDING
        1 -> SupportEvidence.ROUGH
        else -> when {
            (debug["accelMag"] ?: 1f) in 0f..0.44f -> SupportEvidence.UNLOADED
            !tel.balancing && abs(tel.v) < .06f && (abs(tel.pitch) > 25f || abs(debug["w"] ?: 0f) > 65f) -> SupportEvidence.HANDLED
            tel.balancing -> SupportEvidence.SURFACE
            else -> SupportEvidence.UNKNOWN
        }
    }
}

data class WorldPose(
    val x: Float = 0f, val z: Float = 0f, val heading: Float = 0f,
    val pitch: Float = 0f, val roll: Float = 0f,
    val leftWheel: Float = 0f, val rightWheel: Float = 0f,
    val symbolicLift: Float = 0f, val support: SupportEvidence = SupportEvidence.UNKNOWN,
)

/** Encoder pose in metres + live attitude; no invented SLAM or measured lift height. */
class WorldMotion {
    var pose = WorldPose(); private set
    private var initialized = false
    private var firmwareMs = -1L
    private var debugMs: Float? = null
    private var sampleAt = 0L
    private var frameAt = 0L
    private var previousCompass = 0f
    private var orientation = 0f
    private var previousLeft = 0L
    private var previousRight = 0L
    private var wheelReferenced = true
    private var pitchSign = -1f
    private var target = WorldPose()

    fun step(tel: Telemetry, debug: Map<String,Float>, connected: Boolean, nowMs: Long,
             pitchForwardSign: Float = -1f): WorldPose {
        val sign = if(pitchForwardSign > 0f) 1f else -1f
        val valid = connected && !tel.stale && listOf(tel.x,tel.y,tel.pitch,tel.heading,tel.yaw).all { it.isFinite() }
        if(valid && (!initialized || tel.ms != firmwareMs || debug["ms"] != debugMs || sign != pitchSign)) {
            val support = supportEvidence(tel,debug,connected)
            val unloaded = support == SupportEvidence.AIR || support == SupportEvidence.UNLOADED
            val wheelMoved = initialized && (abs(tel.encL-previousLeft)+abs(tel.encR-previousRight) > 2)
            val referenced = !unloaded && (tel.balancing || abs(tel.v) > .025f || wheelMoved)
            if(!initialized || referenced) {
                // Ground movement and CAD must use the SAME odometry bearing. Compass
                // noise/manual idle yaw must not leave a 90-degree offset while driving.
                orientation = tel.heading
            } else {
                val compassTurn = angleDelta(previousCompass,tel.yaw)
                if(abs(compassTurn) > .4f) orientation += compassTurn
            }
            if(initialized && referenced && !wheelReferenced) pose = pose.copy(heading=orientation)
            wheelReferenced = referenced
            previousCompass = tel.yaw
            previousLeft = tel.encL; previousRight = tel.encR; pitchSign = sign
            sampleAt = nowMs; firmwareMs = tel.ms; debugMs = debug["ms"]
            val position = WorldFrame.odometry(tel.x,tel.y)
            target = WorldPose(position[0],position[2],orientation,tel.pitch*sign,
                debug["roll"]?.takeIf { it.isFinite() } ?: 0f,
                -Math.floorMod(tel.encL,4096L)*360f/4096f,
                -Math.floorMod(tel.encR,4096L)*360f/4096f,support.symbolicLift,support)
            if(!initialized || hypot(target.x-pose.x,target.z-pose.z) > 3f) pose = target
            initialized = true
        }
        val dt = ((nowMs-frameAt)/1000f).coerceIn(0f,.1f)
        frameAt = nowMs
        if(!valid || nowMs-sampleAt > 1200) {
            pose = pose.copy(support=SupportEvidence.UNKNOWN)
            return pose
        }
        val blend = 1f-exp(-dt/.10f)
        fun lerp(a: Float,b: Float) = a+(b-a)*blend
        fun angle(a: Float,b: Float) = a+angleDelta(a,b)*blend
        pose = WorldPose(lerp(pose.x,target.x),lerp(pose.z,target.z),angle(pose.heading,target.heading),
            angle(pose.pitch,target.pitch),angle(pose.roll,target.roll),angle(pose.leftWheel,target.leftWheel),
            angle(pose.rightWheel,target.rightWheel),lerp(pose.symbolicLift,target.symbolicLift),target.support)
        return pose
    }

    companion object {
        fun angleDelta(from: Float,to: Float) = ((to-from+180f)%360f+360f)%360f-180f
        fun worldPoint(pose: WorldPose, right: Float, forward: Float, height: Float): FloatArray {
            return WorldFrame.offset(pose,right,forward,height)
        }
    }
}
