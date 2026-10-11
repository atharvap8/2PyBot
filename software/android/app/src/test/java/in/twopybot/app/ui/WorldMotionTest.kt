package `in`.twopybot.app.ui

import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.ui.components.*
import org.junit.Assert.*
import org.junit.Test

class WorldMotionTest {
    @Test fun livePoseUsesMetresAndTheSameAxesAsEncoderTrail() {
        val motion=WorldMotion()
        val pose=motion.step(Telemetry(ms=100,x=2f,y=3f,heading=90f,pitch=7f,stale=false),emptyMap(),true,1000)
        assertEquals(-3f,pose.x,0f); assertEquals(-2f,pose.z,0f); assertEquals(-7f,pose.pitch,0f)
        val point=WorldMotion.worldPoint(pose,0f,1f,.2f)
        assertEquals(-4f,point[0],.001f); assertEquals(-2f,point[2],.001f)
    }
    @Test fun wraparoundTakesTheShortTurnRatherThanSpinningFullCircle() {
        assertEquals(2f,WorldMotion.angleDelta(359f,1f),.001f)
        assertEquals(-2f,WorldMotion.angleDelta(1f,359f),.001f)
    }
    @Test fun staleTelemetryDoesNotAnimateInventedMovement() {
        val motion=WorldMotion()
        val tel=Telemetry(ms=100,x=2f,stale=false)
        val pose=motion.step(tel,emptyMap(),true,1000)
        val stale=motion.step(tel.copy(ms=200,x=10f,stale=true),emptyMap(),true,1100)
        assertEquals(pose.z,stale.z,0f)
        assertEquals(SupportEvidence.UNKNOWN,stale.support)
    }
    @Test fun handHeldRotationCanUseCompassWhenEncodersCannotSeeIt() {
        val motion=WorldMotion()
        val tel=Telemetry(ms=100,yaw=350f,stale=false)
        motion.step(tel,emptyMap(),true,1000)
        val pose=motion.step(tel.copy(ms=200,yaw=10f),emptyMap(),true,1100)
        assertTrue(pose.heading > 0f && pose.heading < 21f)
    }
    @Test fun airborneEventIsAnIndicativeLiftNotMeasuredHeight() {
        val tel=Telemetry(ms=100,stale=false)
        assertEquals(SupportEvidence.UNKNOWN,supportEvidence(tel,mapOf("accelMag" to 1f),true))
        val pose=WorldMotion().step(tel,mapOf("terrainState" to 2f),true,1000)
        assertEquals(SupportEvidence.AIR,pose.support)
        assertTrue(pose.symbolicLift > 0f)
        assertEquals(SupportEvidence.LANDING,supportEvidence(tel,mapOf("terrainState" to 3f),true))
    }
    @Test fun airborneRotationDoesNotTreatSpinningWheelsAsBodyYaw() {
        val motion=WorldMotion()
        val tel=Telemetry(ms=100,yaw=0f,balancing=true,stale=false)
        motion.step(tel,emptyMap(),true,1000)
        val pose=motion.step(tel.copy(ms=200,heading=90f,yaw=10f),mapOf("terrainState" to 2f),true,1100)
        assertTrue(pose.heading > 0f && pose.heading < 11f)
    }
    @Test fun compassIdleTurnCannotLeaveCadSidewaysWhenForwardMotionStarts() {
        val motion=WorldMotion()
        val tel=Telemetry(ms=100,stale=false)
        motion.step(tel,emptyMap(),true,1000)
        motion.step(tel.copy(ms=200,yaw=90f),emptyMap(),true,1100)
        val pose=motion.step(tel.copy(ms=300,x=.10f,encL=100,encR=100,v=.20f,yaw=90f),emptyMap(),true,1200)
        assertEquals(0f,pose.heading,.001f)
        assertTrue(pose.z < 0f)
        assertEquals(0f,pose.x,.001f)
    }
}
