package `in`.twopybot.app.ui

import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.ui.components.*
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.*

class WorldFrameTest {
    @Test fun actualRenderedCadNoseAndForwardTravelAgreeAtEveryCardinalHeading() {
        for(heading in listOf(0f,90f,180f,270f)) {
            val angle=heading*PI.toFloat()/180
            val travel=WorldFrame.odometry(cos(angle),sin(angle))
            val matrix=WorldFrame.bodyMatrix(WorldPose(heading=heading))
            val axle=WorldFrame.transform(matrix,floatArrayOf(0f,.05f,0f))
            val nose=WorldFrame.transform(matrix,floatArrayOf(0f,.05f,-1f))
            assertEquals(travel[0],nose[0]-axle[0],.0001f)
            assertEquals(travel[2],nose[2]-axle[2],.0001f)
        }
    }
    @Test fun negativeRawImuPitchWithSavedMinusSignTiltsCadForwardNotBackward() {
        val tel=Telemetry(ms=100,pitch=-15f,stale=false)
        val pose=WorldMotion().step(tel,emptyMap(),true,1000,-1f)
        val matrix=WorldFrame.bodyMatrix(pose)
        val axle=WorldFrame.transform(matrix,floatArrayOf(0f,.05f,0f))
        val top=WorldFrame.transform(matrix,floatArrayOf(0f,.25f,0f))
        assertTrue(top[2] < axle[2])
        val back=WorldMotion().step(tel.copy(pitch=15f),emptyMap(),true,1000,-1f)
        assertTrue(WorldFrame.transform(WorldFrame.bodyMatrix(back),floatArrayOf(0f,.25f,0f))[2] > 0f)
    }
    @Test fun localDepthAndGuidanceUseTheSameRenderedBodyBearing() {
        val pose=WorldPose(x=2f,z=-3f,heading=123f)
        val matrix=WorldFrame.bodyMatrix(pose,false)
        val cadPoint=WorldFrame.transform(matrix,floatArrayOf(.1f,.2f,-.8f))
        val sensedPoint=WorldFrame.offset(pose,.1f,.8f,.2f)
        for(axis in 0..2) assertEquals(cadPoint[axis],sensedPoint[axis],.0001f)
    }
}
