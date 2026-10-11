package `in`.twopybot.app.ui

import `in`.twopybot.app.ui.components.WorldCamera
import org.junit.Assert.*
import org.junit.Test

class WorldCameraTest {
    @Test fun orbitAndZoomRemainNavigableAtExtremeGestures() {
        val camera=WorldCamera().orbit(6000f,6000f).zoom(.00001f)
        assertEquals(80f,camera.elevation,.001f)
        assertEquals(12f,camera.distance,.001f)
        assertEquals(.35f,camera.zoom(10000f).distance,.001f)
        assertEquals(camera,camera.zoom(Float.NaN))
        assertEquals(camera,camera.zoom(0f))
    }
    @Test fun panMovesTheCameraWithoutChangingOrbitOrDistance() {
        val camera=WorldCamera()
        val moved=camera.pan(100f,50f,400)
        assertNotEquals(camera.panX,moved.panX)
        assertNotEquals(camera.panZ,moved.panZ)
        assertEquals(camera.azimuth,moved.azimuth,0f)
        assertEquals(camera.distance,moved.distance,0f)
    }
    @Test fun rearChaseCameraIsBehindTheCadAndLooksAlongForwardTravel() {
        for(heading in listOf(0f,90f,180f,270f)) {
            val pose=`in`.twopybot.app.ui.components.WorldPose(x=2f,z=-3f,heading=heading)
            val view=WorldCamera().view(pose,true)
            val nose=`in`.twopybot.app.ui.components.WorldFrame.offset(pose,0f,1f,0f)
            val dx=nose[0]-pose.x; val dz=nose[2]-pose.z
            assertTrue((view.eye[0]-pose.x)*dx+(view.eye[2]-pose.z)*dz < 0f)
            assertTrue((view.target[0]-pose.x)*dx+(view.target[2]-pose.z)*dz > 0f)
        }
    }
    @Test fun switchingBetweenFollowAndExploreDoesNotJumpTheView() {
        val pose=`in`.twopybot.app.ui.components.WorldPose(x=4f,z=-5f,heading=67f,symbolicLift=.07f)
        val camera=WorldCamera().orbit(35f,12f)
        val before=camera.view(pose,true)
        val explore=camera.exploreAt(pose)
        val after=explore.view(pose,false)
        val restored=explore.followAt(pose).view(pose,true)
        for(i in 0..2) {
            assertEquals(before.eye[i],after.eye[i],.0001f)
            assertEquals(before.target[i],after.target[i],.0001f)
            assertEquals(before.eye[i],restored.eye[i],.0001f)
            assertEquals(before.target[i],restored.target[i],.0001f)
        }
    }
}
