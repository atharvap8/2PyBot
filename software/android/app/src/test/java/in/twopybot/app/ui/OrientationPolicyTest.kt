package `in`.twopybot.app.ui

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class OrientationPolicyTest {
    @Test fun reversePortraitNeverBecomesATarget() {
        assertEquals(AllowedOrientation.PORTRAIT, OrientationPolicy.target(180, AllowedOrientation.PORTRAIT))
        assertEquals(AllowedOrientation.LANDSCAPE, OrientationPolicy.target(180, AllowedOrientation.LANDSCAPE))
    }
    @Test fun onlySupportedStableWindowsRotate() {
        assertEquals(AllowedOrientation.PORTRAIT, OrientationPolicy.target(0, AllowedOrientation.LANDSCAPE))
        assertEquals(AllowedOrientation.REVERSE_LANDSCAPE, OrientationPolicy.target(90, AllowedOrientation.PORTRAIT))
        assertEquals(AllowedOrientation.LANDSCAPE, OrientationPolicy.target(270, AllowedOrientation.PORTRAIT))
        assertEquals(AllowedOrientation.PORTRAIT, OrientationPolicy.target(-1, AllowedOrientation.PORTRAIT))
        assertEquals(AllowedOrientation.LANDSCAPE, OrientationPolicy.target(220, AllowedOrientation.LANDSCAPE))
    }
    @Test fun poseMustRemainHeldForTwoSeconds() {
        val held = HeldOrientationPolicy()
        held.observe(90, AllowedOrientation.PORTRAIT, 100)
        assertNull(held.ready(2099))
        held.observe(95, AllowedOrientation.PORTRAIT, 1200)
        assertEquals(AllowedOrientation.REVERSE_LANDSCAPE, held.ready(2100))
    }
    @Test fun leavingWindowRestartsTheHold() {
        val held = HeldOrientationPolicy()
        held.observe(270, AllowedOrientation.PORTRAIT, 0)
        held.observe(180, AllowedOrientation.PORTRAIT, 1500)
        assertNull(held.ready(2000))
        held.observe(270, AllowedOrientation.PORTRAIT, 2500)
        assertNull(held.ready(4499))
        assertEquals(AllowedOrientation.LANDSCAPE, held.ready(4500))
    }
    @Test fun returningToCurrentPoseCancelsPendingRotation() {
        val held = HeldOrientationPolicy()
        held.observe(90, AllowedOrientation.PORTRAIT, 0)
        held.observe(0, AllowedOrientation.PORTRAIT, 1900)
        assertNull(held.ready(5000))
    }
}
