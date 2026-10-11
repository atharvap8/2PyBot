package `in`.twopybot.app.ui

import org.junit.Assert.assertEquals
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
}
