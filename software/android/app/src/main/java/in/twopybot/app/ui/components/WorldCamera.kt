package `in`.twopybot.app.ui.components

import kotlin.math.*

/** Camera navigation is deliberately separate from robot pose. Touch never moves the bot. */
data class WorldCamera(
    val azimuth: Float = 90f, val elevation: Float = 22f, val distance: Float = 1.12f,
    val panX: Float = 0f, val panZ: Float = 0f,
    val lookAhead: Float = .45f,
    val focusY: Float = .10f,
) {
    fun orbit(dx: Float, dy: Float) = copy(
        azimuth = (azimuth - dx * .3f) % 360f,
        elevation = (elevation + dy * .25f).coerceIn(10f, 80f),
    )

    fun zoom(scale: Float) = if (scale.isFinite() && scale > 0f)
        copy(distance = (distance / scale).coerceIn(.35f, 12f)) else this

    fun pan(dx: Float, dy: Float, viewportHeight: Int): WorldCamera {
        val metres = distance * 1.2f / viewportHeight.coerceAtLeast(1)
        val angle = azimuth * PI.toFloat() / 180
        return copy(panX = panX + (-sin(angle) * dx + cos(angle) * dy) * metres,
            panZ = panZ + (cos(angle) * dx + sin(angle) * dy) * metres)
    }

    fun view(pose: WorldPose, follow: Boolean): WorldCameraView {
        val az=(azimuth-if(follow) pose.heading else 0f)*PI.toFloat()/180
        val el=elevation*PI.toFloat()/180
        val x=panX+if(follow) pose.x else 0f
        val z=panZ+if(follow) pose.z else 0f
        val y=focusY+if(follow) pose.symbolicLift else 0f
        val forward=WorldFrame.offset(pose,0f,lookAhead,0f)
        val target=floatArrayOf(x+if(follow) forward[0]-pose.x else 0f,y,
            z+if(follow) forward[2]-pose.z else 0f)
        val eye=floatArrayOf(x+cos(az)*cos(el)*distance,y+sin(el)*distance,
            z+sin(az)*cos(el)*distance)
        return WorldCameraView(eye,target)
    }

    fun exploreAt(pose: WorldPose): WorldCamera {
        val current=view(pose,true)
        return fromVector(current.eye,current.target,current.target[0],current.target[2],0f,current.target[1])
    }

    fun followAt(pose: WorldPose): WorldCamera {
        val current=view(pose,false)
        val ahead=WorldFrame.offset(pose,0f,lookAhead,0f)
        val base=floatArrayOf(current.target[0]-ahead[0]+pose.x,current.target[1],
            current.target[2]-ahead[2]+pose.z)
        return fromVector(current.eye,base,base[0]-pose.x,base[2]-pose.z,pose.heading,base[1])
            .copy(focusY=base[1]-pose.symbolicLift)
    }

    private fun fromVector(eye: FloatArray, base: FloatArray, x: Float,z: Float, heading: Float,y: Float): WorldCamera {
        val dx=eye[0]-base[0]; val dy=eye[1]-y; val dz=eye[2]-base[2]
        val horizontal=hypot(dx,dz)
        return copy(azimuth=atan2(dz,dx)*180f/PI.toFloat()+heading,
            elevation=atan2(dy,horizontal)*180f/PI.toFloat(),distance=hypot(horizontal,dy),panX=x,panZ=z,focusY=y)
    }
}

data class WorldCameraView(val eye: FloatArray,val target: FloatArray)
