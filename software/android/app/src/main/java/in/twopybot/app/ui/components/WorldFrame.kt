package `in`.twopybot.app.ui.components

import kotlin.math.*

/** One convention throughout Drive: +X right, +Y up, -Z forward at heading zero. */
object WorldFrame {
    const val AXLE_HEIGHT = .05f

    fun odometry(xForward: Float, yLeft: Float, height: Float = 0f) =
        floatArrayOf(-yLeft,height,-xForward)

    fun offset(pose: WorldPose, right: Float, forward: Float, height: Float): FloatArray {
        val angle = radians(pose.heading)
        return floatArrayOf(pose.x+right*cos(angle)-forward*sin(angle),height+pose.symbolicLift,
            pose.z-right*sin(angle)-forward*cos(angle))
    }

    /** Used by the GLES renderer itself, so tests cover the actual CAD transform. */
    fun bodyMatrix(pose: WorldPose, attitude: Boolean = true): FloatArray {
        val yaw = radians(pose.heading)
        val pitch = radians(if(attitude) -pose.pitch else 0f)
        val roll = radians(if(attitude) pose.roll else 0f)
        val cy=cos(yaw); val sy=sin(yaw); val cp=cos(pitch); val sp=sin(pitch); val cr=cos(roll); val sr=sin(roll)
        val y = floatArrayOf(cy,0f,-sy,0f, 0f,1f,0f,0f, sy,0f,cy,0f, 0f,0f,0f,1f)
        val p = floatArrayOf(1f,0f,0f,0f, 0f,cp,sp,0f, 0f,-sp,cp,0f, 0f,0f,0f,1f)
        val r = floatArrayOf(cr,sr,0f,0f, -sr,cr,0f,0f, 0f,0f,1f,0f, 0f,0f,0f,1f)
        val result = multiply(multiply(y,p),r)
        result[12] = pose.x-result[4]*AXLE_HEIGHT
        result[13] = pose.symbolicLift+AXLE_HEIGHT-result[5]*AXLE_HEIGHT
        result[14] = pose.z-result[6]*AXLE_HEIGHT
        return result
    }

    fun transform(matrix: FloatArray, point: FloatArray) = FloatArray(3) { row ->
        matrix[row]*point[0]+matrix[4+row]*point[1]+matrix[8+row]*point[2]+matrix[12+row]
    }

    private fun multiply(a: FloatArray,b: FloatArray) = FloatArray(16) { index ->
        val row=index%4; val column=index/4
        (0..3).sumOf { (a[it*4+row]*b[column*4+it]).toDouble() }.toFloat()
    }
    private fun radians(degrees: Float) = degrees*PI.toFloat()/180f
}
