package `in`.twopybot.app.ui.components

import kotlin.math.*

/** Display references only: these are not detected roads, lanes or free-space claims. */
object WorldReferenceGeometry {
    fun ribbon(halfWidth: Float = .18f, length: Float = 2.8f): FloatArray {
        val vertices=mutableListOf<Float>()
        for(i in 0 until 24) {
            val a=i/24f; val b=(i+1)/24f
            val widthA=halfWidth+a*.035f; val widthB=halfWidth+b*.035f
            for(p in listOf(floatArrayOf(-widthA,a,-1f),floatArrayOf(widthA,a,1f),floatArrayOf(widthB,b,1f),
                floatArrayOf(-widthA,a,-1f),floatArrayOf(widthB,b,1f),floatArrayOf(-widthB,b,-1f))) {
                vertices.addAll(listOf(p[0],.006f,-(.16f+p[1]*length),p[2],p[1],0f))
            }
        }
        return vertices.toFloatArray()
    }

    fun shoulder(side: Float): FloatArray = (0..32).flatMap { i ->
        val f=i/32f
        listOf(side*(.48f+f*.10f),.004f,.5f-f*4.8f,0f,f,0f)
    }.toFloatArray()

    fun rangeArc(radius: Float): FloatArray = (0..48).flatMap { i ->
        val angle=(-28f+i*56f/48)*PI.toFloat()/180
        listOf(sin(angle)*radius,.007f,-cos(angle)*radius,0f,1f,0f)
    }.toFloatArray()

    fun forwardArrow(): FloatArray = listOf(
        -.05f to -.29f,.05f to -.29f,0f to -.43f,
        -.018f to -.16f,.018f to -.16f,.018f to -.31f,
        -.018f to -.16f,.018f to -.31f,-.018f to -.31f,
    ).flatMap { (x,z) -> listOf(x,.008f,z,0f,1f,0f) }.toFloatArray()
}
