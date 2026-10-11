package `in`.twopybot.app.ui.components

import android.graphics.Paint
import android.graphics.Path
import androidx.compose.foundation.Canvas
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.data.ToFPayload
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlin.math.*

@Serializable private data class Mesh(val parts: List<MeshPart>)
@Serializable private data class MeshPart(val name: String, val color: String, val vertices: List<Float>, val faces: List<Int>)
private data class Triangle(val xyz: List<FloatArray>, val color: Int)

@Composable
fun RobotScene(tel: Telemetry, tof: ToFPayload, modifier: Modifier = Modifier) {
    val context = LocalContext.current
    var failure by remember { mutableStateOf("") }
    val triangles by produceState<List<Triangle>>(emptyList()) {
        value = withContext(Dispatchers.IO) {
            runCatching {
                val raw = context.assets.open("robot-model.json").bufferedReader().use { it.readText() }
                Json { ignoreUnknownKeys = true }.decodeFromString<Mesh>(raw).parts.flatMap { part ->
                    val base = android.graphics.Color.parseColor(part.color)
                    part.faces.chunked(3).map { indices ->
                        val xyz = indices.map { i -> FloatArray(3) { part.vertices[i * 3 + it] } }
                        val u = FloatArray(3) { xyz[1][it] - xyz[0][it] }
                        val w = FloatArray(3) { xyz[2][it] - xyz[0][it] }
                        val n = floatArrayOf(u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0])
                        val length = sqrt(n.sumOf { (it*it).toDouble() }).toFloat().coerceAtLeast(.0000001f)
                        val light = (.55f + .45f * abs((n[0]*.3f-n[1]*.4f+n[2]*.85f)/length)).coerceIn(.4f,1f)
                        Triangle(xyz,android.graphics.Color.rgb((android.graphics.Color.red(base)*light).toInt(),
                            (android.graphics.Color.green(base)*light).toInt(),(android.graphics.Color.blue(base)*light).toInt()))
                    }
                }
            }.getOrElse { failure = "Model unavailable: ${it.message}"; emptyList() }
        }
    }
    Canvas(modifier.semantics { contentDescription = "Assembled 2PyBot CAD model with measured depth returns" }) {
        val canvas = drawContext.canvas.nativeCanvas
        val paint = Paint(Paint.ANTI_ALIAS_FLAG)
        val path = Path()
        val scale = min(size.width,size.height)*2.35f
        fun project(x: Float,y: Float,z: Float): FloatArray {
            val perspective = 1f / max(.65f,1+y*.72f)
            return floatArrayOf(size.width*.5f+(x-y*.25f)*scale*perspective,
                size.height*.85f-(z+y*.27f)*scale*perspective)
        }
        fun line(a: FloatArray,b: FloatArray,color: Int,width: Float = 1f) {
            val p=project(a[0],a[1],a[2]); val q=project(b[0],b[1],b[2])
            paint.color=color; paint.strokeWidth=width*density
            canvas.drawLine(p[0],p[1],q[0],q[1],paint)
        }
        canvas.drawColor(0xffdfe2e5.toInt())
        for (i in 0..12) line(floatArrayOf(-.9f,i*.25f,0f),floatArrayOf(.9f,i*.25f,0f),0xffc9cdd1.toInt())
        for (i in -6..6) line(floatArrayOf(i*.15f,0f,0f),floatArrayOf(i*.15f,3f,0f),0xffc9cdd1.toInt())
        for (sign in listOf(-1,1)) {
            var previous=floatArrayOf(sign*tof.halfWidth,0f,.002f)
            for (i in 1..(min(2f,tof.stoppingDistance)/.05f).toInt()) {
                val y=i*.05f
                val point=floatArrayOf(sign*(tof.halfWidth+min(.3f,.5f*tof.curvature*y*y)),y,.002f)
                line(previous,point,if(tof.guardEnabled) 0xff778c9b.toInt() else 0xffadb5bc.toInt(),2f)
                previous=point
            }
        }
        if (!tof.stale) for (point in tof.points.sortedByDescending { it.y }) {
            line(floatArrayOf(point.x,point.y,0f),floatArrayOf(point.x,point.y,point.z),0xffb9a275.toInt())
            val p=project(point.x,point.y,point.z)
            paint.color=if(point.confirmed) 0xff89775a.toInt() else 0xffbca989.toInt()
            paint.alpha=(255*point.confidence.coerceIn(.3f,1f)).toInt()
            canvas.drawCircle(p[0],p[1],max(2f,5/(1+point.y*.7f))*density,paint); paint.alpha=255
        }
        val angle=tel.pitch*PI.toFloat()/180; val s=sin(angle); val c=cos(angle)
        val rotated=triangles.map { face ->
            val xyz=face.xyz.map { v -> floatArrayOf(v[0],v[1]*c+(v[2]-.05f)*s,.05f+(v[2]-.05f)*c-v[1]*s) }
            Triple(xyz,face.color,xyz.sumOf { (it[1]-.25f*it[2]+.1f*it[0]).toDouble() })
        }.sortedByDescending { it.third }
        for ((xyz,color) in rotated) {
            path.reset()
            xyz.forEachIndexed { i,v -> val p=project(v[0],v[1],v[2]); if(i==0) path.moveTo(p[0],p[1]) else path.lineTo(p[0],p[1]) }
            path.close(); paint.color=color; paint.style=Paint.Style.FILL; canvas.drawPath(path,paint)
        }
        paint.color=0xff525b63.toInt(); paint.textSize=10*density
        canvas.drawText(if(failure.isEmpty()) "Current CAD assembly · measured returns" else failure,10*density,18*density,paint)
        canvas.drawText(if(tof.stale || tof.points.isEmpty()) "Terrain unknown — no usable depth" else "Approximate projection · reference floor",
            10*density,size.height-10*density,paint)
    }
}
