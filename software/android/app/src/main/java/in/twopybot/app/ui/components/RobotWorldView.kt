package `in`.twopybot.app.ui.components

import android.content.Context
import android.opengl.GLES20 as GL
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.data.ToFPayload
import org.json.JSONObject
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.*

internal data class WorldInput(
    val tel: Telemetry = Telemetry(), val tof: ToFPayload = ToFPayload(),
    val trail: List<Pair<Float, Float>> = emptyList(), val follow: Boolean = true,
    val debug: Map<String,Float> = emptyMap(), val connected: Boolean = true,
    val pitchForwardSign: Float = -1f,
    val showGuidance: Boolean = true,
)

/** A real depth-buffered GLES world: shaded triangles, no CAD edge/wireframe pass. */
internal class RobotWorldView(context: Context, onError: (String) -> Unit,
                              private val onExplore: () -> Unit) : GLSurfaceView(context) {
    private val world = WorldRenderer(context) { message -> post { onError(message) } }
    private var lastX = 0f
    private var lastY = 0f
    private var lastCount = 0
    private var resetToken = 0
    private val pinch = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean {
            world.camera = world.camera.zoom(detector.scaleFactor)
            return true
        }
    })

    init {
        setEGLContextClientVersion(2)
        setEGLConfigChooser(8, 8, 8, 0, 24, 0)
        preserveEGLContextOnPause = true
        setRenderer(world)
        renderMode = RENDERMODE_CONTINUOUSLY
        contentDescription = "3D robot world. One finger orbits, two fingers pan, pinch zooms."
    }

    fun update(input: WorldInput, reset: Int) {
        if(world.input.follow != input.follow) {
            val pose = world.pose
            world.camera = if(input.follow) world.camera.followAt(pose) else world.camera.exploreAt(pose)
        }
        world.input = input
        if (reset != resetToken) { resetToken = reset; world.camera = WorldCamera() }
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        pinch.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                lastCount = event.pointerCount
                lastX = centroid(event, true); lastY = centroid(event, false)
            }
            MotionEvent.ACTION_MOVE -> {
                val x = centroid(event, true); val y = centroid(event, false)
                if (lastCount == event.pointerCount) {
                    if (event.pointerCount >= 2) {
                        if(world.input.follow) {
                            world.camera=world.camera.exploreAt(world.pose)
                            world.input=world.input.copy(follow=false)
                            onExplore()
                        }
                        world.camera = world.camera.pan(x - lastX, y - lastY, height)
                    } else world.camera = world.camera.orbit(x - lastX, y - lastY)
                }
                lastX = x; lastY = y; lastCount = event.pointerCount
            }
            MotionEvent.ACTION_POINTER_UP -> lastCount = 0
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                lastCount = 0; parent?.requestDisallowInterceptTouchEvent(false)
                if (event.actionMasked == MotionEvent.ACTION_UP) performClick()
            }
        }
        return true
    }

    override fun performClick(): Boolean { super.performClick(); return true }
    private fun centroid(event: MotionEvent, x: Boolean): Float =
        (0 until event.pointerCount).sumOf { (if (x) event.getX(it) else event.getY(it)).toDouble() }.toFloat() / event.pointerCount
}

private data class GpuMesh(val name: String, val buffer: Int, val count: Int, val color: FloatArray)

private class WorldRenderer(private val context: Context, private val onError: (String) -> Unit) : GLSurfaceView.Renderer {
    @Volatile var input = WorldInput()
    @Volatile var camera = WorldCamera()
    @Volatile var pose = WorldPose(); private set
    private val motion = WorldMotion()
    private var program = 0
    private var parts = emptyList<GpuMesh>()
    private lateinit var floor: GpuMesh
    private lateinit var shadow: GpuMesh
    private lateinit var guidance: GpuMesh
    private lateinit var arrow: GpuMesh
    private var shoulders = emptyList<GpuMesh>()
    private var arcs = emptyList<GpuMesh>()
    private var width = 1
    private var height = 1
    private val projection = FloatArray(16)
    private val view = FloatArray(16)
    private val model = FloatArray(16)
    private val mv = FloatArray(16)
    private val mvp = FloatArray(16)
    private var positionAttribute = 0
    private var normalAttribute = 0
    private var matrixUniform = 0
    private var viewUniform = 0
    private var colorUniform = 0
    private var kindUniform = 0
    private var pointUniform = 0
    private var modelUniform = 0
    private var anchorUniform = 0
    private var dynamicBuffer = 0
    private var trailBuffer = 0
    private var trailCount = 0
    private var lastTrail: List<Pair<Float,Float>>? = null

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        try {
            GL.glClearColor(.028f, .037f, .052f, 1f)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDisable(GL.GL_CULL_FACE) // includes the CAD's thin/open printed surfaces
            program = link(VERTEX_SHADER, FRAGMENT_SHADER)
            positionAttribute = GL.glGetAttribLocation(program, "aPosition")
            normalAttribute = GL.glGetAttribLocation(program, "aNormal")
            matrixUniform = GL.glGetUniformLocation(program, "uMvp")
            viewUniform = GL.glGetUniformLocation(program, "uMv")
            colorUniform = GL.glGetUniformLocation(program, "uColor")
            kindUniform = GL.glGetUniformLocation(program, "uKind")
            pointUniform = GL.glGetUniformLocation(program, "uPointSize")
            modelUniform = GL.glGetUniformLocation(program, "uModel")
            anchorUniform = GL.glGetUniformLocation(program, "uAnchor")
            val raw = context.assets.open("robot-world-model.json").bufferedReader().use { it.readText() }
            val source = JSONObject(raw).getJSONArray("parts")
            parts = (0 until source.length()).map { mesh(source.getJSONObject(it)) }
            floor = upload("floor", plane(150f), floatArrayOf(.055f,.065f,.083f,1f))
            shadow = upload("shadow", plane(1f, true), floatArrayOf(0f,0f,0f,.55f))
            guidance = upload("forward-guide",WorldReferenceGeometry.ribbon(),floatArrayOf(.06f,.49f,.83f,.65f))
            arrow = upload("front-arrow",WorldReferenceGeometry.forwardArrow(),floatArrayOf(.37f,.80f,1f,.72f))
            shoulders = listOf(-1f,1f).map { upload("reference-edge",WorldReferenceGeometry.shoulder(it),floatArrayOf(.38f,.44f,.53f,.24f)) }
            arcs = listOf(.5f,1f,1.5f,2f).map { upload("range-reference",WorldReferenceGeometry.rangeArc(it),floatArrayOf(.29f,.40f,.53f,.22f)) }
            val ids = IntArray(1); GL.glGenBuffers(1,ids,0); dynamicBuffer = ids[0]
            GL.glGenBuffers(1,ids,0); trailBuffer = ids[0]; lastTrail = null
        } catch (e: Exception) { program = 0; onError("3D view: ${e.message}") }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        this.width = width.coerceAtLeast(1); this.height = height.coerceAtLeast(1)
        GL.glViewport(0,0,this.width,this.height)
        Matrix.perspectiveM(projection,0,42f,this.width.toFloat()/this.height,.025f,120f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GL.glClear(GL.GL_COLOR_BUFFER_BIT or GL.GL_DEPTH_BUFFER_BIT)
        if (program == 0) return
        val state = input
        pose = motion.step(state.tel,state.debug,state.connected,System.nanoTime()/1_000_000,state.pitchForwardSign)
        val actual = pose
        val cam = camera
        val perspective=cam.view(actual,state.follow)
        Matrix.setLookAtM(view,0,perspective.eye[0],perspective.eye[1],perspective.eye[2],
            perspective.target[0],perspective.target[1],perspective.target[2],0f,1f,0f)
        GL.glUseProgram(program)
        GL.glEnableVertexAttribArray(positionAttribute); GL.glEnableVertexAttribArray(normalAttribute)
        Matrix.setIdentityM(model,0); draw(floor,0)
        if(lastTrail !== state.trail) {
            val vertices = state.trail.takeLast(3000).flatMap {
                WorldFrame.odometry(it.first,it.second,.002f).toList()+listOf(0f,1f,0f)
            }.toFloatArray()
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER,trailBuffer)
            GL.glBufferData(GL.GL_ARRAY_BUFFER,vertices.size*4,buffer(vertices),GL.GL_DYNAMIC_DRAW)
            trailCount = vertices.size/6; lastTrail = state.trail
        }
        if(trailCount > 1) {
            GL.glLineWidth(2f)
            Matrix.setIdentityM(model,0)
            draw(GpuMesh("trail",trailBuffer,trailCount,floatArrayOf(.10f,.44f,.47f,1f)),3,GL.GL_LINE_STRIP)
        }
        GL.glEnable(GL.GL_BLEND); GL.glBlendFunc(GL.GL_SRC_ALPHA,GL.GL_ONE_MINUS_SRC_ALPHA)
        GL.glDepthMask(false)
        if(state.showGuidance) {
            referenceMatrix()
            if(state.tel.v < -.025f) Matrix.scaleM(model,0,1f,1f,-1f)
            draw(guidance,5)
            GL.glLineWidth(1.5f)
            for(edge in shoulders) draw(edge,6,GL.GL_LINE_STRIP)
            for(arc in arcs) draw(arc,3,GL.GL_LINE_STRIP)
            draw(arrow,3)
        }
        Matrix.setIdentityM(model,0); Matrix.translateM(model,0,actual.x,.001f,actual.z)
        Matrix.rotateM(model,0,actual.heading,0f,1f,0f)
        Matrix.scaleM(model,0,.15f,1f,.10f)
        draw(shadow.copy(color=floatArrayOf(0f,0f,0f,.55f-actual.symbolicLift*4f)),2)
        GL.glDepthMask(true); GL.glDisable(GL.GL_BLEND)
        for (part in parts) {
            robotMatrix(actual.pitch,actual.roll)
            if(part.name.startsWith("wheel_")) {
                val left = part.name.startsWith("wheel_-1")
                val x = if(left) -.092f else .092f
                Matrix.translateM(model,0,x,.05f,0f)
                Matrix.rotateM(model,0,if(left) actual.leftWheel else actual.rightWheel,1f,0f,0f)
                Matrix.translateM(model,0,-x,-.05f,0f)
            }
            draw(part,1)
        }
        if (!state.tof.stale && state.tof.points.isNotEmpty()) {
            val vertices = state.tof.points.flatMap { listOf(it.x,it.z,-it.y-.05f,0f,1f,0f) }.toFloatArray()
            uploadDynamic(vertices)
            robotMatrix(0f)
            draw(GpuMesh("depth",dynamicBuffer,vertices.size/6,floatArrayOf(.96f,.68f,.30f,1f)),4,GL.GL_POINTS)
        }
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER,0)
    }

    private fun robotMatrix(pitch: Float, roll: Float = 0f) {
        WorldFrame.bodyMatrix(pose.copy(pitch=pitch,roll=roll)).copyInto(model)
    }

    private fun referenceMatrix() {
        Matrix.setIdentityM(model,0)
        Matrix.translateM(model,0,pose.x,0f,pose.z)
        Matrix.rotateM(model,0,pose.heading,0f,1f,0f)
    }

    private fun draw(mesh: GpuMesh, kind: Int, primitive: Int = GL.GL_TRIANGLES) {
        Matrix.multiplyMM(mv,0,view,0,model,0); Matrix.multiplyMM(mvp,0,projection,0,mv,0)
        GL.glUniformMatrix4fv(matrixUniform,1,false,mvp,0)
        GL.glUniformMatrix4fv(viewUniform,1,false,mv,0)
        GL.glUniformMatrix4fv(modelUniform,1,false,model,0)
        GL.glUniform2f(anchorUniform,pose.x,pose.z)
        GL.glUniform4fv(colorUniform,1,mesh.color,0)
        GL.glUniform1i(kindUniform,kind); GL.glUniform1f(pointUniform,(height/70f).coerceIn(4f,12f))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER,mesh.buffer)
        GL.glVertexAttribPointer(positionAttribute,3,GL.GL_FLOAT,false,24,0)
        GL.glVertexAttribPointer(normalAttribute,3,GL.GL_FLOAT,false,24,12)
        GL.glDrawArrays(primitive,0,mesh.count)
    }

    private fun mesh(part: JSONObject): GpuMesh {
        val name = part.getString("name")
        val raw = part.getJSONArray("vertices")
        val vertices = Array(raw.length()/3) { i -> floatArrayOf(raw.getDouble(i*3).toFloat(),
            raw.getDouble(i*3+2).toFloat(),-raw.getDouble(i*3+1).toFloat()) }
        val faces = part.getJSONArray("faces")
        val normals = Array(faces.length()/3) { FloatArray(3) }
        val adjacent = Array(vertices.size) { mutableListOf<Int>() }
        for (i in normals.indices) {
            val ids = IntArray(3) { faces.getInt(i*3+it) }
            val a = vertices[ids[0]]; val b = vertices[ids[1]]; val c = vertices[ids[2]]
            val u = FloatArray(3) { b[it]-a[it] }; val v = FloatArray(3) { c[it]-a[it] }
            val sign = if(name.startsWith("wheel_")) 1f else -1f
            normals[i] = normalized(floatArrayOf(sign*(u[1]*v[2]-u[2]*v[1]),
                sign*(u[2]*v[0]-u[0]*v[2]),sign*(u[0]*v[1]-u[1]*v[0])))
            ids.forEach { adjacent[it].add(i) }
        }
        val output = FloatArray(faces.length()*6)
        var offset = 0
        for (i in normals.indices) for (corner in 0..2) {
            val index = faces.getInt(i*3+corner)
            val normal = FloatArray(3)
            for (other in adjacent[index]) if(dot(normals[i],normals[other]) > .82f)
                for (axis in 0..2) normal[axis] += normals[other][axis]
            for(value in vertices[index]) output[offset++] = value
            for(value in normalized(normal)) output[offset++] = value
        }
        val color = android.graphics.Color.parseColor(part.getString("color"))
        val material = if(name == "2pybot_face") floatArrayOf(.15f,.66f,.64f,1f) else
            floatArrayOf(android.graphics.Color.red(color)/255f,android.graphics.Color.green(color)/255f,
                android.graphics.Color.blue(color)/255f,1f)
        return upload(name,output,material)
    }

    private fun upload(name: String, vertices: FloatArray, color: FloatArray): GpuMesh {
        val ids = IntArray(1); GL.glGenBuffers(1,ids,0); GL.glBindBuffer(GL.GL_ARRAY_BUFFER,ids[0])
        GL.glBufferData(GL.GL_ARRAY_BUFFER,vertices.size*4,buffer(vertices),GL.GL_STATIC_DRAW)
        return GpuMesh(name,ids[0],vertices.size/6,color)
    }
    private fun uploadDynamic(vertices: FloatArray) {
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER,dynamicBuffer)
        GL.glBufferData(GL.GL_ARRAY_BUFFER,vertices.size*4,buffer(vertices),GL.GL_DYNAMIC_DRAW)
    }
    private fun plane(radius: Float, uv: Boolean = false): FloatArray =
        listOf(-1f to -1f, 1f to -1f, 1f to 1f, -1f to -1f, 1f to 1f, -1f to 1f)
            .flatMap { (x,z) -> listOf(x*radius,0f,z*radius,if(uv) x else 0f,if(uv) z else 1f,0f) }.toFloatArray()
    private fun dot(a: FloatArray,b: FloatArray) = a.indices.sumOf { (a[it]*b[it]).toDouble() }.toFloat()
    private fun normalized(a: FloatArray): FloatArray {
        val length = sqrt(dot(a,a)).coerceAtLeast(.000001f)
        return FloatArray(3) { a[it]/length }
    }
    private fun buffer(values: FloatArray): FloatBuffer = ByteBuffer.allocateDirect(values.size*4)
        .order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(values); position(0) }

    private fun link(vertex: String, fragment: String): Int {
        fun shader(type: Int, source: String): Int {
            val id = GL.glCreateShader(type); GL.glShaderSource(id,source); GL.glCompileShader(id)
            val status = IntArray(1); GL.glGetShaderiv(id,GL.GL_COMPILE_STATUS,status,0)
            check(status[0] != 0) { GL.glGetShaderInfoLog(id) }
            return id
        }
        val vs = shader(GL.GL_VERTEX_SHADER,vertex); val fs = shader(GL.GL_FRAGMENT_SHADER,fragment)
        val id = GL.glCreateProgram(); GL.glAttachShader(id,vs); GL.glAttachShader(id,fs); GL.glLinkProgram(id)
        val status = IntArray(1); GL.glGetProgramiv(id,GL.GL_LINK_STATUS,status,0)
        check(status[0] != 0) { GL.glGetProgramInfoLog(id) }
        GL.glDeleteShader(vs); GL.glDeleteShader(fs)
        return id
    }

    companion object {
        private const val VERTEX_SHADER = """
            attribute vec3 aPosition;
            attribute vec3 aNormal;
            uniform mat4 uMvp;
            uniform mat4 uMv;
            uniform mat4 uModel;
            uniform float uPointSize;
            varying mediump vec3 vNormal;
            varying mediump vec3 vEye;
            varying mediump vec2 vAux;
            varying mediump vec3 vWorld;
            void main() {
                gl_Position = uMvp * vec4(aPosition,1.0);
                gl_PointSize = uPointSize;
                vEye = (uMv * vec4(aPosition,1.0)).xyz;
                vNormal = mat3(uMv) * aNormal;
                vAux = aNormal.xy;
                vWorld = (uModel * vec4(aPosition,1.0)).xyz;
            }
        """
        private const val FRAGMENT_SHADER = """
            precision mediump float;
            uniform vec4 uColor;
            uniform int uKind;
            uniform vec2 uAnchor;
            varying mediump vec3 vNormal;
            varying mediump vec3 vEye;
            varying mediump vec2 vAux;
            varying mediump vec3 vWorld;
            void main() {
                vec3 color = uColor.rgb;
                float alpha = uColor.a;
                if (uKind == 0) {
                    float pool = exp(-0.16*dot(vWorld.xz-uAnchor,vWorld.xz-uAnchor));
                    color = mix(vec3(0.039,0.047,0.062),vec3(0.068,0.082,0.107),pool);
                    vec2 cell = fract(vWorld.xz*1.25)-vec2(0.5);
                    float marker = 1.0-smoothstep(0.012,0.027,length(cell));
                    color += vec3(0.040,0.045,0.055)*marker;
                } else if (uKind == 1) {
                    vec3 n = normalize(vNormal);
                    vec3 l = normalize(vec3(-0.6,1.2,0.9));
                    vec3 v = normalize(-vEye);
                    float diffuse = max(dot(n,l),0.0);
                    float specular = pow(max(dot(n,normalize(l+v)),0.0),36.0);
                    color = color * (0.34 + 0.68*diffuse) + vec3(0.28*specular);
                    color += vec3(0.04,0.10,0.12)*pow(1.0-max(dot(n,v),0.0),3.0);
                } else if (uKind == 2) {
                    alpha *= exp(-3.0*dot(vAux,vAux));
                } else if (uKind == 4) {
                    vec2 p=(gl_PointCoord-vec2(0.5))*2.0;
                    float radius=dot(p,p);
                    if (radius > 1.0) discard;
                    color *= 0.45+0.55*sqrt(1.0-radius);
                } else if (uKind == 5) {
                    alpha *= (1.0-pow(abs(vAux.x),4.0))*pow(1.0-vAux.y,1.5);
                } else if (uKind == 6) {
                    alpha *= 1.0-vAux.y;
                }
                float fog = 1.0-exp(-0.012*dot(vEye,vEye));
                gl_FragColor = vec4(mix(color,vec3(0.028,0.037,0.052),fog),alpha);
            }
        """
    }
}
