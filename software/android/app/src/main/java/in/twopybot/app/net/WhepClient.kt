package `in`.twopybot.app.net

import android.content.Context
import android.util.Log
import android.os.Handler
import android.os.Looper
import kotlinx.coroutines.*
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import org.webrtc.*
import java.util.concurrent.TimeUnit
import java.io.IOException
import kotlin.coroutines.resumeWithException

/**
 * WHEP (WebRTC-HTTP Egress Protocol) client for MediaMTX.
 *
 * Deliberately non-trickle: we gather ICE candidates for a moment, then POST
 * one complete SDP offer and apply the answer. On a point-to-point hotspot the
 * host candidate resolves instantly, so there is nothing to trickle and no
 * STUN server to wait on — that is where the ~100 ms comes from.
 *
 * Hardware decode is requested via DefaultVideoDecoderFactory, which uses
 * MediaCodec when the phone has an H.264 decoder (every modern phone does).
 */
class WhepClient(context: Context) {
    private val context = context.applicationContext
    private val main = Handler(Looper.getMainLooper())
    private var generation = 0L
    private var remoteVideo: VideoTrack? = null
    private var sink: VideoSink? = null
    private var signalling: Call? = null
    private var resourceUrl: String? = null

    private var factory: PeerConnectionFactory? = null
    private var pc: PeerConnection? = null
    private var eglBase: EglBase? = null
    private val http = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(10, TimeUnit.SECONDS)
        .callTimeout(15, TimeUnit.SECONDS)
        .build()

    var onState: ((String) -> Unit)? = null

    private fun state(epoch: Long, value: String) {
        main.post { if (epoch == generation) onState?.invoke(value) }
    }

    /** A renderer is a temporary UI attachment, not the owner of the connection. */
    fun attachSink(value: VideoSink) {
        if (sink === value) return
        sink?.let { remoteVideo?.removeSink(it) }
        sink = value
        remoteVideo?.addSink(value)
    }

    fun detachSink(value: VideoSink) {
        if (sink !== value) return
        remoteVideo?.removeSink(value)
        sink = null
    }

    private fun receiveTrack(epoch: Long, value: VideoTrack?) {
        main.post {
            if (epoch != generation || value == null || remoteVideo === value) return@post
            sink?.let { remoteVideo?.removeSink(it) }
            remoteVideo = value
            sink?.let { value.addSink(it) }
        }
    }

    fun egl(): EglBase = eglBase ?: EglBase.create().also { eglBase = it }

    private fun ensureFactory() {
        if (factory != null) return
        PeerConnectionFactory.initialize(
            PeerConnectionFactory.InitializationOptions.builder(context)
                .setEnableInternalTracer(false)
                .createInitializationOptions()
        )
        factory = PeerConnectionFactory.builder()
            .setVideoDecoderFactory(DefaultVideoDecoderFactory(egl().eglBaseContext))
            .setVideoEncoderFactory(DefaultVideoEncoderFactory(egl().eglBaseContext, true, true))
            .createPeerConnectionFactory()
    }

    /** Native peer lifecycle is serialized on Main; only HTTP runs on IO. */
    suspend fun connect(url: String) = withContext(Dispatchers.Main.immediate) {
        close()
        val epoch = generation
        try {
        ensureFactory()
        state(epoch, "connecting")

        // No ICE servers: robot and phone share a link, host candidates suffice.
        val cfg = PeerConnection.RTCConfiguration(emptyList<PeerConnection.IceServer>()).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
            continualGatheringPolicy = PeerConnection.ContinualGatheringPolicy.GATHER_ONCE
            bundlePolicy = PeerConnection.BundlePolicy.MAXBUNDLE
            rtcpMuxPolicy = PeerConnection.RtcpMuxPolicy.REQUIRE
        }

        val gathered = CompletableDeferred<Unit>()
        val connection = factory!!.createPeerConnection(cfg, object : PeerConnection.Observer {
            override fun onIceCandidate(c: IceCandidate?) {}
            override fun onIceCandidatesRemoved(c: Array<out IceCandidate>?) {}
            override fun onSignalingChange(s: PeerConnection.SignalingState?) {}
            override fun onIceConnectionChange(s: PeerConnection.IceConnectionState?) {
                state(epoch, when (s) {
                    PeerConnection.IceConnectionState.CONNECTED, PeerConnection.IceConnectionState.COMPLETED -> "connected"
                    PeerConnection.IceConnectionState.FAILED -> "failed"
                    PeerConnection.IceConnectionState.DISCONNECTED -> "disconnected"
                    PeerConnection.IceConnectionState.CLOSED -> "closed"
                    else -> "connecting"
                })
            }
            override fun onIceConnectionReceivingChange(b: Boolean) {}
            override fun onIceGatheringChange(s: PeerConnection.IceGatheringState?) {
                if (s == PeerConnection.IceGatheringState.COMPLETE) gathered.complete(Unit)
            }
            override fun onAddStream(st: MediaStream?) {}
            override fun onRemoveStream(st: MediaStream?) {}
            override fun onDataChannel(d: DataChannel?) {}
            override fun onRenegotiationNeeded() {}
            override fun onTrack(transceiver: RtpTransceiver?) {
                receiveTrack(epoch, transceiver?.receiver?.track() as? VideoTrack)
            }
            override fun onAddTrack(r: RtpReceiver?, s: Array<out MediaStream>?) {
                receiveTrack(epoch, r?.track() as? VideoTrack)
            }
        }) ?: throw IOException("Peer factory failed")
        pc = connection

        connection.addTransceiver(MediaStreamTrack.MediaType.MEDIA_TYPE_VIDEO,
            RtpTransceiver.RtpTransceiverInit(RtpTransceiver.RtpTransceiverDirection.RECV_ONLY))

        val offer = suspendCancellableSdp { obs -> connection.createOffer(obs, MediaConstraints()) }
            ?: throw IOException("SDP offer failed")
        if (epoch != generation) return@withContext
        setDescription(connection, offer, true)

        // brief window for host candidates; do not block forever on a bad link
        withTimeoutOrNull(700) { gathered.await() }
        if (epoch != generation) return@withContext
        val sdp = connection.localDescription?.description ?: offer.description

        val call = http.newCall(
                Request.Builder().url(url)
                    .post(sdp.toRequestBody("application/sdp".toMediaType()))
                    .header("Content-Type", "application/sdp")
                    .build()
            )
        signalling = call
        val (answer, location) = withContext(Dispatchers.IO) {
            call.execute().use { r ->
                if (!r.isSuccessful) throw java.io.IOException("WHEP ${r.code}")
                Pair(r.body?.string() ?: throw IOException("Empty WHEP answer"),
                    r.header("Location")?.let { r.request.url.resolve(it)?.toString() })
            }
        }
        if (epoch != generation) return@withContext
        signalling = null
        resourceUrl = location
        setDescription(connection, SessionDescription(SessionDescription.Type.ANSWER, answer), false)
        } catch (cancelled: CancellationException) {
            if (epoch == generation) close()
            throw cancelled
        } catch (error: Exception) {
            Log.w("Whep", "connection failed", error)
            if (epoch == generation) {
                close()
                onState?.invoke("error: ${error.message}")
            }
        }
    }

    private suspend fun setDescription(connection: PeerConnection, sdp: SessionDescription, local: Boolean): Unit =
        suspendCancellableCoroutine { cont ->
            val observer = object : SdpObserverAdapter() {
                override fun onSetSuccess() { if (cont.isActive) cont.resume(Unit) {} }
                override fun onSetFailure(e: String?) {
                    if (cont.isActive) cont.resumeWithException(IOException(e ?: "SDP rejected"))
                }
            }
            if (local) connection.setLocalDescription(observer, sdp) else connection.setRemoteDescription(observer, sdp)
        }

    fun close() {
        generation++
        signalling?.cancel()
        signalling = null
        sink?.let { runCatching { remoteVideo?.removeSink(it) } }
        remoteVideo = null
        runCatching { pc?.close() }
        runCatching { pc?.dispose() }
        pc = null
        resourceUrl?.let { resource ->
            http.newCall(Request.Builder().url(resource).delete().build()).enqueue(object : Callback {
                override fun onFailure(call: Call, e: IOException) {}
                override fun onResponse(call: Call, response: Response) { response.close() }
            })
        }
        resourceUrl = null
    }

    fun release() {
        close()
        sink = null
        runCatching { factory?.dispose() }
        factory = null
        runCatching { eglBase?.release() }
        eglBase = null
    }
}

// ---- small helpers so the callback API reads like a coroutine ----
open class SdpObserverAdapter : SdpObserver {
    override fun onCreateSuccess(p0: SessionDescription?) {}
    override fun onSetSuccess() {}
    override fun onCreateFailure(p0: String?) {}
    override fun onSetFailure(p0: String?) {}
}

private suspend fun suspendCancellableSdp(
    block: (SdpObserver) -> Unit
): SessionDescription? = suspendCancellableCoroutine { cont ->
    block(object : SdpObserverAdapter() {
        override fun onCreateSuccess(sdp: SessionDescription?) {
            if (cont.isActive) cont.resume(sdp) {}
        }
        override fun onCreateFailure(e: String?) {
            if (cont.isActive) cont.resume(null) {}
        }
    })
}
