package `in`.twopybot.app.net

import android.content.Context
import android.util.Log
import kotlinx.coroutines.*
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import org.webrtc.*
import java.util.concurrent.TimeUnit

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
class WhepClient(private val context: Context) {

    private var factory: PeerConnectionFactory? = null
    private var pc: PeerConnection? = null
    private var eglBase: EglBase? = null
    private val http = OkHttpClient.Builder()
        .connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(10, TimeUnit.SECONDS)
        .build()

    var onState: ((String) -> Unit)? = null

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

    /** Connect and render into [sink]. Safe to call repeatedly; reconnects. */
    suspend fun connect(url: String, sink: VideoSink) = withContext(Dispatchers.IO) {
        close()
        ensureFactory()
        onState?.invoke("connecting")

        // No ICE servers: robot and phone share a link, host candidates suffice.
        val cfg = PeerConnection.RTCConfiguration(emptyList<PeerConnection.IceServer>()).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
            continualGatheringPolicy = PeerConnection.ContinualGatheringPolicy.GATHER_ONCE
            bundlePolicy = PeerConnection.BundlePolicy.MAXBUNDLE
            rtcpMuxPolicy = PeerConnection.RtcpMuxPolicy.REQUIRE
        }

        val gathered = CompletableDeferred<Unit>()
        pc = factory!!.createPeerConnection(cfg, object : PeerConnection.Observer {
            override fun onIceCandidate(c: IceCandidate?) {}
            override fun onIceCandidatesRemoved(c: Array<out IceCandidate>?) {}
            override fun onSignalingChange(s: PeerConnection.SignalingState?) {}
            override fun onIceConnectionChange(s: PeerConnection.IceConnectionState?) {
                onState?.invoke(s?.name?.lowercase() ?: "?")
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
                (transceiver?.receiver?.track() as? VideoTrack)?.addSink(sink)
            }
            override fun onAddTrack(r: RtpReceiver?, s: Array<out MediaStream>?) {
                (r?.track() as? VideoTrack)?.addSink(sink)
            }
        }) ?: run { onState?.invoke("factory failed"); return@withContext }

        pc!!.addTransceiver(MediaStreamTrack.MediaType.MEDIA_TYPE_VIDEO,
            RtpTransceiver.RtpTransceiverInit(RtpTransceiver.RtpTransceiverDirection.RECV_ONLY))

        val offer = suspendCancellableSdp { obs -> pc!!.createOffer(obs, MediaConstraints()) }
            ?: run { onState?.invoke("offer failed"); return@withContext }
        setLocal(offer)

        // brief window for host candidates; do not block forever on a bad link
        withTimeoutOrNull(700) { gathered.await() }
        val sdp = pc!!.localDescription?.description ?: offer.description

        val answer = runCatching {
            http.newCall(
                Request.Builder().url(url)
                    .post(sdp.toRequestBody("application/sdp".toMediaType()))
                    .header("Content-Type", "application/sdp")
                    .build()
            ).execute().use { r ->
                if (!r.isSuccessful) throw java.io.IOException("WHEP ${r.code}")
                r.body!!.string()
            }
        }.getOrElse {
            Log.w("Whep", "signalling failed", it)
            onState?.invoke("no stream (${it.message})")
            return@withContext
        }

        setRemote(SessionDescription(SessionDescription.Type.ANSWER, answer))
        onState?.invoke("connected")
    }

    private suspend fun setLocal(sdp: SessionDescription) = suspendCancellableCoroutineUnit { cont ->
        pc!!.setLocalDescription(object : SdpObserverAdapter() {
            override fun onSetSuccess() { cont() }
            override fun onSetFailure(e: String?) { cont() }
        }, sdp)
    }

    private suspend fun setRemote(sdp: SessionDescription) = suspendCancellableCoroutineUnit { cont ->
        pc!!.setRemoteDescription(object : SdpObserverAdapter() {
            override fun onSetSuccess() { cont() }
            override fun onSetFailure(e: String?) { onState?.invoke("answer rejected"); cont() }
        }, sdp)
    }

    fun close() {
        runCatching { pc?.dispose() }
        pc = null
    }

    fun release() {
        close()
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

private suspend fun suspendCancellableCoroutineUnit(
    block: (() -> Unit) -> Unit
): Unit = suspendCancellableCoroutine { cont ->
    block { if (cont.isActive) cont.resume(Unit) {} }
}
