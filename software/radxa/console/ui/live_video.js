/* Shared streamless-track WHEP viewer, with explicit lifetime and session cleanup. */
export function liveVideo(video, notify = () => {}) {
  let pc = null, session = null, abort = null;
  function stop() {
    abort?.abort(); abort = null;
    pc?.close(); pc = null; video.srcObject = null;
    if (session) fetch(session, { method: 'DELETE' }).catch(() => {});
    session = null;
  }
  async function start() {
    stop();
    const peer = new RTCPeerConnection({ iceServers: [] });
    pc = peer;
    const controller = new AbortController(); abort = controller;
    const url = `${location.protocol}//${location.hostname}:8889/cam/whep`;
    peer.addTransceiver('video', { direction: 'recvonly' });
    const remote = new MediaStream(); video.srcObject = remote;
    peer.ontrack = e => {
      remote.addTrack(e.track);
      video.play().catch(() => {});
    };
    peer.onconnectionstatechange = () => notify(`Camera ${peer.connectionState}`);
    try {
      await peer.setLocalDescription(await peer.createOffer());
      await new Promise(resolve => {
        if (peer.iceGatheringState === 'complete') return resolve();
        const timer = setTimeout(resolve, 2000);
        peer.onicegatheringstatechange = () => {
          if (peer.iceGatheringState === 'complete') { clearTimeout(timer); resolve(); }
        };
      });
      if (pc !== peer) return;
      const response = await fetch(url, { method: 'POST',
        headers: { 'Content-Type': 'application/sdp' }, body: peer.localDescription.sdp,
        signal: controller.signal });
      if (!response.ok) throw new Error(`WHEP ${response.status}`);
      const resource = response.headers.get('Location');
      if (resource) session = new URL(resource, url).href;
      await peer.setRemoteDescription({ type: 'answer', sdp: await response.text() });
    } catch (e) {
      if (pc === peer) { stop(); notify(e.message); }
    }
  }
  return { start, stop };
}
