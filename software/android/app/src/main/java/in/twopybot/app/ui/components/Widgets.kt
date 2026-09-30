package `in`.twopybot.app.ui.components

import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import `in`.twopybot.app.data.PadMirror
import `in`.twopybot.app.data.Param
import `in`.twopybot.app.data.Telemetry
import `in`.twopybot.app.ui.theme.*
import kotlin.math.*

@Composable
fun Section(title: String, hint: String? = null, content: @Composable ColumnScope.() -> Unit) {
    Column(
        Modifier.fillMaxWidth().padding(vertical = 6.dp)
            .clip(RoundedCornerShape(18.dp))
            .background(Panel).border(1.dp, Line, RoundedCornerShape(18.dp))
            .padding(14.dp)
    ) {
        Text(title, style = MaterialTheme.typography.titleMedium, color = TextHi)
        if (hint != null) {
            Spacer(Modifier.height(2.dp))
            Text(hint, fontSize = 12.sp, color = TextDim, lineHeight = 16.sp)
        }
        Spacer(Modifier.height(10.dp))
        content()
    }
}

@Composable
fun Tile(label: String, value: String, accent: Color = Cyan, modifier: Modifier = Modifier) {
    Column(
        modifier.clip(RoundedCornerShape(14.dp)).background(PanelSoft)
            .border(1.dp, Line, RoundedCornerShape(14.dp))
            .padding(vertical = 10.dp, horizontal = 8.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(label.uppercase(), fontSize = 9.5.sp, letterSpacing = 1.sp, color = TextDim)
        Spacer(Modifier.height(3.dp))
        Text(value, style = Mono.copy(fontSize = 18.sp, color = accent, fontWeight = FontWeight.SemiBold),
             maxLines = 1, textAlign = TextAlign.Center)
    }
}

@Composable
fun StatusStrip(tel: Telemetry, connected: Boolean, unsaved: Boolean) {
    val dot by animateColorAsState(
        if (!connected || tel.stale) Red else if (tel.balancing) Green else Amber, label = "dot")
    Row(
        Modifier.fillMaxWidth().clip(RoundedCornerShape(14.dp)).background(Panel)
            .border(1.dp, Line, RoundedCornerShape(14.dp)).padding(10.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Canvas(Modifier.size(10.dp)) { drawCircle(dot) }
        Spacer(Modifier.width(8.dp))
        Text(
            when {
                !connected -> "NO CONSOLE"
                tel.stale  -> "NO ESP32"
                tel.balancing -> "BALANCING"
                else -> "IDLE"
            },
            style = Mono.copy(fontSize = 12.sp), color = dot
        )
        Spacer(Modifier.weight(1f))
        Text("${"%.1f".format(tel.rxHz)} Hz", style = Mono.copy(fontSize = 12.sp), color = TextDim)
        if (unsaved) {
            Spacer(Modifier.width(10.dp))
            Text("● unsaved", style = Mono.copy(fontSize = 12.sp), color = Amber)
        }
    }
}

/** Attitude: a horizon that rolls with pitch, plus the rate needle. */
@Composable
fun AttitudeGauge(pitch: Float, rate: Float, modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val r = min(size.width, size.height) / 2f * 0.92f
        val c = Offset(size.width / 2f, size.height / 2f)
        drawCircle(PanelSoft, r, c)
        drawCircle(Line, r, c, style = Stroke(1.5f))

        // horizon: 1 degree = 2% of radius, clamped so it never leaves the dial
        val off = (pitch.coerceIn(-30f, 30f) / 30f) * r * 0.75f
        val halfW = sqrt(max(0f, r * r - off * off))
        drawLine(Cyan, Offset(c.x - halfW, c.y + off), Offset(c.x + halfW, c.y + off),
                 strokeWidth = 3f, cap = StrokeCap.Round)
        // ladder
        for (d in listOf(-20, -10, 10, 20)) {
            val y = c.y + (d / 30f) * r * 0.75f
            val w = r * 0.18f
            drawLine(Line, Offset(c.x - w, y), Offset(c.x + w, y), strokeWidth = 1.5f)
        }
        // fixed aircraft mark
        drawLine(Amber, Offset(c.x - r * 0.3f, c.y), Offset(c.x - r * 0.08f, c.y), strokeWidth = 3f)
        drawLine(Amber, Offset(c.x + r * 0.08f, c.y), Offset(c.x + r * 0.3f, c.y), strokeWidth = 3f)
        // rate needle around the rim
        val a = (-rate.coerceIn(-300f, 300f) / 300f) * 120f - 90f
        val ar = Math.toRadians(a.toDouble())
        drawLine(Violet, c + Offset((cos(ar) * r * 0.78f).toFloat(), (sin(ar) * r * 0.78f).toFloat()),
                 c + Offset((cos(ar) * r).toFloat(), (sin(ar) * r).toFloat()),
                 strokeWidth = 4f, cap = StrokeCap.Round)
    }
}

/**
 * Read-only mirror of the gamepad. The pad is paired to the ESP32 over
 * Bluepad32 and the app never injects input — this only shows what the
 * firmware reported, which is exactly what you want when diagnosing
 * "why did it turn when I pushed straight forward".
 */
@Composable
fun StickMirror(pad: PadMirror, modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val r = min(size.width, size.height) / 2f * 0.9f
        val c = Offset(size.width / 2f, size.height / 2f)
        drawCircle(PanelSoft, r, c)
        drawCircle(Line, r, c, style = Stroke(1.5f))
        drawCircle(Line, r * 0.33f, c, style = Stroke(1f))
        drawLine(Line, Offset(c.x - r, c.y), Offset(c.x + r, c.y), strokeWidth = 1f)
        drawLine(Line, Offset(c.x, c.y - r), Offset(c.x, c.y + r), strokeWidth = 1f)

        // the cardinal "snap" cone the firmware applies, drawn so you can see
        // how much off-axis travel is ignored before steering engages
        val snap = Math.toRadians(10.0)
        for (s in listOf(-1, 1)) {
            val dx = (sin(snap) * r * s).toFloat()
            drawLine(Color(0x3335E0D8), Offset(c.x, c.y), Offset(c.x + dx, c.y - r), strokeWidth = 1f)
            drawLine(Color(0x3335E0D8), Offset(c.x, c.y), Offset(c.x + dx, c.y + r), strokeWidth = 1f)
        }

        val p = Offset(c.x + pad.steer.coerceIn(-1f, 1f) * r,
                       c.y - pad.fwd.coerceIn(-1f, 1f) * r)
        drawLine(Cyan, c, p, strokeWidth = 2.5f, cap = StrokeCap.Round)
        drawCircle(if (pad.speedHigh) Violet else Cyan, r * 0.13f, p)
    }
}

/** Top-down trail. Dead reckoning, so it drifts — labelled as such in the UI. */
@Composable
fun TrailMap(trail: List<Pair<Float, Float>>, x: Float, y: Float, headingDeg: Float,
             modifier: Modifier = Modifier) {
    Canvas(modifier) {
        val pts = trail + (x to y)
        val minX = pts.minOf { it.first }; val maxX = pts.maxOf { it.first }
        val minY = pts.minOf { it.second }; val maxY = pts.maxOf { it.second }
        val span = max(1.5f, max(maxX - minX, maxY - minY) + 1.0f)
        val cx = (minX + maxX) / 2f; val cy = (minY + maxY) / 2f
        val s = min(size.width, size.height) / span
        fun X(v: Float) = size.width / 2f + (v - cx) * s
        fun Y(v: Float) = size.height / 2f - (v - cy) * s

        var g = floor(cx - span / 2)
        while (g <= cx + span / 2) {
            drawLine(Color(0x14FFFFFF), Offset(X(g), 0f), Offset(X(g), size.height), 1f); g += 0.5f
        }
        g = floor(cy - span / 2)
        while (g <= cy + span / 2) {
            drawLine(Color(0x14FFFFFF), Offset(0f, Y(g)), Offset(size.width, Y(g)), 1f); g += 0.5f
        }
        if (trail.size > 1) {
            val path = Path().apply {
                moveTo(X(trail[0].first), Y(trail[0].second))
                trail.drop(1).forEach { lineTo(X(it.first), Y(it.second)) }
            }
            drawPath(path, Cyan.copy(alpha = 0.85f), style = Stroke(2.5f, cap = StrokeCap.Round))
        }
        // robot arrow, drawn about its own centre
        val th = Math.toRadians(-headingDeg.toDouble())
        val rr = 13f
        val px = X(x); val py = Y(y)
        fun at(ang: Double, rad: Float) =
            Offset(px + (cos(th + ang) * rad).toFloat(), py + (sin(th + ang) * rad).toFloat())
        val p = Path().apply {
            val nose = at(0.0, rr * 1.5f)
            moveTo(nose.x, nose.y)
            at(2.5, rr).let { lineTo(it.x, it.y) }
            lineTo(px, py)
            at(-2.5, rr).let { lineTo(it.x, it.y) }
            close()
        }
        drawPath(p, Green)
    }
}

/** One tunable. Commits on release, not on every pixel — the robot is live. */
@Composable
fun ParamSlider(p: Param, onCommit: (Float) -> Unit) {
    var live by remember(p.key, p.value) { mutableStateOf(p.value) }
    var dragging by remember { mutableStateOf(false) }
    val shown = if (dragging) live else p.value

    Column(Modifier.fillMaxWidth().padding(vertical = 5.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.weight(1f)) {
                Text(p.key, style = Mono.copy(fontSize = 13.sp), color = TextHi)
                Text(p.desc, fontSize = 11.sp, color = TextDim, lineHeight = 14.sp)
            }
            Text(fmt(shown), style = Mono.copy(fontSize = 14.sp),
                 color = if (p.dirty) Amber else Cyan)
        }
        Slider(
            value = shown.coerceIn(p.lo, p.hi),
            onValueChange = { dragging = true; live = it },
            onValueChangeFinished = { dragging = false; onCommit(live) },
            valueRange = p.lo..p.hi,
            colors = SliderDefaults.colors(
                thumbColor = if (p.dirty) Amber else Cyan,
                activeTrackColor = (if (p.dirty) Amber else Cyan).copy(alpha = 0.6f),
                inactiveTrackColor = Line),
            modifier = Modifier.height(28.dp)
        )
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(fmt(p.lo), fontSize = 10.sp, color = TextDim)
            Text(fmt(p.hi), fontSize = 10.sp, color = TextDim)
        }
    }
}

fun fmt(v: Float): String = when {
    abs(v) >= 1000f -> "%.0f".format(v)
    abs(v) >= 10f   -> "%.2f".format(v)
    else            -> "%.4f".format(v).trimEnd('0').trimEnd('.')
}
