package in.twopybot.app.export;

import android.graphics.Canvas;
import android.graphics.Bitmap;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.pdf.PdfDocument;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Locale;

/** Native Android PDF export; no network, storage permission or external renderer. */
public final class RobotReportExporter {
    private RobotReportExporter() {}

    public static Bitmap renderTrail(List<PointF> trail, float x, float y, float heading,
                                     Map<String, String> telemetry, boolean includeTelemetry) {
        Bitmap image = Bitmap.createBitmap(1400, 1100, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(image);
        canvas.drawColor(Color.rgb(20, 25, 31));
        Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setTypeface(Typeface.MONOSPACE);
        paint.setTextSize(22);
        paint.setColor(Color.WHITE);
        canvas.drawText("2PyBot encoder trail — dead reckoning", 40, 40, paint);
        float footer = includeTelemetry ? 60 + ((telemetry.size() + 2) / 3) * 27 : 30;
        float mapWidth = 1320, mapHeight = Math.max(400, 990 - footer);
        float minX = x, maxX = x, minY = y, maxY = y;
        for (PointF p : trail) { minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y); }
        float span = Math.max(1.5f, Math.max(maxX - minX, maxY - minY) + 1);
        float cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
        float scale = Math.min(mapWidth, mapHeight) / span;
        float centerX = 700, centerY = 70 + mapHeight / 2;
        paint.setColor(Color.rgb(51, 58, 68)); paint.setStrokeWidth(1);
        for (float g = (float)Math.floor(cx-span/2); g <= cx+span/2; g += .5f) {
            float px = centerX + (g-cx)*scale; canvas.drawLine(px,70,px,70+mapHeight,paint);
        }
        for (float g = (float)Math.floor(cy-span/2); g <= cy+span/2; g += .5f) {
            float py = centerY-(g-cy)*scale; canvas.drawLine(40,py,1360,py,paint);
        }
        if (trail.size() > 1) {
            Path path = new Path(); boolean first = true;
            for (PointF point : trail) {
                float px = centerX+(point.x-cx)*scale, py = centerY-(point.y-cy)*scale;
                if (first) { path.moveTo(px,py); first=false; } else path.lineTo(px,py);
            }
            paint.setColor(Color.rgb(53,224,216)); paint.setStyle(Paint.Style.STROKE); paint.setStrokeWidth(4);
            canvas.drawPath(path,paint); paint.setStyle(Paint.Style.FILL);
        }
        canvas.save(); canvas.translate(centerX+(x-cx)*scale,centerY-(y-cy)*scale); canvas.rotate(-heading);
        Path arrow = new Path(); arrow.moveTo(25,0); arrow.lineTo(-18,14); arrow.lineTo(-8,0); arrow.lineTo(-18,-14); arrow.close();
        paint.setColor(Color.rgb(74,222,128)); canvas.drawPath(arrow,paint); canvas.restore();
        paint.setColor(Color.LTGRAY); canvas.drawText(String.format(Locale.US,"%.1f m across | %d trail points",span,trail.size()),40,90+mapHeight,paint);
        if (includeTelemetry) {
            int index=0; paint.setTextSize(18);
            for (Map.Entry<String,String> entry : telemetry.entrySet()) {
                canvas.drawText(entry.getKey()+"  "+entry.getValue(),40+(index%3)*450,125+mapHeight+(index/3)*27,paint); index++;
            }
        }
        return image;
    }

    public static void writeMapPdf(OutputStream output, Bitmap image, String host, String version)
            throws IOException {
        PdfDocument document = new PdfDocument();
        try {
            PdfDocument.Page page=document.startPage(new PdfDocument.PageInfo.Builder(842,680,1).create());
            Canvas canvas=page.getCanvas(); canvas.drawColor(Color.WHITE);
            Paint paint=new Paint(Paint.ANTI_ALIAS_FLAG); paint.setTextSize(12); paint.setColor(Color.DKGRAY);
            canvas.drawText("2PyBot trail | "+host+" | App "+version+" | "+new Date(),24,22,paint);
            canvas.drawBitmap(image,null,new RectF(24,36,818,660),paint);
            document.finishPage(page); document.writeTo(output);
        } finally { document.close(); }
    }

    public static void writeLog(OutputStream output, List<String> lines, String host, String version)
            throws IOException {
        PdfDocument document = new PdfDocument();
        Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
        text.setTypeface(Typeface.MONOSPACE);
        text.setTextSize(9f);
        text.setColor(Color.rgb(25, 30, 38));
        Paint heading = new Paint(Paint.ANTI_ALIAS_FLAG);
        heading.setTypeface(Typeface.create(Typeface.SANS_SERIF, Typeface.BOLD));
        heading.setTextSize(15f);
        String timestamp = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss Z", Locale.US).format(new Date());
        int pageNumber = 0;
        PdfDocument.Page page = null;
        float y = 0;
        try {
            List<String> source = lines.isEmpty() ? java.util.Collections.singletonList("No log rows available.") : lines;
            for (String row : source) {
                for (String paragraph : row.split("\\r?\\n", -1)) {
                    String remainder = paragraph;
                    do {
                        if (page == null || y > 790) {
                            if (page != null) document.finishPage(page);
                            page = document.startPage(new PdfDocument.PageInfo.Builder(595, 842, ++pageNumber).create());
                            Canvas canvas = page.getCanvas();
                            canvas.drawColor(Color.WHITE);
                            canvas.drawText("2PyBot robot log", 32, 36, heading);
                            canvas.drawText("Host " + host + " | App " + version, 32, 54, text);
                            canvas.drawText(timestamp + " | " + lines.size() + " buffered rows | page " + pageNumber, 32, 68, text);
                            y = 92;
                        }
                        int length = remainder.isEmpty() ? 0 : text.breakText(remainder, true, 531, null);
                        if (!remainder.isEmpty() && length < 1) length = 1;
                        page.getCanvas().drawText(remainder.substring(0, length), 32, y, text);
                        y += 12;
                        remainder = remainder.substring(length);
                    } while (!remainder.isEmpty());
                }
            }
            if (page != null) document.finishPage(page);
            document.writeTo(output);
        } finally {
            document.close();
        }
    }
}
