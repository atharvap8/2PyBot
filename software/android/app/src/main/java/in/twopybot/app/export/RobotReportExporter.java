package in.twopybot.app.export;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.pdf.PdfDocument;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/** Native Android PDF export; no network, storage permission or external renderer. */
public final class RobotReportExporter {
    private RobotReportExporter() {}

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
