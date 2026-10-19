"""HTTP surface for guided geometry calibration, separate from exposure tuning."""
import io
import json

from fastapi import HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field


class Board(BaseModel):
    kind: str = "charuco"
    columns: int = Field(8, ge=4, le=14)
    rows: int = Field(6, ge=4, le=12)
    square_mm: float = Field(20, ge=5, le=60)
    marker_ratio: float = Field(.70, ge=.4, le=.85)


class Start(BaseModel):
    board: Board = Field(default_factory=Board)
    goal: int = Field(32, ge=24, le=60)


def register(app, service):
    def attempt(operation):
        try:
            return operation()
        except (ValueError, OSError, ImportError) as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.get("/api/calibration/status")
    def status():
        return service.status()

    @app.post("/api/calibration/start")
    def start(body: Start):
        values = body.board.model_dump() if hasattr(body.board, "model_dump") else body.board.dict()
        return attempt(lambda: service.start(values, body.goal))

    @app.post("/api/calibration/action/{action}")
    def action(action: str):
        return attempt(lambda: service.action(action))

    @app.get("/api/calibration/board.svg")
    def svg(kind: str = "charuco", columns: int = 8, rows: int = 6, square_mm: float = 20):
        maths = attempt(service.maths)
        settings = attempt(lambda: maths.board_settings(dict(kind=kind, columns=columns, rows=rows, square_mm=square_mm)))
        return Response(attempt(lambda: maths.board_svg(settings)), media_type="image/svg+xml",
                        headers={"Content-Disposition": "attachment; filename=2PyBot-calibration-board.svg"})

    @app.get("/api/calibration/board.pdf")
    def pdf(kind: str = "charuco", columns: int = 8, rows: int = 6, square_mm: float = 20):
        def render():
            from reportlab.pdfgen import canvas
            from reportlab.lib.utils import ImageReader
            from reportlab.lib.units import mm
            maths = service.maths()
            settings = maths.board_settings(dict(kind=kind, columns=columns, rows=rows, square_mm=square_mm))
            width, height = columns*square_mm, rows*square_mm
            page_width, page_height = max(210, width+30), max(297, height+65)
            output = io.BytesIO()
            report = canvas.Canvas(output, pagesize=(page_width*mm, page_height*mm))
            y = (page_height-20-height)*mm
            report.drawImage(ImageReader(io.BytesIO(maths.board_png(settings))), (page_width-width)/2*mm, y, width*mm, height*mm)
            report.setFont("Helvetica", 11)
            report.drawString(15*mm, y-12*mm, f"{kind}: {columns} x {rows} squares, {square_mm:g} mm each")
            report.drawString(15*mm, y-20*mm, "Print at actual size / 100%. Measure a square before capturing.")
            report.line(15*mm, y-30*mm, 65*mm, y-30*mm)
            report.drawString(15*mm, y-37*mm, "This line must measure 50 mm.")
            report.save()
            return output.getvalue()
        return Response(attempt(render), media_type="application/pdf",
                        headers={"Content-Disposition": "attachment; filename=2PyBot-calibration-board.pdf"})

    @app.get("/api/calibration/preview.jpg")
    def preview(corrected: bool = False):
        image = service.corrected if corrected else service.preview
        if not image and not corrected:
            image, _ = service.frame_source()
        if not image:
            raise HTTPException(404, "Preview is not ready")
        return Response(image, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/calibration/session/{identifier}.json")
    def session(identifier: str):
        folder = attempt(lambda: service.folder(identifier))
        path = folder / "session.json"
        if not path.is_file():raise HTTPException(404, "Session not found")
        return FileResponse(path, media_type="application/json", filename=f"2PyBot-calibration-{identifier}.json")

    @app.get("/api/calibration/session/{identifier}/report.pdf")
    def report(identifier: str):
        path = attempt(lambda: service.report(identifier))
        return FileResponse(path, media_type="application/pdf", filename="2PyBot-camera-calibration-report.pdf")

    @app.get("/api/calibration/session/{identifier}/export.zip")
    def export(identifier: str):
        path = attempt(lambda: service.archive(identifier))
        return FileResponse(path, media_type="application/zip", filename=f"2PyBot-calibration-{identifier}.zip")

    @app.get("/api/calibration/active")
    def active():
        try:
            identifier = json.loads((service.directory / "active.json").read_text())["id"]
            return json.loads((service.folder(identifier) / "intrinsics.json").read_text())
        except (OSError, ValueError, KeyError):
            return {"calibrated": False, "message": "No validated camera intrinsics yet"}
