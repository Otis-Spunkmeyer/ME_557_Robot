#!/usr/bin/env python3
"""
Letter Coordinate Maker
------------------------
Draw letter strokes on a grid matching the real robot board.

HOW TO USE
  1. Click anywhere on the canvas to start a chain.
  2. Each subsequent click extends the chain by one more segment.
     All segments within a chain are CONTINUE (pen stays down) by default.
  3. Press "End Chain" to finish the chain.
     The last segment of the chain is marked as LIFT (pen lifts, travel follows).
  4. Click anywhere again to start the next chain.
  5. Press "Done - Save" when finished.

Segment colours:
  Orange = pen stays down after this segment (continue)
  Cyan   = pen lifts after this segment (travel)

Output: letterCoordinates.json in the same directory as this script.
"""

import json
import os
import tkinter as tk
from tkinter import messagebox

# ── Board / workspace limits ─────────────────────────────────────────────────
LAT_MIN   = -7.0
LAT_MAX   =  7.0
Z_MIN     =  0.0    # inches  0 = 1" above board bottom edge (skips border)
Z_MAX     = 12.0    # inches  arm max reach at corners (world z ≈ 14")
SNAP_GRID =  0.25
BORDER_IN =  1.0    # 1" no-draw safety margin around all edges

SAFE_LAT_MIN = LAT_MIN + BORDER_IN
SAFE_LAT_MAX = LAT_MAX - BORDER_IN
SAFE_Z_MIN   = Z_MIN   + BORDER_IN
SAFE_Z_MAX   = Z_MAX   - BORDER_IN

# ── Canvas geometry ──────────────────────────────────────────────────────────
PX_PER_IN = 40
MARGIN    = 65

DRAW_W = int((LAT_MAX - LAT_MIN) * PX_PER_IN)   # 560 px  (14" × 40 px/in)
DRAW_H = int((Z_MAX   - Z_MIN)   * PX_PER_IN)   # 480 px  (12" × 40 px/in)
CANVAS_W = DRAW_W + 2 * MARGIN
CANVAS_H = DRAW_H + 2 * MARGIN

# ── Colours ──────────────────────────────────────────────────────────────────
COL_BG        = "#1e1e2e"
COL_BOARD     = "#2a2a3e"
COL_GRID_MAIN = "#44445a"
COL_GRID_SUB  = "#303045"
COL_AXIS      = "#888899"
COL_LABEL     = "#aaaacc"

COL_SEG_CONT  = "#ff9900"   # segment: continue (pen stays down)
COL_SEG_LIFT  = "#00ccff"   # segment: lift/travel after

COL_PT_START  = "#00ff88"   # chain start dot
COL_PT_MID    = "#ff9900"   # mid-chain junction dot (continue colour)
COL_PT_END    = "#00ccff"   # chain end dot (lift colour)
COL_PT_PEND   = "#ffff00"   # pending first-point dot (awaiting second click)
COL_HOVER     = "#ffffff"

OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "letterCoordinates.json")


# ── Coordinate helpers ────────────────────────────────────────────────────────

def lat_to_cx(lat):
    return MARGIN + (lat - LAT_MIN) / (LAT_MAX - LAT_MIN) * DRAW_W

def z_to_cy(z):
    return MARGIN + DRAW_H - (z - Z_MIN) / (Z_MAX - Z_MIN) * DRAW_H

def cx_to_lat(cx):
    return LAT_MIN + (cx - MARGIN) / DRAW_W * (LAT_MAX - LAT_MIN)

def cy_to_z(cy):
    return Z_MIN + (MARGIN + DRAW_H - cy) / DRAW_H * (Z_MAX - Z_MIN)

def snap(value, grid=SNAP_GRID):
    return round(round(value / grid) * grid, 3)

def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# ── Main GUI ──────────────────────────────────────────────────────────────────

class LetterCoordinateMaker:
    def __init__(self, root):
        self.root = root
        self.root.title("Letter Coordinate Maker")
        self.root.configure(bg=COL_BG)
        self.root.resizable(False, False)

        # ── Data model ──────────────────────────────────────────────────────
        # self.segments: list of {start:[lat,z], end:[lat,z], lift_after:bool}
        # self.chain_tail: [lat, z] of the last-placed point in the active chain,
        #                  or None when no chain is active.
        #                  Set as soon as the FIRST click of a new chain lands,
        #                  even before any segment exists.
        self.segments   = []
        self.chain_tail = None   # None = no active chain

        self.hover_lat = 0.0
        self.hover_z   = 0.0
        self.snap_on   = True

        self._build_ui()
        self._draw_grid()
        self._refresh()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.canvas = tk.Canvas(
            self.root, width=CANVAS_W, height=CANVAS_H,
            bg=COL_BG, highlightthickness=0, cursor="crosshair",
        )
        self.canvas.pack(side=tk.LEFT, padx=(10, 0), pady=10)
        self.canvas.bind("<Motion>",   self._on_mouse_move)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Leave>",    self._on_leave)

        panel = tk.Frame(self.root, bg=COL_BG, width=220)
        panel.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        panel.pack_propagate(False)

        tk.Label(panel, text="LETTER COORDINATE\nMAKER",
                 bg=COL_BG, fg="#00ccff",
                 font=("Courier", 12, "bold"), justify=tk.CENTER).pack(pady=(10, 5))

        # ── Legend ───────────────────────────────────────────────────────────
        legend = tk.Frame(panel, bg=COL_BG)
        legend.pack(pady=(0, 10))
        items = [
            ("---", COL_SEG_CONT, "Continue (pen down)"),
            ("---", COL_SEG_LIFT, "Travel   (pen lifts)"),
        ]
        for row, (sym, col, txt) in enumerate(items):
            tk.Label(legend, text=sym, bg=COL_BG, fg=col,
                     font=("Courier", 11)).grid(row=row, column=0, padx=(0, 4))
            tk.Label(legend, text=txt, bg=COL_BG, fg=COL_LABEL,
                     font=("Courier", 9)).grid(row=row, column=1, sticky="w")

        # ── Status / coord readout ────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Click the canvas\nto start a chain.")
        tk.Label(panel, textvariable=self.status_var,
                 bg=COL_BG, fg=COL_HOVER,
                 font=("Courier", 10), wraplength=205,
                 justify=tk.CENTER).pack(pady=(0, 6))

        self.coord_var = tk.StringVar(value='lat:  0.0"\nz:    0.0"')
        tk.Label(panel, textvariable=self.coord_var,
                 bg=COL_BG, fg=COL_LABEL,
                 font=("Courier", 10), justify=tk.LEFT).pack(pady=(0, 10))

        # Snap toggle
        self.snap_var = tk.BooleanVar(value=True)
        tk.Checkbutton(panel, text='Snap to 0.25" grid',
                       variable=self.snap_var,
                       bg=COL_BG, fg=COL_LABEL,
                       selectcolor="#333355",
                       activebackground=COL_BG,
                       font=("Courier", 9),
                       command=lambda: setattr(self, "snap_on", self.snap_var.get())
                       ).pack(pady=(0, 10))

        # ── Action buttons ───────────────────────────────────────────────────
        self._btn_end = tk.Button(
            panel, text="[■] End Chain",
            bg="#332200", fg="#ffaa00",
            activebackground="#554400", activeforeground="#ffcc00",
            font=("Courier", 11, "bold"), relief=tk.FLAT,
            width=20, height=2,
            command=self._cmd_end_chain,
        )
        self._btn_end.pack(pady=4)

        tk.Button(
            panel, text="[<] Undo Last",
            bg="#222222", fg="#aaaaaa",
            activebackground="#444444", activeforeground="#cccccc",
            font=("Courier", 10), relief=tk.FLAT,
            width=20, height=2,
            command=self._cmd_undo,
        ).pack(pady=4)

        tk.Button(
            panel, text="[X] Clear All",
            bg="#330000", fg="#ff6666",
            activebackground="#550000", activeforeground="#ff8888",
            font=("Courier", 10), relief=tk.FLAT,
            width=20, height=2,
            command=self._cmd_clear,
        ).pack(pady=4)

        tk.Frame(panel, bg=COL_GRID_MAIN, height=2).pack(fill=tk.X, pady=10)

        tk.Button(
            panel, text="[OK] DONE - Save",
            bg="#003300", fg="#00ff88",
            activebackground="#005500", activeforeground="#44ffaa",
            font=("Courier", 12, "bold"), relief=tk.FLAT,
            width=20, height=2,
            command=self._cmd_done,
        ).pack(pady=4)

        tk.Label(panel, text="Segments:", bg=COL_BG, fg=COL_LABEL,
                 font=("Courier", 9, "bold")).pack(pady=(12, 2))
        self.seg_list_var = tk.StringVar(value="(none)")
        tk.Label(panel, textvariable=self.seg_list_var,
                 bg=COL_BG, fg=COL_LABEL,
                 font=("Courier", 8), justify=tk.LEFT,
                 wraplength=210).pack()

    # ── Static grid ───────────────────────────────────────────────────────────

    def _draw_grid(self):
        c = self.canvas
        c.create_rectangle(MARGIN, MARGIN,
                            MARGIN + DRAW_W, MARGIN + DRAW_H,
                            fill=COL_BOARD, outline="")

        lat = LAT_MIN
        while lat <= LAT_MAX + 1e-9:
            x = lat_to_cx(lat)
            col = COL_GRID_MAIN if abs(lat % 1.0) < 0.01 else COL_GRID_SUB
            c.create_line(x, MARGIN, x, MARGIN + DRAW_H, fill=col, width=1)
            lat = round(lat + 0.5, 3)

        z = Z_MIN
        while z <= Z_MAX + 1e-9:
            y = z_to_cy(z)
            col = COL_GRID_MAIN if abs(z % 1.0) < 0.01 else COL_GRID_SUB
            c.create_line(MARGIN, y, MARGIN + DRAW_W, y, fill=col, width=1)
            z = round(z + 0.25, 3)

        for i in range(-7, 8):
            x = lat_to_cx(float(i))
            c.create_text(x, MARGIN + DRAW_H + 15,
                          text=str(i), fill=COL_LABEL, font=("Courier", 8))
        c.create_text(MARGIN + DRAW_W // 2, MARGIN + DRAW_H + 32,
                      text="<-- lateral (inches, 0=center) -->",
                      fill=COL_LABEL, font=("Courier", 9))

        for i in range(0, int(Z_MAX) + 2):
            if float(i) > Z_MAX + 1e-9:
                break
            y = z_to_cy(float(i))
            label = '0" bottom' if i == 0 else f'{i}"'
            c.create_text(MARGIN - 28, y,
                          text=label, fill=COL_LABEL, font=("Courier", 8))

        ref_y5 = z_to_cy(5.0)
        c.create_line(MARGIN, ref_y5, MARGIN + DRAW_W, ref_y5,
                      fill="#555588", width=1, dash=(6, 4))
        c.create_text(MARGIN + DRAW_W - 4, ref_y5 - 8,
                      text='5" (letter top)', fill="#555588",
                      font=("Courier", 8), anchor="e")

        c.create_line(MARGIN, z_to_cy(0.0), MARGIN + DRAW_W, z_to_cy(0.0),
                      fill="#445544", width=2)

        c.create_text(14, MARGIN + DRAW_H // 2,
                      text="height (inches)\n0 = board bottom",
                      fill=COL_LABEL, font=("Courier", 8), angle=90,
                      justify=tk.CENTER)

        c.create_line(lat_to_cx(0.0), MARGIN, lat_to_cx(0.0), MARGIN + DRAW_H,
                      fill="#555577", width=1, dash=(4, 4))

        c.create_rectangle(MARGIN, MARGIN,
                            MARGIN + DRAW_W, MARGIN + DRAW_H,
                            outline=COL_AXIS, width=1)

        # 1" safety margin boundary
        bx1 = lat_to_cx(SAFE_LAT_MIN)
        bx2 = lat_to_cx(SAFE_LAT_MAX)
        by1 = z_to_cy(SAFE_Z_MAX)
        by2 = z_to_cy(SAFE_Z_MIN)
        c.create_rectangle(bx1, by1, bx2, by2,
                           outline="#ff6633", width=1, dash=(6, 3))
        c.create_text(bx1 + 4, by1 + 10,
                      text="1\" margin", fill="#ff6633",
                      font=("Courier", 7), anchor="nw")

    # ── Dynamic refresh ───────────────────────────────────────────────────────

    def _refresh(self):
        c = self.canvas
        c.delete("dynamic")

        # Draw all committed segments
        for i, seg in enumerate(self.segments):
            s, e = seg["start"], seg["end"]
            lift = seg["lift_after"]
            col  = COL_SEG_LIFT if lift else COL_SEG_CONT
            sx, sy = lat_to_cx(s[0]), z_to_cy(s[1])
            ex, ey = lat_to_cx(e[0]), z_to_cy(e[1])

            c.create_line(sx, sy, ex, ey, fill=col, width=2, tags="dynamic")

            # Index label near midpoint
            c.create_text((sx+ex)/2 + 6, (sy+ey)/2 - 8, text=str(i + 1),
                          fill=col, font=("Courier", 7, "bold"), tags="dynamic")

            # Start dot: green on first point of each chain, orange otherwise
            # A chain starts when the previous segment had lift_after=True (or i==0)
            is_chain_start = (i == 0) or self.segments[i - 1]["lift_after"]
            start_col = COL_PT_START if is_chain_start else COL_PT_MID
            c.create_oval(sx-5, sy-5, sx+5, sy+5,
                          fill=start_col, outline="", tags="dynamic")

            # End dot: cyan if lift, orange if continue
            end_col = COL_PT_END if lift else COL_PT_MID
            c.create_oval(ex-5, ey-5, ex+5, ey+5,
                          fill=end_col, outline="", tags="dynamic")

        # Draw pending first-point (chain started but no segment yet)
        if self.chain_tail is not None:
            px, py = lat_to_cx(self.chain_tail[0]), z_to_cy(self.chain_tail[1])
            # Show as green if no segments exist yet, or pure yellow pending
            if not self.segments or self.segments[-1]["lift_after"]:
                dot_col = COL_PT_START
            else:
                dot_col = COL_PT_MID
            c.create_oval(px-6, py-6, px+6, py+6,
                          fill=dot_col, outline=COL_HOVER, width=1,
                          tags="dynamic")

            # Preview dashed line to cursor
            hx, hy = lat_to_cx(self.hover_lat), z_to_cy(self.hover_z)
            if self._in_board_px(hx, hy):
                c.create_line(px, py, hx, hy,
                              fill=COL_SEG_CONT, width=1, dash=(5, 3),
                              tags="dynamic")

        # Hover crosshair
        hx, hy = lat_to_cx(self.hover_lat), z_to_cy(self.hover_z)
        if self._in_board_px(hx, hy):
            r = 4
            c.create_oval(hx-r, hy-r, hx+r, hy+r,
                          fill=COL_HOVER, outline="", tags="dynamic")
            c.create_line(hx-10, hy, hx+10, hy,
                          fill=COL_HOVER, width=1, tags="dynamic")
            c.create_line(hx, hy-10, hx, hy+10,
                          fill=COL_HOVER, width=1, tags="dynamic")

    # ── Events ────────────────────────────────────────────────────────────────

    def _in_board_px(self, cx, cy):
        return MARGIN <= cx <= MARGIN + DRAW_W and MARGIN <= cy <= MARGIN + DRAW_H

    def _resolve(self, event):
        if not self._in_board_px(event.x, event.y):
            return None
        lat = clamp(cx_to_lat(event.x), SAFE_LAT_MIN, SAFE_LAT_MAX)
        z   = clamp(cy_to_z(event.y),   SAFE_Z_MIN,   SAFE_Z_MAX)
        if self.snap_on:
            lat, z = snap(lat), snap(z)
        return round(lat, 3), round(z, 3)

    def _on_mouse_move(self, event):
        pt = self._resolve(event)
        if pt:
            self.hover_lat, self.hover_z = pt
            self.coord_var.set(
                f"lat: {self.hover_lat:+.2f}\"\nz:   {self.hover_z:5.2f}\""
            )
        self._refresh()

    def _on_leave(self, event):
        self._refresh()

    def _on_click(self, event):
        pt = self._resolve(event)
        if not pt:
            return

        if self.chain_tail is None:
            # First click of a new chain — just record the tail, no segment yet
            self.chain_tail = list(pt)
            self.status_var.set(
                f"Chain started.\n({pt[0]:+.2f}\", {pt[1]:.2f}\")\n\n"
                "Click to add points.\nPress End Chain when done."
            )
        else:
            # Extend chain: create a new continue-segment from tail to pt
            self.segments.append({
                "start":      list(self.chain_tail),
                "end":        list(pt),
                "lift_after": False,
            })
            self.chain_tail = list(pt)
            n = len(self.segments)
            self.status_var.set(
                f"Segment {n} added.\n-> ({pt[0]:+.2f}\", {pt[1]:.2f}\")\n\n"
                "Click to continue.\nPress End Chain when done."
            )
            self._update_seg_list()

        self._refresh()

    # ── Buttons ───────────────────────────────────────────────────────────────

    def _cmd_end_chain(self):
        if self.chain_tail is None:
            self.status_var.set("No active chain.")
            return

        # Find the last segment of this chain (the one whose end == chain_tail)
        # and mark it lift_after=True.
        if self.segments and self.segments[-1]["end"] == self.chain_tail:
            self.segments[-1]["lift_after"] = True
            n = len(self.segments)
            self.status_var.set(
                f"Chain ended.\n{n} segment(s) total.\n\n"
                "Click canvas to\nstart a new chain."
            )
        else:
            # chain_tail was set but no segment was added yet (single orphan point)
            self.status_var.set(
                "Chain removed\n(no segments drawn).\n\n"
                "Click canvas to start."
            )

        self.chain_tail = None
        self._update_seg_list()
        self._refresh()

    def _cmd_undo(self):
        if self.chain_tail is not None:
            # We are mid-chain
            if self.segments and self.segments[-1]["end"] == self.chain_tail:
                # Step back one point: remove the last segment, restore tail
                removed = self.segments.pop()
                self.chain_tail = list(removed["start"])
                n = len(self.segments)
                self.status_var.set(
                    f"Undid last segment.\n{n} remain.\n\nContinue clicking."
                )
            else:
                # Only the orphan first-click exists
                self.chain_tail = None
                self.status_var.set("Removed chain start.\nClick to start over.")
        elif self.segments:
            # No active chain — undo the last committed segment.
            # If it was the start of a chain, just remove it.
            # Restore it as an active chain so the user can re-extend.
            removed = self.segments.pop()
            # Only re-activate the chain if the removed segment was a lift
            # (otherwise the previous segment's lift flag keeps things clean).
            if removed["lift_after"]:
                # The removed segment ended a chain; re-open it
                self.segments.append({**removed, "lift_after": False})
                self.chain_tail = list(removed["end"])
                self.status_var.set("Undo: chain re-opened.\nPress End Chain to close.")
            else:
                self.chain_tail = list(removed["start"])
                self.status_var.set("Undid last segment.\nChain re-opened.")
            self._update_seg_list()
            self._refresh()
            return
        else:
            self.status_var.set("Nothing to undo.")
        self._update_seg_list()
        self._refresh()

    def _cmd_clear(self):
        if (self.segments or self.chain_tail) and not messagebox.askyesno(
                "Clear all", "Remove all segments?"):
            return
        self.segments.clear()
        self.chain_tail = None
        self.status_var.set("All cleared.\nClick to start.")
        self._update_seg_list()
        self._refresh()

    def _cmd_done(self):
        if not self.segments:
            messagebox.showwarning("No segments", "Draw at least one segment first.")
            return
        data = {"segments": self.segments}
        with open(OUTPUT_FILE, "w") as f:
            json.dump(data, f, indent=2)
        messagebox.showinfo(
            "Saved!",
            f"Saved {len(self.segments)} segment(s) to:\n{OUTPUT_FILE}\n\n"
            "Trigger the robot to replay."
        )
        self.status_var.set(f"Saved {len(self.segments)} segment(s)!")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _update_seg_list(self):
        if not self.segments:
            self.seg_list_var.set("(none)")
            return
        lines = []
        for i, seg in enumerate(self.segments):
            s, e = seg["start"], seg["end"]
            icon = "[UP]" if seg["lift_after"] else "[->"
            lines.append(
                f"{i+1}{icon} ({s[0]:+.1f}\",{s[1]:.1f}\") ->\n"
                f"     ({e[0]:+.1f}\",{e[1]:.1f}\")"
            )
        self.seg_list_var.set("\n".join(lines))


def main():
    root = tk.Tk()
    LetterCoordinateMaker(root)
    root.mainloop()


if __name__ == "__main__":
    main()
