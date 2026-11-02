"""Writes tools/ots-overlay/ui/icons/*.svg.

Most icons are Lucide (https://lucide.dev, ISC license); basestation, tracker, controller and hmd are
drawn in the same style because Lucide has no VR hardware. Strokes are white so the UI can tint them
with the RCSS `image-color` property.
"""
import pathlib

LUCIDE = {
    "activity": ["M22 12h-4l-3 9L9 3l-3 9H2"],
    "map": ["M14.106 5.553a2 2 0 0 0 1.788 0l3.659-1.83A1 1 0 0 1 21 4.619v12.764a1 1 0 0 1-.553.894l-4.553 2.277a2 2 0 0 1-1.788 0l-4.212-2.106a2 2 0 0 0-1.788 0l-3.659 1.83A1 1 0 0 1 3 19.381V6.618a1 1 0 0 1 .553-.894l4.553-2.277a2 2 0 0 1 1.788 0z",
            "M15 5.764v15", "M9 3.236v15"],
    "list": ["M3 12h.01", "M3 18h.01", "M3 6h.01", "M8 12h13", "M8 18h13", "M8 6h13"],
    "list-filter": ["M3 6h18", "M7 12h10", "M10 18h4"],
    "rotate-ccw": ["M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8", "M3 3v5h5"],
    "pencil": ["M21.174 6.812a1 1 0 0 0-3.986-3.987L3.842 16.174a2 2 0 0 0-.5.83l-1.321 4.352a.5.5 0 0 0 .623.622l4.353-1.32a2 2 0 0 0 .83-.497z",
               "m15 5 4 4"],
    "check": ["M20 6 9 17l-5-5"],
    "move": ["M12 2v20", "m15 19-3 3-3-3", "m19 9 3 3-3 3", "M2 12h20", "m5 9-3 3 3 3", "m9 5 3-3 3 3"],
    "rotate-3d": ["M16.466 7.5C15.643 4.237 13.952 2 12 2 9.239 2 7 6.477 7 12s2.239 10 5 10c.342 0 .677-.069 1-.2",
                  "m15.194 13.707 3.814 1.86-1.86 3.814",
                  "M19 15.57c-1.804.885-4.274 1.43-7 1.43-5.523 0-10-2.239-10-5s4.477-5 10-5c4.838 0 8.873 1.718 9.8 4"],
    "chevron-right": ["m9 18 6-6-6-6"],
    "sliders-horizontal": ["M21 4h-7", "M10 4H3", "M21 12h-9", "M8 12H3", "M21 20h-5", "M12 20H3",
                           "M14 2v4", "M8 10v4", "M16 18v4"],
    "triangle-alert": ["m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3", "M12 9v4", "M12 17h.01"],
    "flag": ["M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z", "M4 22v-7"],
    "languages": ["m5 8 6 6", "m4 14 6-6 2-3", "M2 5h12", "M7 2h1", "m22 22-5-10-5 10", "M14 18h6"],
    "github": ["M15 22v-4a4.8 4.8 0 0 0-1-3.5c3 0 6-2 6-5.5.08-1.25-.27-2.48-1-3.5.28-1.15.28-2.35 0-3.5 0 0-1 0-3 1.5-2.64-.5-5.36-.5-8 0C6 2 5 2 5 2c-.3 1.15-.3 2.35 0 3.5A5.403 5.403 0 0 0 4 9c0 3.5 3 5.5 6 5.5-.39.49-.68 1.05-.85 1.65-.17.6-.22 1.23-.15 1.85v4",
               "M9 18c-4.51 2-5-2-7-2"],
    "settings": ["M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.08a2 2 0 0 1-1-1.74v-.5a2 2 0 0 1 1-1.74l.15-.09a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z"],
}
SHAPES = {
    "settings": ['<circle cx="12" cy="12" r="3"/>'],
    "battery": ['<rect x="2" y="7" width="16" height="10" rx="2" ry="2"/>', '<path d="M22 11v2"/>'],
    # Lucide "battery-charging"
    "battery-charging": ['<path d="M15 7h1a2 2 0 0 1 2 2v6a2 2 0 0 1-2 2h-2"/>', '<path d="M6 7H4a2 2 0 0 0-2 2v6a2 2 0 0 0 2 2h1"/>',
                         '<path d="m11 7-3 5h4l-3 5"/>', '<path d="M22 11v2"/>'],
    "monitor": ['<rect x="2" y="3" width="20" height="14" rx="2"/>', '<path d="M8 21h8"/>', '<path d="M12 17v4"/>'],
    "info": ['<circle cx="12" cy="12" r="10"/>', '<path d="M12 16v-4"/>', '<path d="M12 8h.01"/>'],
    "circle-dot": ['<circle cx="12" cy="12" r="10"/>', '<circle cx="12" cy="12" r="4" fill="#ffffff"/>'],
    "square": ['<rect x="5" y="5" width="14" height="14" rx="2" fill="#ffffff"/>'],
    "palette": ['<path d="M12 22a1 1 0 0 1 0-20 10 9 0 0 1 10 9 5 5 0 0 1-5 5h-2.25a1.75 1.75 0 0 0-1.4 2.8l.3.4a1.75 1.75 0 0 1-1.4 2.8z"/>',
                '<circle cx="13.5" cy="6.5" r=".5" fill="#ffffff"/>', '<circle cx="17.5" cy="10.5" r=".5" fill="#ffffff"/>',
                '<circle cx="6.5" cy="12.5" r=".5" fill="#ffffff"/>', '<circle cx="8.5" cy="7.5" r=".5" fill="#ffffff"/>'],
    # Lucide "box" (3D view)
    "box": ['<path d="M21 8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16Z"/>',
            '<path d="m3.3 7 8.7 5 8.7-5"/>', '<path d="M12 22V12"/>'],
    # Map views, Lucide style: floor seen from above / a standing figure seen from the side
    "view-top": ['<rect x="3" y="3" width="18" height="18" rx="2"/>', '<circle cx="12" cy="12" r="2.5"/>'],
    "view-side": ['<path d="M3 21h18"/>', '<circle cx="12" cy="6" r="2.5"/>', '<path d="M12 10.5V21"/>'],
    # VR hardware, Lucide style
    "basestation": ['<rect x="5" y="4" width="14" height="16" rx="3"/>'],
    "tracker": ['<circle cx="12" cy="12" r="8"/>', '<circle cx="12" cy="12" r="3"/>'],
    "controller": ['<path d="M9 3h6l-1 9h-4z"/>', '<path d="M10 12v7a2 2 0 0 0 4 0v-7"/>'],
    "hmd": ['<path d="M3 8a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2v7a2 2 0 0 1-2 2h-4l-2-2.5h-2L9 17H5a2 2 0 0 1-2-2z"/>'],
}

HEAD = ('<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" '
        'stroke="#ffffff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">')


def main():
    out = pathlib.Path(__file__).resolve().parent.parent / "tools" / "ots-overlay" / "ui" / "icons"
    out.mkdir(parents=True, exist_ok=True)
    for name in sorted(set(LUCIDE) | set(SHAPES)):
        body = "".join(f'<path d="{d}"/>' for d in LUCIDE.get(name, [])) + "".join(SHAPES.get(name, []))
        (out / f"{name}.svg").write_text(HEAD + body + "</svg>\n", encoding="utf-8")
    (out / "LICENSE.txt").write_text(
        "Icons from Lucide (https://lucide.dev), ISC License.\n"
        "Copyright (c) for portions of Lucide are held by Cole Bemis 2013-2022 as part of Feather (MIT).\n"
        "All other copyright (c) for Lucide are held by Lucide Contributors 2022.\n\n"
        "Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is\n"
        "hereby granted, provided that the above copyright notice and this permission notice appear in all copies.\n\n"
        "THE SOFTWARE IS PROVIDED \"AS IS\" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE.\n\n"
        "basestation.svg, tracker.svg, controller.svg, hmd.svg, view-top.svg and view-side.svg are original to this project.\n",
        encoding="utf-8")
    print(f"wrote {len(set(LUCIDE) | set(SHAPES))} icons to {out}")


if __name__ == "__main__":
    main()
