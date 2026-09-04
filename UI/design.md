# UI Design System — Balancing Robot WebUI

> Extracted from `UI/css/*.css` + `UI/index.html`. Single source of truth for visual patterns.
> Theme origin: Stitch NEBULA VOID export (`UI/stitch_store_llm_store_page_redesign.zip`, reference only, untracked).

---

## 1. Design Language

**Aesthetic**: NEBULA VOID military terminal — pure black base, bone-white text, hairline gray borders, single blood-red action accent, teal reserved for live/connected status. Zero rounded corners, zero shadows, zero blur. Flat data plates, not floating cards.

**Motion**: Linear 0.2s width transitions on sidebar. No bounce, no glow pulses (field tool, not a storefront).

---

## 2. Color Palette

All colors are CSS custom properties in `:root` (`UI/css/theme.css:3-15`).

| Token | Value | Usage |
|---|---|---|
| `--bg` | `#000000` | Page background, deepest layer |
| `--surface` | `#0A0A0A` | Cards, panels, sidebar hover |
| `--border` | `#2A2A2A` | All hairline borders and dividers |
| `--ghost-border` | `#3A3A3A` | Ghost button outlines, input borders |
| `--text` | `#F2F2F2` | Headings, values, primary text |
| `--muted` | `#8A8A8A` | Labels, hints, secondary text |
| `--red` | `#CC0000` | Primary actions, ARM/armed, active nav |
| `--red-hot` | `#FF0A0A` | Critical highlights, ARM hover, brand span |
| `--teal` | `#00E5CC` | Connected status, live telemetry values, console text |

**Rules**:
- Red = something you can tap to do something, or where you are. Never decorative.
- Teal = system-is-live (connected pill, telemetry numbers, console). Never competes with red in the same element.
- Disconnected/idle = muted gray ghost. No color until it matters.
- Global `* { border-radius: 0 !important; }` (`theme.css:17`). Square corners everywhere.

---

## 3. Typography

Loaded in `UI/index.html` head from Google Fonts with system fallback.

| Font | Stack | Usage |
|---|---|---|
| Display | `'Oswald', 'Anton', 'Arial Narrow', sans-serif` (`--font-display`) | h1/h2/h3, brand, sidebar labels. Uppercase, letter-spacing 0.08em |
| Mono | `'JetBrains Mono', 'Space Mono', monospace` (`--font-mono`) | Body default, labels, telemetry, buttons, inputs, console. Uppercase, letter-spacing 0.12–0.16em, 11–13px |

Hierarchy is size + weight + tracking, not color. Labels are muted 11px mono tracked out; values are larger white or teal.

---

## 4. Spacing & Layout

**Desktop**: left icon rail + top header bar + scrollable content column (`.app` flex row, `theme.css:34-37`).
**Mobile** (`@media max-width: 768px`): sidebar becomes a fixed bottom tab bar (row, red top indicator on active), panels stack single-column, PID grid collapses to 1 column, header shrinks to 12px, content pads above the tab bar with safe-area insets.
**Touch targets**: min 44px height on all buttons, selects, inputs, nav items (mobile only, `theme.css:115-121`).
**Card padding**: 14px. Section gap: 12px. Max content width: unconstrained (tool UI, fills viewport).

---

## 5. Components

### 5.1 Sidebar (48px collapsed / 260px expanded)
- `UI/css/sidebar.css:2-13` — Left rail, pure black, 1px right hairline, always visible on desktop.
- Hamburger toggle row (`14px 12px` padding), display font, uppercase.
- Nav items: full-width, `10px 12px` padding, 3px transparent left bar, muted 12px mono 700 uppercase labels with 24px icon slot.
- Hover: surface bg + white text. Active: surface bg + red-hot text + red left bar (`sidebar.css:59-63`).
- Mobile: same markup becomes fixed bottom bar via CSS only — row direction, icon-over-label, red top indicator, toggle/spacer hidden (`theme.css:123-166`).

### 5.2 Header data plate
- `UI/css/panels.css:1-21` — Flat black bar, bottom hairline, brand left (display 15px, red span on second word), status cluster right.
- Connection pill: ghost outline; connected = teal text/border, disconnected = muted gray (`panels.css:33-35`).
- ARM button: disarmed = ghost (muted), armed = solid red fill white text, hover red-hot (`panels.css:37-41`).
- Battery indicator: black plate, hairline border, mono 11px uppercase (`panels.css:56-69`).

### 5.3 Buttons
- Primary (`button.control-btn`): solid `#CC0000` fill, white 12px mono 700 uppercase, 0.16em tracking, `8px 14px` padding. Hover: red-hot. Disabled: ghost + muted (`theme.css:56-70`).
- Secondary/ghost (`.secondary`, `#fullscreenBtn`, `.console-clear-btn`): transparent fill, 1px ghost border, muted text. Hover: white text + white border.
- Range inputs: red accent-color, full width (`theme.css:54`).

### 5.4 Telemetry data plates
- `.card`: surface bg, 1px border, 14px padding (`panels.css:71-77`). Card titles display 14px white uppercase; subtitles muted mono 12px.
- `.telemetry-item`: hairline bottom divider, muted uppercase micro-label, white `<strong>`, teal live `<span>` values (`panels.css:106-110`).
- Vehicle selector row: flex wrap, muted mono label + ghost select (`panels.css:95-104`).

### 5.5 Joystick instrument
- `UI/css/stick.css` — `#joystick1` black plate, 1px border, crosshair cursor, 280px desktop / 240px mobile height, full-width `100dvh`-derived height on small phones (`stick.css:39-47`).
- Text-select bug fix baked in: `user-select: none`, `-webkit-touch-callout: none`, `touch-action: none`, `overscroll-behavior: none` on wrap + canvas; JS uses preventDefault + `{ passive: false }` + touchcancel + contextmenu block (`UI/js/stick.js`).
- Readout line: centered muted mono uppercase, teal values (`stick.css:24-34`).

### 5.6 Console
- `#serialConsole`: black plate, hairline border, 180px scroll region, 11px mono teal text (`panels.css:125-136`).

### 5.7 Graph + gauges
- `#angleChart`: full width, 300px desktop / 260px mobile. Legend muted mono uppercase (`panels.css:152-165`).
- Attitude/compass canvases: black square plates, hairline border, wrap in flex row that stacks on mobile (`panels.css:167-170`).

### 5.8 Forms (PID / trim / settings)
- 3-column PID grid (1 column on mobile). Inputs: black fill, ghost border, mono 12px uppercase; focus border turns red (`theme.css:43-53`).

---

## 6. File Map

| File | Lines | Owns |
|---|---|---|
| `UI/index.html` | ~250 | Shell markup, sidebar/header/panels, css/js/manifest links, font links |
| `UI/css/theme.css` | ~167 | Tokens, reset, buttons, inputs, error bar, mobile tab-bar + touch rules |
| `UI/css/sidebar.css` | ~80 | Rail, toggle, nav items, active states |
| `UI/css/panels.css` | ~186 | Header, pills, ARM, battery, cards, telemetry, PID, console, graph, gauges |
| `UI/css/stick.css` | ~47 | Joystick plate, readout, phone sizing |
| `UI/js/state.js` | ~84 | State object, vehicle select + persistence, settings |
| `UI/js/link.js` | ~193 | WebSocket connect/reconnect, message parse, telemetry render, console |
| `UI/js/stick.js` | ~112 | Stick canvas, per-vehicle mapping, select-bug fix listeners |
| `UI/js/panels.js` | ~385 | Panel switching, PID/trim/calibration actions, gauges, battery, ARM, fullscreen |
| `UI/manifest.webmanifest` | ~10 | Install metadata: fullscreen, landscape |
| `UI/quadcopter.html` | legacy | Untouched fallback single-file build |

Rules: plain `<script>` tags in dependency order (no modules, works over file://). Element ids are the JS contract — rename in both or neither. WS message shapes must match `firmware/balancing_robot/src/comms/websocket_handler.cpp`.
