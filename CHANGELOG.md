# Changelog

All notable changes to **WingBuilder** will be documented in this file.

The format is based on *Keep a Changelog*, and this project aims to follow *Semantic Versioning*.

## [0.1.0] - 2026-02-13
### Added
- Fusion 360 add-in to generate multi-station wings from airfoil `.dat` files (root/mid/tip station blending).
- Planform controls: semi-span/full span, mirroring, station spacing (including kink-aware cosine spacing).
- Geometry controls: chord distribution, sweep (including kinked inner/outer), dihedral, twist about a selectable axis.
- Airfoil processing: resampling, optional Z flip, thickness/camber scaling distributions.
- Tip treatments: flat and pointed/cap styles.
- Optional winglet generation: height, cant, sweep, tip chord ratio, tip twist delta, optional rails.
- Guide-rail lofting support with robust fallback behavior for loft kernel edge cases.
- Console output of key planform metrics (area, AR, MAC and location, sweep at LE/¼/TE).

### Fixed
- Winglet orientation/handedness issues that could invert the winglet root section.
- Winglet root alignment improved so the first winglet section matches the main wing tip cleanly.
- Minor UI/exception logging robustness improvements (avoid undefined `ui` reference in exception handler).

### Changed
- Project renamed from previous internal names to **WingBuilder** for public release packaging.
