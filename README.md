# WingBuilder (Fusion 360 Add-In)


Overview
--------
WingBuilder is a Fusion 360 add-in that generates a parametric wing (and optional winglet)
from airfoil .dat files. You choose span, chord distribution, sweep/dihedral/twist, and airfoil stations,
and the script builds the wing as lofted geometry with optional guide rails for clean leading/trailing edges.

This tool is aimed at RC / UAV wings, wing prototypes, and “quick iteration” concept work where you want
consistent geometry without hand-drawing every station.


Included Airfoil Database
-------------------------
This file set includes an airfoil.dat database that’s ready to use with the add-in and contains ~1600
airfoil .dat files. If you’re new to airfoils, this makes it easy to browse and test common airfoils
without hunting them down online.


What This Add-In Creates
------------------------
Depending on options you enable, the add-in can generate:

- A multi-station wing (semi-span or full span) with automatic station spacing
- Kink-aware planforms (inner/outer break for sweep/dihedral)
- Twist about a selectable reference axis (e.g., quarter-chord)
- Airfoil blending between root / mid / tip .dat files
- Thickness and camber scaling along the span (optional)
- Tip treatment:
  - Flat tip
  - Pointed tip (cap / near-zero chord style)
- Optional winglet:
  - Height / cant / sweep
  - Tip chord ratio
  - Optional use of rails for the winglet
  - Smooth transition from main wing to winglet

It also prints helpful planform outputs (MAC, AR, area, sweep at LE/¼/TE, etc.) to Fusion’s text console.


Requirements
------------
- Autodesk Fusion 360 (Personal is fine; some advanced behaviors depend on Fusion loft/rail limits)
- Python Add-In support (Fusion’s built-in API environment)


Installation
------------
1) Put the add-in folder in your Fusion Add-Ins directory, typically:
   Windows:
   C:\Users\<You>\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\AddIns\

2) In Fusion 360:
   Utilities -> Add-Ins -> Scripts and Add-Ins
   Find the add-in (e.g., WingBuilder) and click Run
   Optionally set it to Run on Startup


Quick Start (5 minutes)
-----------------------
If you don’t know what to pick yet, these values generally produce a “reasonable wing”:

1) Span
   - Use Semi-span if you want the add-in to mirror for you.
   - Start with 250–400 mm semi-span for RC scale experiments.

2) Stations
   - 7–11 stations is a good start.
   - Use Auto (cosine / kink-aware) spacing so more stations cluster near root/tip.

3) Chord
   - Start with a simple taper:
     Root: 120 mm
     Tip: 60 mm
   - Add a Mid station if you want a “kink” or a more complex distribution.

4) Angles
   - Sweep: 10–25° is typical for mild swept wings
   - Dihedral: 2–4° for stability (0° for flat wings)
   - Twist: try something small like 0° at root -> -2° at tip

5) Airfoils
   - Choose a root airfoil and tip airfoil (or reuse the same one everywhere).
   - If you’re using the included database, just browse/select from it.

6) Build
   - Run/Build and inspect the resulting body.


Key Concepts (Plain English)
----------------------------


  A “station” is a cross-section slice of the wing at some span position.
  More stations = the loft follows your intended shape more closely.


  Chord is the “front-to-back” length of the airfoil at a station.
  A wing that tapers has a smaller chord at the tip than at the root.


  Sweep moves the wing backward (or forward) along the span.
  The tool supports kinked sweep (different inner/outer angles with a break point).


  Dihedral tilts the wing upward as it goes outboard. This often improves roll stability.


  Twist rotates each station’s airfoil around an axis (often quarter-chord).
  Typical use: washout (tip has less angle than root) to soften tip stall.

  MAC (Mean Aerodynamic Chord)  The script prints MAC and its location, useful for CG and aerodynamic calculations.


Parameter Guide
---------------

Span & Mirroring
- Semi-span + Mirror: builds half the wing and mirrors it.
- Full span: generates the full wing directly.

Station Spacing
- Auto / cosine: more stations near root and tip (usually best)
- Kink-aware: if you have sweep/dihedral breaks, it keeps stations around the break so the kink stays sharp

Chord Distribution
- Piecewise linear (root-mid-tip): easy, predictable taper or “kinked taper”
- Other distributions (if present in your UI): useful for elliptical-ish planforms or custom shaping

Sweep / Dihedral Distributions
- Constant: one value from root to tip
- Kinked: inner value -> break point -> outer value
  Great for wing planforms that change sweep or dihedral partway out.

Twist
- Often best applied around quarter-chord (0.25) to keep leading edge behavior sane.
- A typical RC starting point: Root 0° -> Tip -2° to -4°

Airfoil Processing
- Resample N: number of points per airfoil outline. Higher = smoother but heavier.
- Flip Z: flips airfoil vertical axis if the .dat orientation is opposite what Fusion expects.
- Thickness / Camber scaling: “morph” airfoil characteristics along the span without changing the base .dat.

Tip Treatment
- Flat: simplest and most robust.
- Pointed / cap: creates a closing tip by shrinking the chord near the end.

Note:
  Extremely thin, near-zero chord tips can stress loft solvers (especially with guide rails).
  The script includes logic to avoid unstable loft conditions by adjusting loft/rails strategy when needed.


Winglets
--------
Enable winglet options to add a vertical (or canted) winglet at the tip.

Common controls:
- Height: how tall the winglet is
- Cant: 90° is vertical; <90° leans outward/inward
- Winglet sweep: backward sweep of the winglet planform
- Tip chord ratio: tip chord relative to winglet root chord
- Tip twist delta: additional twist applied across the winglet
- Use rails: can improve edge quality but may be sensitive in extreme shapes

Alignment note:
  The winglet root is constructed to align cleanly to the main wing tip to avoid tiny offset seams.


Guide Rails (What They Do)
--------------------------
Rails are curves (typically along leading and trailing edges) that the loft uses as guidance.
They often improve:
- Leading edge smoothness
- Trailing edge smoothness
- Twist and sweep tracking through the loft

However, rails make the loft more constrained. In extreme geometry (very pointy tips, aggressive sweep changes,
very thin chord caps), rails can cause loft failures. If that happens, the script may automatically fall back to
a more robust loft strategy.


Outputs You’ll See (Console)
----------------------------
After a build, the tool prints useful metrics like:
- Total span
- Area
- Aspect ratio (AR)
- Taper ratio
- MAC and its location
- Sweep at leading edge, quarter-chord, trailing edge

These are helpful for quick performance sanity checks and for setting CG targets.


Tips for New Users
------------------
- Start with Flat tip, moderate sweep, and mild twist.
- Don’t go straight to near-zero chord tips until your base wing is working reliably.
- If your wing looks “inside out”:
  - Toggle Flip Z
  - Verify your .dat file orientation (many .dat files differ in sign conventions)
- If loft fails:
  - Reduce twist magnitude
  - Reduce sweep breaks or make them less extreme
  - Increase station count
  - Try disabling rails (temporarily) to confirm the planform is valid


Troubleshooting
---------------

“Loft would intersect itself” / Loft Compute Failed
  Usually means the geometry got too constrained or folded.
  Common causes: aggressive sweep + thin tip chord + pointed cap + rails.

  Try:
  - Flat tip
  - More stations
  - Less extreme sweep/dihedral breaks
  - Reduce twist
  - Disable rails (temporarily) to validate the base shape

Winglet looks “thicker” at the end
  Often a viewing effect caused by section plane orientation and the cap face.
  Use Section Analysis and measure thickness if you want to confirm.


Suggested Workflow
------------------
1) Build a clean main wing (no winglet).
2) Add winglet with conservative settings (shorter height, mild sweep).
3) Iterate:
   - tweak twist
   - tweak sweep breaks
   - tweak tip treatment
   - test rails on/off for edge quality


File Structure (Typical)
------------------------
- WingBuilder.py (main add-in)
- WingBuilder.manifest
- commands/ (UI + command code)
- lib/ (helpers)
- airfoil.dat/ (included airfoil database, ~1600 .dat files)

(Your actual folder names may vary slightly depending on how you packaged it.)


Beginner Presets (Copy-These-First)
-----------------------------------
These presets are meant to get a clean, buildable wing fast. Once one works, then start pushing geometry
(higher sweep, more twist, pointed caps, winglets, etc.)

Notes:
- If you’re using Semi-span + Mirror, use the “Semi-span” values below.
- Stations: 7–11 is a sweet spot.
- Tip treatment: start with Flat until everything looks correct.

Preset A — “Trainer / Stable Wing” (easy to fly, forgiving)
- Semi-span: 300 mm (mirror on -> 600 mm total)
- Stations: 7 (Auto / kink-aware cosine)
- Chord: Root 140 mm / Mid 120 mm / Tip 80 mm  (or Root 140 / Tip 80)
- Sweep: Constant 5–10°
- Dihedral: Constant 3–5°
- Twist: Root 0° / Tip -2° (quarter-chord axis)
- Airfoils: same airfoil root->tip (simple), or slightly thinner at tip
- Tip: Flat
- Rails: On (if it builds)
Why it works: mild sweep + decent dihedral + mild washout tends to behave nicely.

Preset B — “Sport / General Purpose” (faster, still tame)
- Semi-span: 300–400 mm
- Stations: 9
- Chord: Root 130 / Mid 110 / Tip 60–70
- Sweep: Kinked 10° inner -> 20° outer (break ~0.25)
- Dihedral: 2–3°
- Twist: Root 0° / Mid -1° / Tip -3°
- Tip: Flat (or pointed once stable)
- Rails: On
Why it works: adds sweep without going full “dart,” maintains stall friendliness via washout.

Preset C — “Fast Swept Wing” (speed-focused)
- Semi-span: 300–450 mm
- Stations: 11
- Chord: Root 120 / Mid 95 / Tip 45–55
- Sweep: Kinked 15° inner -> 35° outer (break ~0.20)
- Dihedral: 0–2°
- Twist: Root +1° / Mid 0° / Tip -2°  (keep twist modest)
- Tip: Flat first; pointed later if you want the look
- Rails: On, but if loft errors appear -> try Off to validate shape
Why it works: sweep + moderate taper gives speed efficiency; restrained twist avoids rail/loft folding.

Preset D — “STOL-ish / Slow Wing” (lower stall, fatter wing)
- Semi-span: 250–350 mm
- Stations: 7–9
- Chord: Root 160 / Mid 140 / Tip 110 (low taper)
- Sweep: 0–5°
- Dihedral: 3–6°
- Twist: Root 0° / Tip -1° (don’t overdo)
- Airfoils: thicker, more cambered at root
- Tip: Flat
- Rails: On
Why it works: low sweep + low taper + thicker sections = slow, stable, forgiving.

Preset E — “Winglet Starter” (clean transition, low drama)
Start from Preset A or B and add:
- Winglet Enable: Yes
- Height: 30–60 mm
- Cant: 80–90° (near vertical)
- Winglet sweep: 15–35°
- Tip chord ratio: 0.6–0.8
- Tip twist delta: 0° initially
- Use rails: Yes (if it builds; otherwise Off and re-test)
Why it works: moderate winglet geometry tends to loft cleanly and gives you a baseline to iterate.


How to Pick an Airfoil (RC/UAV Cheat Sheet)
-------------------------------------------
Airfoil choice matters, but don’t let it stop you. For most projects, you can get 80% of the result
by choosing something reasonable and setting twist + taper sensibly.

The 3 airfoil traits that matter most
1) Thickness (%)
   - Thicker = stiffer wing, more internal volume, tends to be more forgiving
   - Thinner = less drag at speed, but can be twitchier and structurally harder

2) Camber (%)
   - More camber = more lift at low speed (often better slow flight)
   - Less camber = better for speed and inverted / aerobatics

3) Leading edge shape
   - Rounder LE = gentler stall
   - Sharper LE = can be efficient, but stall more abruptly

Practical starting guidelines
- Trainer / slow cruise: moderately thick + moderate camber
- Sport wing: medium thickness + moderate camber
- Fast wing: thinner + lower camber (but don’t go razor-thin unless structure is solid)
- Winglet: usually fine with the tip airfoil or a slightly thinner variant

Reynolds number (don’t panic — just be aware)
Small RC wings operate at low Reynolds numbers compared to full-scale aircraft.
Some airfoils that look great at high Reynolds can behave poorly when small.

Rule of thumb:
- If your wing is small and slow, prefer airfoils known to behave well at RC-ish Reynolds numbers.
- If you’re unsure, use the included database and test a few common “RC friendly” shapes.

Use the tool’s 3-station airfoil blending to your advantage
You can do:
- Root: thicker / more cambered for lift + structure
- Mid: your main “character” airfoil
- Tip: thinner / less cambered to reduce tip stall tendencies (especially with washout)

A common beginner-friendly pattern:
- Root: slightly thicker
- Tip: slightly thinner + a bit less camber
...and add washout (negative twist toward tip).

Don’t try to “airfoil your way out” of geometry issues
If a wing stalls nasty or feels unstable, it’s often:
- Too little washout (or even wash-in)
- Too aggressive taper
- Too much sweep without compensating design choices
- CG placement and tail volume (outside this script)
Airfoil helps, but twist + planform often matter more.

Quick “good default” approach
- Pick one airfoil from the included database and use it for root/mid/tip.
- Set twist to 0° root -> -2° to -4° tip (washout).
- Build, inspect, iterate.
Then experiment with:
- thinner tip airfoil
- slightly reduced camber at tip
- small changes first


Bonus: “Good Practices” for Predictable Wings
---------------------------------------------
- Avoid extreme taper unless you really know what you’re chasing. Very tiny tip chords can cause tip stall and loft issues.
- Washout is your friend (negative twist toward tip).
- Kinked sweep/dihedral is powerful — but large angle jumps can stress lofts and aerodynamics.
- Validate in steps: build main wing -> confirm -> add winglet -> confirm -> try pointed cap.
