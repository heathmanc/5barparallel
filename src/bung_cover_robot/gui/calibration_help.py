"""Operator help text for the Calibration tab (shown by the Help button)."""

from __future__ import annotations

HELP_HTML = """
<h2>Calibrating the camera to the robot</h2>
<p>Calibration builds a <b>planar homography</b> that turns a pixel in the overhead
camera image into a robot-frame position in millimetres. It is a flat,
plane-to-plane map: it is only valid at <b>one height</b> &mdash; the plane where
covers and holes actually sit &mdash; and only for the <b>current recipe</b>.
Re-calibrate after any changeover, camera bump, lens/focus change, or if the
fixture height changes.</p>

<h3>What you will do (step by step)</h3>
<ol>
  <li><b>Select the recipe</b> (battery type) you are calibrating. The result is
      saved per recipe.</li>
  <li><b>Place a target</b> with several known reference points on the work
      surface, at the exact height the parts are picked from. Do not let it sag or
      lift.</li>
  <li><b>Capture a frame.</b> Confirm the whole work area and every reference point
      is sharp and well lit &mdash; no glare, no shadow, no motion blur.</li>
  <li><b>Click a known point</b> in the image (zoom in first for precision), then
      <b>type that point's robot X and Y in millimetres</b> in the table.</li>
  <li>Repeat for <b>at least 4 points</b> &mdash; 6 to 9 is better &mdash; spread
      across the field of view.</li>
  <li><b>Fit the homography.</b> Read the <b>RMS residual</b> it reports.</li>
  <li>If the residual is acceptable, <b>Save</b>. The calibration is broadcast live
      to the Vision tab for the active recipe.</li>
</ol>

<h3>How to get robot XY for each point</h3>
<p>Two reliable methods:</p>
<ul>
  <li><b>Jog the robot to the point (best).</b> Move the TCP / a fine pointer down
      onto a mark you can also see in the image, read the controller's reported X/Y,
      and type it in. This calibrates against the robot's <i>own</i> reported
      coordinates, which is exactly what you want &mdash; it folds any small robot
      offset into the map.</li>
  <li><b>Known fixture geometry.</b> Use a machined plate or printed grid whose
      feature positions in robot coordinates are known (from a datum corner and
      measured spacing). Faster, but only as accurate as your knowledge of where the
      plate sits in the robot frame.</li>
</ul>

<h3>Best targets and tools</h3>
<ul>
  <li><b>A machined dowel/hole fixture</b> at a known datum &mdash; most accurate and
      repeatable; the machining tolerance is your floor.</li>
  <li><b>A printed dot or checkerboard grid</b> on rigid, flat stock (foam-board or
      aluminium &mdash; <i>not</i> loose paper, which curls and adds parallax error).
      Cheap and good if kept dead flat and taped down.</li>
  <li><b>A fine-tip pointer / calibration pin on the TCP</b> for the jog-to-point
      method &mdash; the sharper the tip, the tighter the residual.</li>
  <li><b>High-contrast fiducials</b> (dark rings/crosses on a light field) read
      cleanly on a mono camera. Click the geometric <b>centre</b> of each mark, and
      click the same feature type every time.</li>
</ul>

<h3>Where to place the points</h3>
<ul>
  <li><b>Cover the whole field of view.</b> Put points in all <b>four corners plus
      the centre</b>, then fill in a few mid-edge points. Accuracy is only
      trustworthy <i>inside</i> the convex hull of your points &mdash; a map fit only
      near the middle degrades badly at the edges.</li>
  <li><b>Match the working area.</b> Concentrate points over the region where covers
      and holes actually appear during a run.</li>
  <li><b>Stay at the working-plane height (parallax).</b> Every point must be at the
      same height as the picked parts. A target sitting even a few millimetres too
      high or too low shifts its apparent pixel position under the overhead lens and
      will bias the whole map. This is the single most common source of error.</li>
  <li><b>Keep points non-collinear and well spread.</b> Four points in a line
      &mdash; or bunched in one corner &mdash; give a degenerate or unstable fit. Aim
      for a wide quadrilateral with points in between.</li>
  <li><b>Use 6 to 9 points.</b> Four is the minimum and lets one bad click wreck the
      fit with no way to detect it. Extra points average out click noise and make the
      residual meaningful.</li>
</ul>

<h3>Reading the RMS residual</h3>
<p>After fitting, the tool reports the <b>RMS residual in millimetres</b> &mdash; the
average distance between where each point's robot coordinates <i>are</i> and where
the fitted map <i>predicts</i> they are. Lower is better.</p>
<ul>
  <li><b>Under 1 mm &mdash; good.</b> Ready to save for a normal pick task.</li>
  <li><b>1 to 3 mm &mdash; marginal.</b> Usable only if it beats your placement
      tolerance. Usually means an imprecise click, a slightly off typed coordinate,
      or a target not quite flat/level.</li>
  <li><b>Over 3 mm &mdash; reject and redo.</b> Suspect a wrong robot XY entry, a
      point at the wrong height, a mislabelled or transposed X/Y, or points too
      collinear/clustered.</li>
</ul>
<p><b>Residual is a floor, not a guarantee.</b> It only measures fit at the points
you gave. A low residual from points bunched in the centre still hides large error
at the edges &mdash; which is why full-field coverage matters. To truly verify, click
or jog a <i>fresh</i> point you did <i>not</i> use in the fit and check the predicted
mm against reality.</p>
<p>If one point has a large residual, it is usually a single bad click or typo
&mdash; remove that row and re-fit rather than accepting a poor overall number. Lens
distortion is corrected before the fit, so on this full-resolution sensor edge points
are handled correctly; a stubborn edge-only error points to parallax (wrong height)
instead.</p>

<h3>Zoom &amp; precise picking</h3>
<p>Use the mouse wheel to zoom about the cursor, drag to pan, and the on-cursor
magnifier loupe to place the reticle on the exact pixel. Buttons: <b>+ / &minus; /
Fit / 1:1</b>. Keyboard: <code>+ &minus; 0 1</code> and arrow keys.</p>
"""
