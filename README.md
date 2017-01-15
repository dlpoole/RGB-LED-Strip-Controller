# RGB-LED-Strip-Controller
Select desired illumination via an IR remote using either an HSB or RGB color model. 
<html>
<body lang="en-US" dir="ltr">
<p><b>D.L. Poole</b></p>
<p><b>January 2017 </b>
</p>
<p><b>This controller
</b>arose from unsuccessful attempts to purchase an off-the-shelf RGB
LED Strip Controller that could reach a wide, fine-grained range of
colors. It allows selecting or editing a desired illumination via an
infrared remote using either a Hue/Saturation/Brightness (HSB) or
Red/Green/Blue (RGB) color model interchangeably.  The outputs are
three 8-bit PWMs for MOSFETs or some other LED drivers.</p>
<p>With
reference to the accompanying remote keypad overlay, t</span>he
</span><b>Remote key functions</b> are as follows: 
</p>
<ul>
<li/>
	<b>HUE</b>: Select or modify a hue, moving clockwise or
	counterclockwise around a color circle</p>
	<li/>
<p><b>BRIGHT</b>:
	Raise total red+green+blue value without change of hue</p>
	<li/>
<p><b>DIM</b>:
	Lower total red+green+blue value without change of hue</p>
	<li/>
<p><b>DESAT</b>:
	Desaturate the selected hue towards white (center of the color
	circle)</p>
	<li/>
<p><b>RESAT</b>:
	Resaturate towards a saturated primary or secondary (Red==0 ||
	Green==0 || Blue==0)</p>
	<li/>
<p><b>RESET</b>:
	Set a pure, saturated blue at 100% brightness (Blue = 255)</p>
	<li/>
<p><b>VARY</b>:
	Start varying HUE around color circle or slow ongoing rate of
	variation by half</p>
	<li/>
<p><b>SAT</b>:
	Return to the selected, fully saturated hue at max brightness;
	cancel variable hue</p>
	<li/>
<p><b>+RED</b>:
	Increase the Red component and change HSB accordingly</p>
	<li/>
<p><b>+GREEN</b>:
	Increase the Green component and adjust HSB accordingly</p>
	<li/>
<p><b>+BLUE</b>:
	Increase the Blue component and adjust HSB accordingly</p>
	<li/>
<p><b>-RED</b>:
	Decrease the Red component and adjust HSB accordingly</p>
	<li/>
<p><b>-GREEN</b>:
	Decrease the Green component and adjust HSB accordingly</p>
	<li/>
<p><b>-BLUE</b>:
	Decrease the Blue component and adjust HSB accordingly</p>
	<li/>
<p><b>WARM</b>:
	Simulate a tungsten bulb at max brightness</p>
	<li/>
<p><b>BRIGHT</b>:
	Simulate a halogen bulb at max brightness</p>
	<li/>
<p><b>DAYLIGHT</b>:
	Simulate a daylight source at max brightness</p>
</ul>
<p>A <b>single keypress</b>
performs a function or produces a single step in HUE, BRIGHTNESS, or
SATURATION. Pressing, then holding a key for a half-second repeats
the key’s function ten times per second. One can release the button
at an appropriate value then jog forward or backward one step at a
time to reach a final preference. Changes to HUE loop around the
color circle indefinitely; when a brightness or saturation has
reached a limit and further key presses can accomplish nothing, a
blink occurs instead.</p>
<p><b>Edits to either
HSB or RGB</b> values are immediately transformed to the other model
and output to the LEDs so one can freely move between color models.
RGB edits to full ON (white) or full OFF leave hue undefined by the
RGB values alone. At such endpoints, conversions to HSB preserve hue
so that the user can resaturate a white back to its starting hue.</p>
<p >
<b>Color Models</b></p>
<p >
Again with reference to the diagram on the remote overlay, the HSB
color space lies around a unit circle of saturated primary (RGB) and
secondary (CMY) hues along which hue is a floating point variable 0.0
&lt;= hue &lt;=1.0  Using a clock metaphor where saturated blue lies
at 12:00, red at 4:00, and green at 8:00, the three primaries are
linearly interpolated two at a time so that magenta lies at 2:00,
yellow at 6:00, and cyan at 10:00. Along the hue circle, the SUM of
RGB values (and the corresponding PWM outputs, red + green + blue
equals 255, with one primary always zero.</p>
<p>RGB values are
maintained as floats for ease of interpolation. They are rounded to
the nearest integer before a write to the PWM pins, so they
extinguish below bright = 1/254.5 = 0.004 and turn full ON above
254.5 Edits to red, green, blue, and brightness are performed in
logarithmic steps rather than increments so as to produce visually
similar steps at both ends of the range. The last few steps to
blackout pass through some 8-bit underflow steps at which the LEDs
don’t change.</p>
<p>Small hue changes in
saturated colors are more readily perceived between red and green and
less perceptible around blue. The choice of step size must be large
enough to conveniently traverse the hue circle at a 10/sec repeat
rate of a held key, yet small enough to appear continuous. This is
accomplished by halving the step size between red and green. 
</p>
<p>The addition of any
third primary desaturates a saturated hue towards white, which lies
at the figurative center of the clock and at which hue is undefined.
The radial dimension of any point in HSB space is represented by the
floating point variable sat where 0.0 &lt;= sat &lt;=1.0 Thus, sat is
1.0 along the saturated hue circle and outside it and zero at its
center.</p>
<p>Brightness is
represented by the floating point variable 0.0 &lt;= bright &lt;=1.0
which is here defined as (red+green+blue)/255/3. Brightness is thus a
constant 1/3 along the saturated hue circle. Reducing bright dims the
display and extinguishes it as bright approaches zero. Brightness
will “max out” a saturated primary at bright = 1/3, the two
primaries of a secondary color at bright = 2/3, and the three
primaries of a white at bright = 1. When any RGB component reaches
255, a further keypress will be ignored so as not to cause a hue
shift, though a blink will occur to acknowledge it. 
</p>
<p>Although the polar
coordinates of this HSB space suggests trigonometric definitions for
the primaries around the hue circle, linear interpolation (triangular
ramps) produced a much smoother range of hues. 
</p>
<p><b>Hardware</b></p>
<p>This controller consists of an Arduino UNO, a Vishay TSOP34438 IR
Receiver, and an Adafruit Mini Remote Control (Product ID 389.) The
original implementation was done on a stacked protoboard with the IR
receive, three power MOSFETS, and terminal blocks for +12V input and
the LED outputs. The MOSFETS were salvaged from one of the
unsuccessful attempts to purchase an off-the-shelf controller, but
were sufficient to drive the six feet of LED strip needed with 5V
gate drive directly from the UNO pins. A scratch implementation might
be done with the International Rectifier IRLIZ44NPBF N-channel power
MOSFET. These FETs should be good to about 8 Amperes per channel at
100% ON without a heat sink or higher voltage gate driver, however,
this author hasn’t actually tried that configuration, so YOYO and
YRMV. 
</p>
<p>The <b>Hardware connections</b> are as follows: 
</p>
<ul>
	<li/>
<p>D2: Input
	from Vishay TSOP34438 38KHz IR Receiver</p>
</ul>
<ul>
	<li/>
<p>PWM 9: Red
	PWM Pin; High = ON</p>
	<li/>
<p>PWM 10: Green
	PWM Pin; High = ON</p>
	<li/>
<p>PWM 11: Blue
	PWM Pin; High = ON</p>
</ul>
<p><b>The sketch
</b>requires the IRLib2 library which can be downloaded from
<a href="https://github.com/cyborg5/IRLib2">https://github.com/cyborg5/IRLib2</a>
For the Vishay TSOP34438 IR Receiver and Adafruit Mini Remote Control
(Product ID 389,) which generates NEC codes at 38KHz, decoding was
most reliable using the library’s IRrecvPCI receiver with the
enableAutoResume method and markExcess = 0 Use of the PCI receiver
also avoids conflict with the PWM on pin D11 for TIMER2.</p>
<p><b>Remote Key
Assignments </b>are tabulated
</span>constants for the NEC key codes of the Adafruit Mini Remote.
The table can be modified for other codes or remotes, though a change
to the repeat logic may be necessary. 
</p>
<p><b>A keypad overlay
</b>for the Mini Remote is available in the GitHub Repo in several
formats. If printed at scale on self adhesive (label) paper, the
overlay can be optionally laminated with clear polyester tape then
applied over the face of the Adafruit Mini Remote. 
</p>
<p>The
</span><b>Variable Hue </b>key
begins</span> continuous cycling
the hue at the then-current</span> brightness
and saturation. Subsequent keypresses slow the cycling by a factor of
two. The hue can be dimmed, brightened, desaturated, or resaturated
while cycling; the SAT key or any RGB edit stops the cycling at the
then-current hue. </span>
</p>
<p><br/>

</p>
<p align="center" >
End-of-Document</p>
</body>
</html>
