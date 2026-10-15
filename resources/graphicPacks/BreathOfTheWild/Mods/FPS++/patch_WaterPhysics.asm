[BotW_WaterPhysics_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; turn quadratic scaling of timesteps into mostly linear scaling by setting one of the two timestep variables closer to 1.0
; this is the more correct behavior as the applied velocities also scale linearly with timestep, but not exactly due to external factors like friction (probably)
; 
; fixes objects not decelerating fast enough when falling into water
; fixes object drifting speed when pushed by water currents

_constFloat1:
.float 1.0

_constTimestepDampingFactor:
.float 0.45 ; damping factor    0.0 -> linear scaling by timestep, 1.0 -> quadratic scaling by timestep

_constInverse30:
.float 1.0 / 30.0

0x034a804c = bla _water_physics_timestep_fix

; input:
;    f30 <-- Timestep30 (Timestep scale factor where 30 FPS is 1.0. 60 FPS is ~0.5, 90 FPS is ~0.333)
;    f1  <-- TimescaleProperty (a property from the object, usually 1.0)
; output:
;   f30  --> Scaled timestep. Normally only f30 * f1 but we introduce a damping factor:
;                             f30 = ((f30 - 1.0) * _constTimestepDampingFactor + 1.0) * f1
; available registers:
;   f12, r28
_water_physics_timestep_fix:
; replaced instruction: fmr f30, f1
; scale f30 to be close to 1/30 FPS
; - 1.0
lis r28, _constFloat1@ha
lfs f12, _constFloat1@lo(r28)
fsubs f30, f30, f12
; * _constTimestepDampingFactor
lis r28, _constTimestepDampingFactor@ha
lfs f12, _constTimestepDampingFactor@lo(r28)
fmuls f30, f30, f12
; + 1.0
lis r28, _constFloat1@ha
lfs f12, _constFloat1@lo(r28)
fadds f30, f30, f12
; * f1 (property from object)
fmuls f30, f30, f1
blr

