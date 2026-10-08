# Servo map

Bioloid Premium humanoid (Type A) layout. Odd IDs are on the robot's right side,
even IDs on its left. The same table, with directions and limits, lives in
[`config/humanoid_type_a.json`](../config/humanoid_type_a.json).

| ID | Joint | ID | Joint |
|---:|---|---:|---|
| 1 | Right shoulder pitch | 2 | Left shoulder pitch |
| 3 | Right shoulder roll | 4 | Left shoulder roll |
| 5 | Right elbow | 6 | Left elbow |
| 7 | Right hip yaw | 8 | Left hip yaw |
| 9 | Right hip roll | 10 | Left hip roll |
| 11 | Right hip pitch | 12 | Left hip pitch |
| 13 | Right knee | 14 | Left knee |
| 15 | Right ankle pitch | 16 | Left ankle pitch |
| 17 | Right ankle roll | 18 | Left ankle roll |

## Conventions used in the code

- **Angles** are degrees from the servo's centre (raw position 512). The AX-12A covers ±150°, about 0.29° per step.
- **Mirroring:** left and right servos are mounted as mirror images, so the same physical movement needs opposite raw values. Each joint has a `direction` (+1/−1) so a symmetric pose uses the same numbers on both sides, e.g. `r_knee: 30, l_knee: 30`.
- **Trim:** `trim_deg` corrects small offsets from assembly (a horn mounted one spline off, for example).
- **Limits:** every command is clamped to `min_deg`/`max_deg` before it reaches the servo.
