# `LKAnimNode_AnimVerlet` Tutorial and Parameter Reference

This document explains how to configure and tune the **AnimVerlet** skeletal-control node in an Unreal Engine Animation Blueprint. It is based on the current implementation of `FLKAnimNode_AnimVerlet`, its editor graph node, collision data types, and Blueprint collision helper functions.

> The example bone names in this document are illustrative. Replace them with names from your skeleton.

## 1. What the node does

`AnimVerlet` converts one or more skeletal bone hierarchies into particles and constraints, simulates them in component space, and writes the resulting position and rotation back to the real bones. Typical uses include:

- ponytails, braids, antennae, ears, and tails;
- straps, cords, chains, and hanging accessories;
- capes or other cloth-like bone grids made from several parallel chains;
- secondary motion that must collide with the character, the world, or itself.

The node supports two main working styles:

- **Animation-pose-following secondary motion**: the animation pose remains an active target. This is forgiving and usually best for hair, tails, and accessories.
- **Physics-oriented motion**: the animation pose is ignored, gravity and forces use squared delta time, and constraints define most of the shape. This is useful for cloth-like grids and physically motivated setups.

The simulated root of every chain is automatically pinned. Real bones below the root are free unless they are locked by a per-bone override or by the tip-lock option.

## 2. Before adding the node

Prepare the skeleton with these rules in mind:

1. A `VerletBones` entry starts at one `RootBone` and recursively visits all descendants.
2. A simple, unbranched hierarchy is the most predictable representation of one chain.
3. For a cape, use several parallel, similarly sized chains and add one `VerletBones` entry per chain.
4. For multi-chain cloth, enter the roots in their physical side-to-side order. Neighboring array entries are used to build side and diagonal constraints.
5. If different LODs remove bones used by the simulation, decide whether to enable `bRebuildSimulationOnLODChange`.

Example single chain:

```text
hair_root
└── hair_01
    └── hair_02
        └── hair_03
```

Example multi-chain cape:

```text
cape_left_root    cape_center_root    cape_right_root
└── ...           └── ...             └── ...
```

## 3. Add and connect the node

1. Open the character's Animation Blueprint.
2. In the **AnimGraph**, add an **AnimVerlet** node.
3. Connect the pose that should drive the simulation to the node's input pose.
4. Connect AnimVerlet to the next skeletal-control node or to the final animation pose.
5. Add at least one element to **Setup > Verlet Bones** and select its `RootBone`.
6. Compile the Animation Blueprint.

Placement in the AnimGraph matters. Put AnimVerlet after any animation or IK operation that should be treated as its input pose. Put an operation after AnimVerlet only when it should modify the already simulated bones.

The node also inherits standard skeletal-control properties such as the output alpha and LOD threshold from Unreal Engine's skeletal-control base node. Use the inherited alpha to blend the entire node on or off without changing its internal simulation parameters.

## 4. Quick-start tutorial: animation-driven ponytail

Assume the hierarchy is `hair_root -> hair_01 -> hair_02 -> hair_03`.

### Step 1: Define the chain

Add one `VerletBones` entry:

| Field | Value |
|---|---|
| `RootBone` | `hair_root` |
| `ExcludeBones` | Empty |
| `Mass` | `1.0` |
| `bFakeBone` | `false` |

The node automatically includes every descendant of `hair_root`. Do not add every bone as a separate `VerletBones` entry.

### Step 2: Start from the animation-pose preset

Set **Preset > Preset Type** to `AnimationPose`. The preset sets:

| Property | Preset value |
|---|---:|
| `bIgnoreAnimationPose` | `false` |
| `bUseXPBDSolver` | `false` |
| `bUseSquaredDeltaTime` | `false` |
| `bUseIsometricBendingConstraint` | `false` |
| `Damping` | `0.8` |
| `SolveIteration` | `2` |
| `Gravity` | `(0, 0, -9.8)` |

Changing a preset-related property changes the editor's preset selection back to `Custom`.

### Step 3: Establish shape and stability

Use these values as a starting point:

| Property | Suggested start | Purpose |
|---|---:|---|
| `bMakeFakeTipBone` | `true` | Gives the last real bone a simulated child from which to derive rotation. |
| `FakeTipBoneLength` | Approximately the final bone length | Prevents an under-defined tip rotation. |
| `bPreserveLengthFromParent` | `true` | Keeps each segment near its authored length. |
| `LengthFromParentMargin` | `0.1 cm` | Allows a small correction tolerance. |
| `ConeAngle` | `45–70 degrees` | Prevents the chain from folding through its root or parent direction. |
| `Thickness` | Match the visible hair volume | Defines simulated collision radius. |
| `SolveIteration` | `2–4` | Increase only if constraints or collisions remain visibly unresolved. |

### Step 4: Tune the response

- Increase `Damping` toward `1.0` to retain more motion and produce a longer swing.
- Decrease `Damping` to settle faster.
- Increase `AnimationPoseInertia` to pull particles more strongly toward the current animation-pose location.
- Increase `AnimationPoseDeltaInertia` to transmit more input-pose movement into the simulation.
- Use `AnimationPoseDeltaInertiaClampMax` to prevent an animation discontinuity from producing an extreme jump.
- Increase `ShapeMemoryForce` when the chain should actively return toward its authored curve.

### Step 5: Add body collision

Add a capsule to `CapsuleCollisionShapes`:

| Field | Example |
|---|---|
| `bUseAbsoluteWorldTransform` | `false` |
| `AttachedBone` | `spine_03` |
| `LocationOffset` | Adjust to the upper torso |
| `RotationOffset` | Align the capsule's local Z axis with the torso |
| `Radius` | `12 cm` |
| `HalfHeight` | `22 cm` |

Enable the capsule display in **Collision Input**, select the collider in the preview viewport, and use the transform gizmo to fit it. The values are offsets relative to the attached bone when `bUseAbsoluteWorldTransform` is false.

## 5. Tutorial: multi-chain cape using XPBD

Assume there are six unbranched cape chains with comparable bone counts, arranged from the character's left side to the right side across the cape's width.

### Step 1: Enter ordered roots

Add one `VerletBones` element for each chain, ordered across the cape from left to right:

```text
cape_l_03_root
cape_l_02_root
cape_l_01_root
cape_r_01_root
cape_r_02_root
cape_r_03_root
```

Array order is significant because the implementation creates side, diagonal, bending, and triangle relationships between neighboring entries. A cape is normally an open sheet, so the outermost left and right entries should remain separate. If the cape mesh contains an unusual closed or folded section, inspect the preview constraints and add custom constraints where an extra connection is required.

### Step 2: Select the physics preset

Choose `Physics_XPBD`. It enables squared delta time, XPBD, and isometric bending, and uses Unreal-scale gravity `(0, 0, -980)`.

Suggested starting values:

| Property | Suggested start |
|---|---:|
| `InvCompliance` | `100000000` |
| `InvBendingCompliance` | `10000` |
| `SolveIteration` | `4` |
| `bPreserveLengthFromParent` | `true` |
| `bPreserveSideLength` | `true` |
| `bConstrainRightDiagonalDistance` | `true` |
| `bConstrainLeftDiagonalDistance` | `true` |
| `bUseCapsuleCollisionForChain` | `true` |

With more than one configured chain, chain collision geometry becomes triangles when `bUseCapsuleCollisionForChain` is true. If the topology or chain lengths do not form a clean surface, use sphere collision instead or fix the bone layout.

### Step 3: Tune stretching and folding separately

- `InvCompliance` controls distance-constraint rigidity. Higher inverse compliance means less stretch.
- The bending inverse-compliance values control resistance to folding without directly replacing the distance stiffness.
- Enable `bUseBendingComplianceRange` to make the cloth softer near its authored angle and stronger when heavily folded.
- `BendingComplianceMaxAngle` is the fold angle at which `InvBendingComplianceMax` is fully applied.

### Step 4: Improve collision quality

1. Set `Thickness` to the desired particle/surface radius.
2. Add shoulder, upper-back, torso, and optional arm capsules, or use a collision data asset/physics asset.
3. Keep `bUseBroadphase` enabled.
4. Raise `SolveIteration` only after the topology and collider dimensions are correct.
5. Add friction gradually; excessive `FrictionCoefficient` can make the cape stick to the torso or arms.

### Step 5: Optional self-collision

Enable `bUseSelfCollision` only after ordinary constraints and body collision are stable.

- For a multi-chain surface, `bUseTriangleSelfCollision = false` uses sphere-versus-triangle checks.
- `bUseTriangleSelfCollision = true` uses triangle-versus-triangle checks and is more expensive.
- `SelfCollisionAdditionalThickness` expands the self-contact distance.

## 6. Bone-chain data structures

### `FLKAnimVerletBoneSetting`

Each `VerletBones` element contains:

| Property | Default | Explanation |
|---|---:|---|
| `RootBone` | None | First skeletal bone recursively included in this simulated hierarchy. The root particle is automatically pinned. |
| `ExcludeBones` | Empty | Bones skipped as simulated particles. Their descendants are still visited. |
| `BoneUnitSettingOverride` | Empty | Optional per-bone subdivision, lock, joint, thickness, collision-mode, and mass overrides. |
| `bStraightenExcludedBonesByParent` | `true` | Repositions excluded bones along the simulated parent/child direction instead of leaving only their original pose placement. |
| `bFakeBone` | `false` | Simulates an offset proxy chain based on these bones. This entry is primarily a virtual helper topology and does not directly output its marked particles as normal real-bone transforms. |
| `FakeBoneOffsetDir` | Forward `(1,0,0)` | Offset direction used when `bFakeBone` is enabled. It is normalized before use. |
| `FakeBoneOffsetSize` | `3 cm` | Distance of the virtual chain from the source bone transforms. |
| `Mass` | `1.0` | Default mass for particles in this entry. Force response is scaled by inverse mass. Minimum effective mass is `0.01`. |

### Excluding a bone without breaking the hierarchy

Suppose a chain contains a helper bone:

```text
strap_root -> strap_01 -> strap_twist_helper -> strap_02 -> strap_03
```

Add `strap_twist_helper` to `ExcludeBones`. The helper is not simulated as a particle, but traversal continues to `strap_02`. With `bStraightenExcludedBonesByParent` enabled, the excluded helper is reconstructed from surrounding simulated motion.

Exclusion is different from a collider's `ExcludeBones`: chain exclusion changes simulation topology, while collider exclusion only prevents selected simulated bones from contacting that collider.

### `FLKAnimVerletBoneUnitSetting`

Add an element to `BoneUnitSettingOverride`, select its `Bone`, then enable only the overrides required by that bone:

| Property | Default | Explanation |
|---|---:|---|
| `Bone` | None | Real skeletal bone to override. |
| `bOverrideSubDivideBones` | `false` | Overrides subdivision for every segment that starts at this bone and ends at one of its simulated children. On a terminal bone, this is also the only way to subdivide the automatic fake-tip segment. |
| `bSubDivideBones` | `false` | Effective only when the subdivision override is enabled. True inserts virtual particles on the owned segment; false explicitly prevents subdivision even if the node default is enabled. |
| `NumSubDividedBone` | `1` | Number of virtual particles inserted into each segment controlled by this override. Zero inserts none. |
| `bLockBone` | `false` | Pins this particle to its animation-pose location. |
| `LockMargin` | `0 cm` | Permitted distance from the pin target. Zero is an exact positional pin. |
| `bOverrideConstrainConeAngleFromParent` | `false` | Enables a local override of the global cone-reference mode. |
| `bConstrainConeAngleFromParent` | `false` | When overridden, uses the grandparent-to-parent direction as the cone reference instead of the animation-pose direction. |
| `bOverrideConeAngle` | `false` | Enables this bone's local cone limit. |
| `ConeAngle` | `0 degrees` | Local ball-socket cone angle, clamped to `0–90` degrees. A positive local value takes precedence over the global value. |
| `bOverrideConeAngleOffset` | `false` | Replaces the node's global cone-center rotation offset for this bone. It can be enabled independently of the local cone-angle override. |
| `ConeAngleOffset` | Zero Rotator | Local rotation applied to this bone's cone center direction. The rotation uses the parent particle (`BoneA`) animation-pose local space. |
| `bOverrideThickness` | `false` | Enables a local collision-radius override. |
| `Thickness` | `0.3 cm` | Local particle, capsule, or surface thickness. |
| `bOverrideToUseSphereCollisionForChain` | `false` | Uses sphere contact for this unit when the node is otherwise using capsule/triangle chain collision. It has no effect if global chain-capsule collision is disabled. |
| `bOverrideMass` | `false` | Enables local mass. |
| `Mass` | `1.0` | Local mass, with a minimum effective value of `0.01`. |

Useful applications:

- add collision resolution only to one long segment;
- disable subdivision on a short or performance-sensitive segment while the node default remains enabled;
- divide an automatic fake tip into several shorter collision segments;
- lock an intermediate attachment point;
- make a broad tip lighter or heavier;
- enlarge collision only around a hair ornament;
- replace a problematic capsule/triangle segment with a sphere;
- tighten the cone limit near the root while leaving the tip loose;
- bias an individual cone backward or sideways without changing the authored skeleton pose.

## 7. Setup parameters

| Property | Default | How to use it |
|---|---:|---|
| `VerletBones` | Empty | Ordered list of simulated root hierarchies. One entry is a single-chain setup; multiple entries enable cloth-like side relationships. |
| `bSubDivideBones` | `false` | Default subdivision setting for real parent-child segments. A parent bone's unit setting can override it for the segments owned by that parent. |
| `NumSubDividedBone` | `1` | Default number of virtual particles inserted per subdivided segment. More particles increase cost and may require additional solve iterations. |
| `bRebuildSimulationOnLODChange` | `false` | Rebuilds topology, constraints, broadphase, and collision state when the required-bone LOD changes. Matching particle state is preserved where possible. Enable it when simulated bones differ between LODs. |
| `bActivate` | `true` | Completely disables node evaluation when false. Exposed as a default graph pin. |
| `bSkipUpdateOnDedicatedServer` | `true` | Skips evaluation on a dedicated server. Disable only if server-side simulated transforms are genuinely required. |
| `bPause` | `false` | Stops integration while retaining and outputting the current simulation state. It does not perform a reset, but it clears any pending fixed-step backlog. Exposed as a default graph pin. |
| `PlaySpeedRate` | `1.0` | Scales update delta time and time dilation. `0` effectively freezes time after clamping behavior; prefer `bPause` for an explicit pause. |
| `bUseWarmup` | `true` | After initialization or `ResetPhysics`, performs fixed substeps before exposing the result. |
| `WarmupStepCount` | `8` | Number of warmup substeps. Warmup runs on the next evaluated, unpaused frame. |
| `WarmupFixedDeltaTime` | `0.016666667 s` | Delta time for each warmup substep, approximately 60 Hz by default. |
| `OutputBlendDuration` | `0.2 s` | Blend duration from input animation pose to simulation after initialization/reset. Zero applies the result immediately. |
| `bMakeFakeTipBone` | `true` | Adds a virtual child after each leaf so the last real bone gets meaningful rotation and tip collision. |
| `FakeTipBoneLength` | `10 cm` | Length of the virtual tip segment. Match the visual continuation of the final real bone. |
| `bLockTipBone` | `false` | Pins real leaf particles. Enabling it prevents fake-tip creation. |
| `TipBoneLockMargin` | `0 cm` | Allowed movement radius for pinned tips. |
| `StartBoneLockMargin` | `0 cm` | Allowed movement radius for every automatically pinned chain root. |

### Per-segment subdivision workflow

Subdivision ownership is parent-based. A `BoneUnitSettingOverride` for `hair_01` controls the segment from `hair_01` to each of its simulated children; it does not control the segment from `hair_root` to `hair_01`.

For ordinary real-bone segments, the decision order is:

1. Start with the node-level `bSubDivideBones` and `NumSubDividedBone` defaults.
2. Look for a unit setting on the segment's simulated parent.
3. If that setting enables `bOverrideSubDivideBones`, replace both defaults with its local `bSubDivideBones` and `NumSubDividedBone` values.

This allows a global baseline with localized exceptions. For example, leave global subdivision off, add `cape_center_02` to `BoneUnitSettingOverride`, enable both `bOverrideSubDivideBones` and its local `bSubDivideBones`, and set `NumSubDividedBone = 2`. Only segments that start at `cape_center_02` receive two intermediate particles.

Automatic fake tips deliberately keep the legacy one-segment behavior regardless of the global subdivision setting. To subdivide a fake tip, add the terminal real bone to `BoneUnitSettingOverride` and explicitly enable its subdivision override. If the terminal override uses `NumSubDividedBone = 2`, the full `FakeTipBoneLength` is split into three equal segments: two intermediate virtual particles plus the final fake-tip particle.

For a multi-chain cape, keep subdivision overrides consistent across corresponding left-to-right chains unless an asymmetric topology is intentional. Side, diagonal, bending, and triangle relationships are created by chain order and particle depth; mismatched subdivision counts can connect non-corresponding rows or leave an uneven surface.

The `bPreserveLengthFromParentBetweenRealBones` and `bPreserveSideLengthBetweenRealBones` options detect the actual inserted particles, so they also work when subdivision is enabled only through a bone-unit override.

### Activate, pause, reset, and warmup are different

- **Deactivate**: the skeletal control does not evaluate.
- **Pause**: the node continues to output its retained state but does not integrate.
- **Reset Simulation** in the editor: clears motion and the fixed-step accumulator, then synchronizes particles with the current pose.
- **Reset Physics** from Unreal's animation dynamics flow: schedules a reset against the current pose/component transform and clears the fixed-step accumulator when that reset is performed. If warmup is enabled, warmup occurs on the following evaluated frame.

Warmup deliberately ignores component movement and rotation inertia by using the same component transform for every warmup substep. The input pose is prepared again between warmup steps so the pose history advances consistently instead of reusing one stale preparation.

When the result is applied, `OutputBlendDuration` blends from the **current** incoming component-space pose, not from a pose cached by an earlier simulation step. The simulated location and rotation are combined with the current input-pose scale. This keeps animated scale changes current even on a rendered frame where the fixed-step accumulator schedules no simulation step.

## 8. Animation-pose settings

These parameters are active only when `bIgnoreAnimationPose` is false unless stated otherwise.

| Property | Default | How to use it |
|---|---:|---|
| `AnimationPoseDeltaInertia` | `0.03` | Fraction of per-frame input-pose movement transferred into a particle. |
| `AnimationPoseDeltaInertiaScale` | `1.0` | Additional multiplier for pose-delta transfer. |
| `bClampAnimationPoseDeltaInertia` | `true` | Limits the transferred pose delta. Recommended for teleports, animation jumps, and montage transitions. |
| `AnimationPoseDeltaInertiaClampMax` | `0.1` | Maximum transferred pose-delta displacement per update when clamping is enabled. |
| `bIgnoreAnimationPose` | `false` | Disables pose-following corrections and leaves shape primarily to constraints and forces. |
| `bAlignAnimationPoseToGravity` | `false` | Rotates the authored pose from `AnimationPoseReferenceDirection` toward current gravity before using it as the pose target. |
| `AnimationPoseInertia` | `0.03` | Direct pull toward the current parent-relative animation-pose location. Higher values follow animation more tightly. |
| `bApplyAnimationPoseInertiaCorrection` | `true` | Scales `AnimationPoseInertia` using its target frame rate. The denominator is clamped average FPS in variable-step mode, or `DeltaTimeCorrectionTargetFrameRate` while corrected fixed-step simulation is active. |
| `AnimationPoseInertiaTargetFrameRate` | `60` | Reference frame rate for pose-inertia correction. |
| `AnimationPoseReferenceDirection` | Down `(0,0,-1)` | Component-space direction representing gravity in the authored animation pose. Used by the gravity-alignment options. |

Gravity alignment is useful when an asset was authored hanging in one component-space direction but the character can rotate so that world gravity points elsewhere. A zero reference vector or zero gravity disables the alignment rotation.

## 9. Solver and constraint parameters

### Core solver

| Property | Default | How to use it |
|---|---:|---|
| `Damping` | `0.9` | Fraction of previous Verlet displacement retained. `1` retains all; lower values remove motion faster. |
| `bApplyDampingCorrection` | `false` | Adjusts damping for more consistent decay. It uses clamped average FPS in variable-step mode and `DeltaTimeCorrectionTargetFrameRate` while corrected fixed-step simulation is active. |
| `DampingCorrectionTargetFrameRate` | `60` | Reference FPS for damping correction. |
| `bUseXPBDSolver` | `false` | Selects XPBD constraint solving. False selects PBD. |
| `InvCompliance` | `100000000` | XPBD inverse compliance. Higher values mean a more rigid distance constraint; actual compliance is `1 / InvCompliance`. Do not set it to zero. |
| `Stiffness` | `0.8` | PBD distance stiffness. Used when XPBD is disabled. |
| `SolveIteration` | `4` | Constraint passes per simulation update. Higher values improve convergence and increase cost. Minimum is `1`. |
| `bUseSquaredDeltaTime` | `false` | Applies gravity and force terms with `dt²` instead of `dt`, producing the physics-oriented integration mode used by the physics presets. |

`InvCompliance`, `Stiffness`, solver type, and topology are captured when simulation constraints are built. In the editor, changing node properties clears and rebuilds the preview simulation. At runtime, treat topology/solver changes as requiring a dynamics reset or node reinitialization.

### Sleep

| Property | Default | How to use it |
|---|---:|---|
| `bUseSleep` | `true` | Allows nearly stationary particles to stop updating their motion state. |
| `bIgnoreSleepWhenParentWakedUp` | `true` | Forces a child awake while its parent is awake. This avoids frozen downstream sections. |
| `SleepDeltaThreshold` | `0.05 cm` | Maximum per-update displacement considered quiet. |
| `SleepTriggerDuration` | `5 s` | Continuous quiet time required before sleep. |
| `WakeUpDeltaThreshold` | `0.1 cm` | Displacement that wakes a sleeping particle. |

Component-frame movement wakes particles, so slow actor movement is not silently discarded by sleep.

### Length, side, diagonal, and straightening constraints

| Property | Default | How to use it |
|---|---:|---|
| `bConstrainRightDiagonalDistance` | `false` | Adds one diagonal direction between neighboring chains. Helpful when ignoring the animation pose. |
| `bConstrainLeftDiagonalDistance` | `false` | Adds the opposite diagonal direction. Enable both for a more shear-resistant grid. |
| `bPreserveLengthFromParent` | `true` | Applies a final fixed-distance correction along every parent-child segment. This often allows fewer solve iterations. |
| `bPreserveLengthFromParentBetweenRealBones` | `false` | With subdivision, also preserves the original real-parent-to-real-child length across inserted virtual particles. |
| `LengthFromParentMargin` | `0.1 cm` | Tolerance for parent-child fixed length. |
| `bPreserveSideLength` | `true` | Applies fixed-distance correction across neighboring chains. Relevant to multi-chain cloth. |
| `bPreserveSideLengthBetweenRealBones` | `false` | With subdivision, also constrains corresponding real bones across chains. |
| `SideLengthMargin` | `0.1 cm` | Tolerance for side fixed length. |
| `bStretchEachBone` | `false` | Makes distance constraints reference each segment's authored length explicitly. |
| `StretchStrength` | `1.0` | Strength used by the per-segment stretch behavior. |
| `bStraightenBendedBone` | `false` | Adds a three-particle straightening constraint that does not use the animation pose as its direction target. |
| `StraightenBendedBoneStrength` | `0.0003` | Strength of the bend-straightening constraint. Small changes can be significant. |

### Isometric bending

| Property | Default | How to use it |
|---|---:|---|
| `bUseIsometricBendingConstraint` | `false` | Adds bending constraints. For one chain it uses a 1D three-particle bend; for multiple chains it builds surface-bending relationships. |
| `InvBendingCompliance` | `10000` | Constant XPBD inverse compliance for bending. |
| `bUseBendingComplianceRange` | `false` | Varies XPBD bend strength with fold angle. |
| `InvBendingComplianceMin` | `10000` | Inverse compliance near the initial angle. The implementation uses the lower of Min/Max at rest. |
| `InvBendingComplianceMax` | `80000` | Inverse compliance at or beyond the configured maximum fold angle. |
| `BendingComplianceMaxAngle` | `120 degrees` | Fold angle at which maximum XPBD bend strength is reached. |
| `BendingStiffness` | `0.01` | Constant PBD bending stiffness. |
| `bUseBendingStiffnessRange` | `false` | Varies PBD bending stiffness with fold angle. |
| `BendingStiffnessMin` | `0.01` | PBD stiffness near the initial angle. |
| `BendingStiffnessMax` | `0.1` | PBD stiffness at the configured maximum fold angle. |
| `BendingStiffnessMaxAngle` | `120 degrees` | Fold angle at which maximum PBD bend stiffness is reached. |

The internal flat-bending fields are currently not exposed as editable `UPROPERTY` fields and should not be considered part of the normal editor workflow.

### Delta-time control

| Property | Default | How to use it |
|---|---:|---|
| `FixedDeltaTime` | `0.01666 s` | Simulation delta used by every fixed step. Set it to `0` to use one variable step based on the update delta instead. |
| `bApplyDeltaTimeCorrection` | `true` | With a positive fixed delta, uses an elapsed-time accumulator to schedule zero, one, or multiple fixed steps at the target rate. If disabled, the node runs exactly one fixed step per evaluated, unpaused frame. |
| `DeltaTimeCorrectionTargetFrameRate` | `60` | Frequency at which the accumulator schedules fixed steps. It is shown only when `FixedDeltaTime > 0` and correction is enabled; minimum is `1`. |
| `MaxSubStep` | `3` | Maximum fixed steps evaluated in one rendered frame. Accumulated time beyond this budget is discarded to prevent a catch-up spiral. It has the same visibility conditions as the target frame rate; minimum is `1`. |
| `MinDeltaTime` | `KINDA_SMALL_NUMBER` | Lower clamp applied to the variable delta or to `FixedDeltaTime * TimeDilation`. |
| `MaxDeltaTime` | `0.05 s` | Upper clamp applied to that per-step delta. It does not limit how many fixed steps run; `MaxSubStep` does. |

`PlaySpeedRate` participates in both input delta time and time dilation. Test slow motion and fast-forward states explicitly if a gameplay system changes global or custom time dilation.

The three execution modes are:

1. **Variable step** — `FixedDeltaTime = 0`: one step uses the update delta clamped to `MinDeltaTime–MaxDeltaTime`.
2. **One fixed step per frame** — positive `FixedDeltaTime`, correction disabled: every evaluated, unpaused frame runs one step using `FixedDeltaTime * TimeDilation`, clamped to the same range. This mode still depends on render/evaluation frequency.
3. **Accumulator-driven fixed steps** — positive `FixedDeltaTime`, correction enabled: nonnegative elapsed update time is accumulated at `DeltaTimeCorrectionTargetFrameRate`. A fast rendered frame may run no step; a slow frame may run several, up to `MaxSubStep`. Fractional step time remains for the next frame, while time exceeding the maximum backlog is dropped.

During multiple fixed steps, the node interpolates the component transform from the previous rendered frame to the current one and prepares the pose again for every substep. This distributes component movement/rotation inertia across the substeps and advances Verlet pose history correctly. `OutputBlendDuration` also advances by the total simulated time (`per-step delta × executed steps`), not merely once per rendered frame.

For real-time pacing, normally pair reciprocal values such as `FixedDeltaTime = 0.01666` and `DeltaTimeCorrectionTargetFrameRate = 60`. Their product determines how much simulation time advances per second before time dilation. With a 60 Hz target, `MaxSubStep = 2` can catch up to a 30 FPS frame and `3` can catch up to a 20 FPS frame; choose the smallest budget that covers the expected frame-rate dip.

### Cone and custom distance constraints

| Property | Default | How to use it |
|---|---:|---|
| `bConstrainConeAngleFromParent` | `false` | Uses the moving grandparent-to-parent direction as the global cone axis. False uses the authored animation-pose direction. |
| `ConeAngle` | `0 degrees` | Global ball-socket cone limit. Zero disables it. Valid editor range is `0–90` degrees. |
| `ConeAngleOffset` | Zero Rotator | Global rotation offset for the cone center direction. Individual bones inherit it unless `bOverrideConeAngleOffset` is enabled. |
| `CustomDistanceConstraints` | Empty | Manual bone-pair minimum/maximum distance constraints. |

`ConeAngleOffset` changes the center axis of the permitted cone; it does not directly rotate a simulated bone or change `ConeAngle`. The runtime first obtains the normal cone direction from either the animation pose or the grandparent-to-parent direction. It then converts that direction into the parent particle's (`BoneA`) animation-pose local space, applies the offset rotator, and converts the direction back. The offset therefore follows the skeletal orientation instead of remaining fixed to component or world axes.

The global offset is copied to real particles and generated fake-tip particles when the simulation is built. A bone-unit offset override replaces it for the selected real bone and for a fake tip generated from that leaf. The offset has a visible constraint effect only when the effective global or per-bone cone angle is greater than zero.

### Tutorial: biasing a cape cone away from the back

1. Set a positive `ConeAngle`, for example `45` degrees.
2. Enable `bShowConstraints` and `bShowSimulatingBallSocketConstraints` in the preview.
3. Adjust the global `ConeAngleOffset` in small increments until the magenta wire cones open in the intended direction.
4. If only an outer or shoulder chain needs a different direction, add its root or upper bone to `BoneUnitSettingOverride`, enable `bOverrideConeAngleOffset`, and set its local `ConeAngleOffset`.
5. Reset the preview simulation and test several animation poses.

Pitch, yaw, and roll are interpreted around each parent bone's animation-pose local axes, so the useful values depend on the skeleton's authored orientation. Use the wire-cone preview instead of assuming that a particular Euler component always means character forward or backward.

Each `FLKAnimVerletCustomDistanceConstraintSetting` has:

| Property | Meaning |
|---|---|
| `BoneA`, `BoneB` | Constraint endpoints. A bone inside the simulation uses its simulated particle. A valid bone outside the configured chains becomes a pinned, current-animation-pose anchor. |
| `MinDistance` | Minimum allowed separation in centimeters. |
| `MaxDistance` | Maximum allowed separation in centimeters. |

If both distances are zero, the initial animation-pose distance becomes both minimum and maximum, preserving an exact rest distance. Avoid using the same bone for both endpoints. Invalid or LOD-missing external anchor bones are skipped.

Example: keep a simulated strap near a chest attachment:

```text
BoneA       = strap_03
BoneB       = spine_03
MinDistance = 8 cm
MaxDistance = 15 cm
```

`spine_03` does not need to be in `VerletBones`; it follows the animation pose as a pinned anchor.

## 10. Collision setup and data sources

AnimVerlet can combine all of these sources:

1. direct node arrays: sphere, capsule, box, and plane shapes;
2. `CollisionDataAsset`;
3. `CollisionPhysicsAsset`;
4. per-frame `DynamicCollisionShapes`;
5. world collision through `WorldCollisionProfile`;
6. self-collision.

The first three static sources are additive during initialization: direct shapes are copied first, then data-asset shapes and physics-asset shapes are appended. `DynamicCollisionShapes` is evaluated separately every frame and is also additive. Do not assign the same collider through several sources unless duplicate contact is intentional.

### Global collision parameters

| Property | Default | How to use it |
|---|---:|---|
| `bUseBroadphase` | `true` | Uses an acceleration structure to reduce collision candidate checks. Keep enabled for nontrivial setups. |
| `Thickness` | `0.3 cm` | Global simulated particle radius and chain/surface thickness. |
| `FrictionCoefficient` | `0` | Coulomb friction used by PBD-style collision projection in both PBD and XPBD modes. |
| `bUseCapsuleCollisionForChain` | `true` | One chain uses parent-child capsules; multiple chains use triangles. False uses spheres. |
| `bUseSelfCollision` | `false` | Enables collision among the simulated bones/surface. Potentially expensive. |
| `bUseTriangleSelfCollision` | `false` | On multiple chains, chooses triangle-triangle instead of sphere-triangle self collision. Single-chain self collision uses capsules regardless. |
| `SelfCollisionAdditionalThickness` | `0.1 cm` | Extra separation radius for self contact. |
| `WorldCollisionProfile` | None | A non-None profile enables world sweep collision against that Unreal collision profile. |
| `WorldCollisionExcludeBones` | Empty | Simulated bones excluded from world collision. |
| `SphereCollisionShapes` | Empty | Direct local/world sphere colliders. |
| `CapsuleCollisionShapes` | Empty | Direct local/world capsule colliders. |
| `BoxCollisionShapes` | Empty | Direct local/world box colliders. |
| `PlaneCollisionShapes` | Empty | Direct local/world finite or infinite plane colliders. |
| `CollisionDataAsset` | None | Shareable AnimVerlet collider data. Editable and previewable through the node tools. |
| `CollisionPhysicsAsset` | None | Imports supported physics-asset primitives at runtime. It cannot be modified in the AnimVerlet preview like a collision data asset. |
| `DynamicCollisionShapes` | Empty | Graph pin intended for collision supplied every frame by an Animation Blueprint or external code. |

World collision performs physics sweeps and is generally more expensive than a small set of local shapes. With world collision, the implementation uses sphere/capsule-style chain tests rather than multi-chain triangle geometry.

### Common collider fields

Every direct or dynamic collider contains:

| Property | Explanation |
|---|---|
| `bUseAbsoluteWorldTransform` | False means bone-attached offsets. True means `LocationOffset` and, where applicable, `RotationOffset` are an absolute world transform. |
| `AttachedBone` | Bone used when absolute-world mode is false. |
| `ExcludeBones` | Simulated bones that do not collide with this shape. |
| `LocationOffset` | Bone-relative location offset, or absolute world location in world-transform mode. |

Shape-specific fields:

| Shape | Fields |
|---|---|
| Sphere | `Radius` |
| Capsule | `RotationOffset`, `Radius`, `HalfHeight` |
| Box | `RotationOffset`, `HalfExtents` |
| Plane | `RotationOffset`, `bFinitePlane`, `FinitePlaneHalfExtents` |

Planes use the rotated local up/Z direction as their normal. An infinite plane has `bFinitePlane = false`; a finite plane uses its XY half extents.

### Bone-attached versus absolute-world colliders

Use a bone-attached collider for character anatomy:

```text
bUseAbsoluteWorldTransform = false
AttachedBone               = pelvis
LocationOffset             = local offset from pelvis
```

Use an absolute-world collider for an external object already expressed in world space:

```text
bUseAbsoluteWorldTransform = true
LocationOffset             = object world location
RotationOffset             = object world rotation
```

An absolute-world collider is converted into the simulated component's space every evaluation.

### Collision data asset workflow

1. Create a Data Asset whose class is `LKAnimVerletCollisionDataAsset`.
2. Assign it to `CollisionDataAsset`.
3. Either edit its sphere/capsule/box/plane data directly or first create direct node shapes.
4. Use **AnimVerlet Tool > Convert Collision To DataAsset** to copy direct node shapes into the asset.
5. Save the data asset manually after conversion.

Other editor conversion buttons:

| Button | Result |
|---|---|
| `Convert Collision From DataAsset` | Clears direct node shape arrays, then copies the data asset into them. Save the Animation Blueprint manually. |
| `Convert Collision From PhysicsAsset to DataAsset` | Clears/replaces the target data asset with converted physics-asset primitives. Save the data asset manually. |
| `Convert Collision From PhysicsAsset` | Clears direct node shapes, then copies supported physics-asset primitives into them. Save the Animation Blueprint manually. |

The physics-asset conversion supports spheres, capsules/sphyls, and boxes. Planes are AnimVerlet-specific and must be authored separately.

### Runtime dynamic-collision tutorial

Use `DynamicCollisionShapes` when AnimVerlet must collide with a weapon, shield, vehicle part, or another moving component.

Blueprint flow:

```text
Moving Primitive Component
        │
        ▼
Make Collision Shape List From Primitive Component
        │
        ▼
AnimVerlet.DynamicCollisionShapes
```

Available Blueprint helpers in `ULKAnimVerletBlueprintFunctionLibrary`:

| Function | Use |
|---|---|
| `MakeCollisionShapeListFromPrimitiveComponent` | Builds absolute-world shapes from one primitive component's body instance. |
| `MakeCollisionShapeListFromSkeletalMeshComponent` | Builds shapes from all body instances owned by a skeletal mesh component. |
| `MakeCollisionShapeListFromCollisionDataAsset` | Evaluates a data asset against a skeletal mesh component's current bone transforms and outputs absolute-world shapes. |
| `MakeCollisionShapeListFromPhysicsAsset` | Evaluates a physics asset against a skeletal mesh component and outputs absolute-world shapes. |
| `MakeCollisionShapeListFromCollisionShapeList` | Converts bone-attached shapes to absolute-world shapes using current skeletal transforms. |

Update the list before the Animation Blueprint evaluation that consumes it. If the data originates in the character or another gameplay object, store the generated `FLKAnimVerletCollisionShapeList` in a thread-safe Animation Blueprint input variable and connect it to the node pin according to the project's animation update architecture.

## 11. Gravity, forces, and wind

### Gravity

| Property | Default | How to use it |
|---|---:|---|
| `Gravity` | `(0,0,0)` | Gravity-like displacement term. The presets choose `-9.8` for linear-delta animation mode and `-980` for squared-delta physics mode. |
| `bGravityInWorldSpace` | `true` | Converts gravity from world to component space each update. False interprets it directly in component space. |

### Other forces

| Property | Default | How to use it |
|---|---:|---|
| `StretchForce` | `0` | Pushes each particle in its authored parent-to-child direction. |
| `bAlignStretchForceToGravity` | `false` | Uses the gravity-aligned authored direction for stretch. |
| `SideStraightenForce` | `0` | Applies a sideward force derived from the positional relationship between chain roots. Relevant to multi-chain layouts. |
| `ShapeMemoryForce` | `0` | Pushes particles toward the direction of their animation-pose target. Unlike `AnimationPoseInertia`, this is applied as a force-like term. |
| `bAlignShapeMemoryForceToGravity` | `false` | Uses the gravity-aligned pose as shape-memory target. |
| `ExternalForce` | `(0,0,0)` | User-supplied constant force vector. |
| `bExternalForceInWorldSpace` | `true` | Converts external force from world to component space. |

Except for gravity in the current implementation, force response is scaled by inverse mass. With `bUseSquaredDeltaTime = false`, force terms are multiplied by `dt`; with it enabled, they are multiplied by `dt²`. Tune values within the selected integration mode rather than copying values blindly between presets.

### Random wind

| Property | Default | How to use it |
|---|---:|---|
| `RandomWindDirection` | `(0,0,0)` | Main random-force direction. A zero vector disables this wind. |
| `RandomWindSizeMin` | `0` | Minimum random magnitude sampled independently for each particle/update. |
| `RandomWindSizeMax` | `0` | Maximum random magnitude. Keep Min less than or equal to Max for intuitive behavior. |
| `bRandomWindDirectionInWorldSpace` | `true` | Treats the direction as world space; false treats it as component space. |
| `AdditionalRandomWinds` | Empty | Additional independently sampled directional random-force records. |
| `bAdjustWindComponent` | `false` | Samples Unreal `UWindDirectionalSourceComponent` wind from the world scene. |
| `WindComponentScale` | `1.0` | Exposed wind-component scale. In the current runtime update path this value is stored and synchronized but is not multiplied into the sampled scene wind, so changing it alone does not change the result. |

Each `AdditionalRandomWinds` entry has `RandomForceDirection`, `RandomForceSizeMin`, `RandomForceSizeMax`, and `bRandomForceDirectionInWorldSpace`.
Random-force direction vectors are not normalized by the simulation update, so their vector length also scales the final effect. Use a unit direction vector when the Min/Max fields should be the only magnitude controls.

## 12. Component inertia

Component inertia is the apparent secondary motion caused by movement or rotation of the owning skeletal mesh component in world space.

| Property | Default | How to use it |
|---|---:|---|
| `MoveInertiaScale` | `1.0` | Multiplier for component translation inertia. Set to `0` to remove translation-driven lag. |
| `bIgnoreSuddenMoveInertia` | `false` | Drops translation inertia for a frame when movement exceeds the threshold. Useful for teleports. |
| `MoveInertiaIgnoreThreshold` | `800 cm` | Sudden-move distance threshold. |
| `bClampMoveInertia` | `true` | Caps accepted component translation instead of allowing unbounded displacement. |
| `MoveInertiaClampMaxDistance` | `300 cm` | Maximum accepted translation displacement. |
| `RotationInertiaScale` | `1.0` | Multiplier for component rotation inertia. |
| `bIgnoreSuddenRotationInertia` | `false` | Drops rotational inertia for a frame beyond the threshold. |
| `RotationInertiaIgnoreDegrees` | `90 degrees` | Sudden-rotation threshold. |
| `bClampRotationInertia` | `true` | Caps accepted component rotation. |
| `RotationInertiaClampDegrees` | `30 degrees` | Maximum accepted rotation per update. |
| `ComponentInertiaTangentialDamping` | `1.0` | Fraction of parent-relative tangential velocity retained after component inertia. `1` keeps it; `0` removes it. The value is treated as retention per 60 Hz step and adjusted for the current delta time. |

For gameplay teleports, Unreal's `ResetPhysics` flow is the cleanest way to synchronize the simulation with the new pose. Ignore/clamp settings are additional protection for large unannounced component deltas.

## 13. Preview and editor controls

These properties belong to the editor graph node and affect visualization, not runtime simulation:

### Collision input display

| Property | Default | Purpose |
|---|---:|---|
| `bShowAndModifySphereCollision` | `true` | Shows direct/data-asset/physics-asset sphere inputs and allows supported selection/editing. |
| `bShowAndModifyCapsuleCollision` | `true` | Shows capsule inputs. |
| `bShowAndModifyBoxCollision` | `true` | Shows box inputs. |
| `bShowAndModifyPlaneCollision` | `true` | Shows plane inputs. |
| `bShowCollisionAssetSource` | `true` | Shows collision sources from assigned assets. |

### Simulation debug display

| Property | Default | Purpose |
|---|---:|---|
| `bShowBones` | `true` | Draws simulated particles/bones. |
| `BoneThicknessRenderScale` | `1.0` | Scales only the debug rendering of thickness. |
| `bShowCapsuleBoneChainConstraints` | `true` | Draws capsule/triangle/sphere chain geometry according to the selected mode. |
| `bShowSleep` | `true` | Uses the sleep-state visualization. |
| `bShowBoneBounds` | `false` | Draws broadphase bounds. |
| `bShowConstraints` | `true` | Master display for active constraint visuals. |
| `bShowDistanceConstraintLengths` | `true` | Displays the current endpoint distance of every active `FLKAnimVerletConstraint_Distance` at its midpoint, formatted to two decimal places in centimeters. Hidden when `bShowConstraints` is false. |
| `bShowFixedPoints` | `true` | Draws pinned/root points. |
| `bShowSimulatingBallSocketConstraints` | `true` | Draws cone/ball-socket constraints. |
| `bShowSimulatingSphereCollisionConstraints` | `false` | Draws active sphere collision constraints. |
| `bShowSimulatingSphereCollisionConstraintsBounds` | `false` | Draws their bounds. |
| `bShowSimulatingCapsuleCollisionConstraints` | `false` | Draws active capsule collision constraints. |
| `bShowSimulatingCapsuleCollisionConstraintsBounds` | `false` | Draws their bounds. |
| `bShowSimulatingBoxCollisionConstraints` | `false` | Draws active box collision constraints. |
| `bShowSimulatingBoxCollisionConstraintsBounds` | `false` | Draws their bounds. |
| `bShowSimulatingPlaneCollisionConstraints` | `false` | Draws active plane collision constraints. |
| `bShowIsometricBendingConstraints` | `false` | Draws bending relationships. |

Use **Preview > Reset Simulation** whenever a parameter change leaves old momentum in the preview or when comparing two settings from the same initial pose.

The distance overlay includes structural parent-child constraints, multi-chain side/diagonal constraints, and valid custom distance constraints because all of them are stored in the runtime distance-constraint list. Use it to compare current spacing with authored or custom Min/Max targets. Disable it on dense cloth grids when the labels obscure the other debug visuals; it is an editor-only display and does not change runtime simulation.

## 14. Preset reference

| Preset | Pose following | Solver | Delta mode | Bending | Damping | Iterations | Gravity |
|---|---|---|---|---|---:|---:|---|
| `Custom` | Unchanged | Unchanged | Unchanged | Unchanged | Unchanged | Unchanged | Unchanged |
| `AnimationPose` | Enabled | PBD | Linear `dt` | Off | `0.8` | `2` | `(0,0,-9.8)` |
| `Physics_XPBD` | Ignored | XPBD | Squared `dt` | On | `0.99` | `4` | `(0,0,-980)` |
| `Physics_PBD` | Ignored | PBD | Squared `dt` | Off | `0.9` | `4` | `(0,0,-980)` |

Presets change only the listed values. They do not configure bones, collision shapes, mass, compliance, cone limits, subdivision, or inherited skeletal-control alpha.

## 15. Tuning order

Tune in this order to avoid compensating for a topology problem with excessive solver cost:

1. **Topology**: correct roots, array order, exclusions, and matching chain lengths.
2. **Integration style**: choose animation-pose, XPBD physics, or PBD physics.
3. **Rest shape**: fake tip, pose following, cone limits, length and side constraints.
4. **Collision geometry**: thickness, collider transforms, and per-collider exclusions.
5. **Material response**: stiffness/compliance, bending, damping, and mass.
6. **External motion**: gravity, forces, wind, and component inertia.
7. **Robustness**: delta clamps, warmup, reset behavior, sleep, and LOD rebuild.
8. **Performance**: broadphase, solver iterations, subdivisions, world collision, and self-collision.

## 16. Troubleshooting

### Nothing moves

- Confirm `bActivate` is true and inherited node alpha is greater than zero.
- Confirm `bPause` is false.
- Confirm every `RootBone` is valid at the current LOD.
- Remember that the root is pinned; use a chain with descendants.
- If all particles are locked by overrides or tip/root locks, no free motion remains.
- Add gravity, force, animation motion, or component movement so the system has an input.

### The node has no visible output

- Compile the Animation Blueprint after changing bone topology.
- Check that the node is connected to the final pose path.
- Check inherited skeletal-control LOD threshold and alpha.
- Avoid using `bFakeBone` on the only chain unless an offset proxy topology is specifically intended.
- Verify the simulated bones exist in the required-bone container for the current LOD.

### The chain explodes after a hitch or teleport

- With corrected fixed stepping, set `MaxSubStep` high enough for ordinary short frame drops but low enough to bound catch-up work. Excess backlog is discarded automatically.
- Reduce `MaxDeltaTime` when the **per-step** delta can become too large, especially with variable stepping or high time dilation. It does not cap fixed-step count.
- Trigger Unreal's `ResetPhysics` behavior after a teleport.
- Enable sudden-move and sudden-rotation ignore options.
- Keep movement and rotation clamps enabled.
- Use warmup and a nonzero `OutputBlendDuration`.
- Check for zero or extremely small XPBD inverse compliance values.

### The chain stretches too much

- Keep `bPreserveLengthFromParent` enabled.
- Increase PBD `Stiffness` or XPBD `InvCompliance`.
- Increase `SolveIteration`.
- Reduce force magnitude.
- With subdivision, try `bPreserveLengthFromParentBetweenRealBones`.

### Multi-chain cloth shears or separates

- Verify `VerletBones` are ordered spatially and have corresponding depths.
- Enable both diagonal constraints.
- Keep `bPreserveSideLength` enabled.
- Use isometric bending for fold behavior.
- Inspect side and bending constraints in the preview.
- Enable `bShowDistanceConstraintLengths` to locate rows or diagonals whose current spacing is abnormal.

### A cone limit points in the wrong direction

- Enable `bShowSimulatingBallSocketConstraints` and inspect the magenta cone.
- Confirm whether `bConstrainConeAngleFromParent` should use the moving parent direction or the authored pose direction.
- Adjust `ConeAngleOffset`; remember that it uses the parent bone's animation-pose local axes.
- Check `BoneUnitSettingOverride` for an unintended `bOverrideConeAngleOffset`.
- Reset the preview after changing the constraint setup.

### Collision is missed

- Increase `Thickness` or a per-bone thickness override.
- Add subdivision to long segments. Use `bOverrideSubDivideBones` when only selected segments or fake tips need the extra particles.
- Confirm the collider is attached to a valid bone at the current LOD.
- Confirm absolute-world transforms are truly world transforms.
- Confirm the particle is not in the collider's `ExcludeBones`.
- Increase `SolveIteration` only after checking geometry.
- For an awkward segment, enable its sphere-collision override.

### Collision sticks or jitters

- Reduce `FrictionCoefficient`.
- Remove duplicate colliders supplied by more than one source.
- Avoid overlapping colliders that demand contradictory corrections.
- Reduce self-collision thickness.
- Raise iterations gradually.
- Use an appropriate mass distribution; very large neighboring mass differences can look unstable.

### Physics changes with frame rate

- Use the physics preset with squared delta time for acceleration-like behavior.
- Use corrected fixed stepping for frame-rate-independent scheduling. Pair reciprocal values such as `FixedDeltaTime = 0.01666` and `DeltaTimeCorrectionTargetFrameRate = 60`.
- Size `MaxSubStep` for the lowest frame rate that should recover without dropping accumulated time; for example, use at least `2` for a 60 Hz target at 30 FPS.
- Enable damping and animation-pose inertia corrections where appropriate.
- Keep `MaxDeltaTime` bounded.
- Test the actual project frame-rate range. Variable stepping uses clamped average FPS for damping/pose-inertia correction, while corrected fixed stepping uses `DeltaTimeCorrectionTargetFrameRate`.

### LOD switching breaks the chain

- Enable `bRebuildSimulationOnLODChange`.
- Ensure each configured root exists in every LOD where the node evaluates.
- Avoid external custom-distance anchors that disappear from the required-bone set.
- Consider disabling the node through its inherited LOD threshold before a destructive skeleton reduction.

## 17. Performance checklist

- Keep `bUseBroadphase` enabled.
- Prefer local character colliders over world sweeps when a few primitives are sufficient.
- Start with `SolveIteration = 2–4`.
- Remember that a slow rendered frame can execute up to `MaxSubStep` complete simulation updates. Keep the value to the smallest practical recovery budget.
- Add subdivisions only to segments that need collision resolution or softer curvature; prefer per-segment overrides over raising particle counts across the entire node.
- Enable self-collision only when visually necessary.
- Prefer sphere-triangle over triangle-triangle self collision for multi-chain cloth unless the visual difference justifies the cost.
- Use sleep for accessories that remain still for long periods.
- Skip dedicated-server updates unless authoritative simulated transforms are required.
- Use the editor visualization to find unnecessary colliders, excessive thickness, and redundant constraints.

## 18. Recommended baseline recipes

### Soft ponytail

```text
Preset                         = AnimationPose
Damping                        = 0.82
AnimationPoseInertia           = 0.03
AnimationPoseDeltaInertia      = 0.03
ConeAngle                      = 60 deg
bPreserveLengthFromParent      = true
bMakeFakeTipBone               = true
SolveIteration                 = 2
```

### Heavy tail

```text
Preset                         = AnimationPose
Damping                        = 0.9
AnimationPoseInertia           = 0.015
ConeAngle                      = 40 deg
Mass                           = 2.0
Gravity                        = tune for the selected delta mode
SolveIteration                 = 3
```

### Physics-oriented cape

```text
Preset                         = Physics_XPBD
InvCompliance                  = 100000000
bUseIsometricBendingConstraint = true
InvBendingCompliance           = 10000
bPreserveLengthFromParent      = true
bPreserveSideLength            = true
bConstrainRightDiagonalDistance = true
bConstrainLeftDiagonalDistance  = true
bUseBroadphase                 = true
SolveIteration                 = 4
```

These are starting points, not universal material definitions. Bone spacing, character scale, animation speed, collider layout, and target frame rate all affect the final result.
