# [**AnimVerlet**](https://github.com/chonjay21/AnimVerlet)
![GitHub](https://img.shields.io/github/license/chonjay21/AnimVerlet)
![GitHub stars](https://img.shields.io/github/stars/chonjay21/AnimVerlet?style=social)
## AnimVerlet. Yet another anim dynamics plugin for cloth, string, blob simulation in UE4, UE5 (Unreal Engine)
* Support Cloth(cloak, cape, flag) simulation while respecting the animation pose.
* Support String(hair, skirt)(one chain cloth) simulation while respecting the animation pose.
* Support Blob, fluid like soft body simulation while respecting the animation pose.
* Support custom collision(Sphere, Capsule, Box, Plain) and support the feature to easily modify collision transforms in the preview editor
* Support ball socket constraints(Cone angle constraints)
* Use simple position based physics simulation or optionally use extended position based physics simulation (by Verlet Integration).
* Minimize solve iterations by using a heuristic solve order.(1 iteration is usually sufficient)
* Easy readable source code

<br />

# Screenshots
![](https://github.com/chonjay21/Screenshots/blob/main/AnimVerlet_UE5.png)
![](https://github.com/chonjay21/Screenshots/blob/main/AnimVerlet_Short.gif)
![](https://github.com/chonjay21/Screenshots/blob/main/AnimVerlet_Graph.gif)
* [XPBD verlet simulation]<br />
![](https://github.com/chonjay21/Screenshots/blob/main/AnimVerlet_String.gif)
* [Drag and drop collision setup in preview]<br />
![](https://github.com/chonjay21/Screenshots/blob/main/AnimVerlet_Collision.gif)

<br />

# Who I am

Find me at:
* [GitHub](https://github.com/chonjay21)
* [Youtube](https://www.youtube.com/channel/UCIwbmzMBsIJ0FVHlbyOGPDg/featured)

<br />

# Supported Version

The UnrealEngine versions supported by this plugin are:

| Engine | Engine Version | Precompiled Version(Win64 only) | SourceCode Version(All platforms) |
| :----: | --- | --- | --- |
| UE4 | 4.27 or later | v.220628 | Latest |
| UE5 | 5.0.2 ~ 5.2.1 | v.220628 | Latest |
| UE5 | 5.3.0 ~ 5.4.3 | v3.0 | Latest |
| UE5 | 5.5.4 | v3.3 | Latest |
| UE5 | 5.6.1 | v3.5 | Latest |
| UE5 | 5.7.1 or later | Latest | Latest |

<br />

# Usage

1. Download AnimVerlet example from [GitHub](https://github.com/chonjay21/AnimVerlet)
2. Put the "AnimVerlet" directory to your Project`s "Plugins" directory
3. Use AnimVerlet Animation Node in Animation Blueprint
4. You can choose preset in AnimVerlet Node(AnimationPose, Physics_XPBD, Physics_PBD) or customize each settings

<br />

# Example

* UE4: [UE4 Example project](https://github.com/chonjay21/AnimVerletExample_UE4)
* UE5: [UE5 Example project](https://github.com/chonjay21/AnimVerletExample_UE5)
* Youtube: [Youtube Example](https://youtu.be/3p2-tD12Z5A)

<br />

# License

View [license information](https://github.com/chonjay21/AnimVerlet/blob/master/LICENSE) of this plugin.
