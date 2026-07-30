# `LKAnimNode_AnimVerlet` 튜토리얼 및 파라미터 레퍼런스

이 문서는 Unreal Engine Animation Blueprint에서 **AnimVerlet** 스켈레탈 컨트롤 노드를 구성하고 튜닝하는 방법을 설명합니다. 현재 `FLKAnimNode_AnimVerlet` 구현, 에디터 그래프 노드, 충돌 데이터 타입, Blueprint 충돌 헬퍼 함수를 기준으로 작성되었습니다.

> 문서에 사용된 본 이름은 예시입니다. 실제 스켈레톤의 본 이름으로 바꾸어 사용하십시오.

## 1. 노드의 역할

`AnimVerlet`은 하나 이상의 스켈레탈 본 계층을 파티클과 제약조건으로 변환하고, 컴포넌트 공간에서 시뮬레이션한 뒤 계산된 위치와 회전을 실제 본에 다시 적용합니다. 다음과 같은 용도에 적합합니다.

- 포니테일, 땋은 머리, 더듬이, 귀, 꼬리
- 스트랩, 끈, 체인, 매달린 액세서리
- 여러 개의 병렬 체인으로 구성된 망토 또는 천 형태의 본 그리드
- 캐릭터, 월드 또는 자기 자신과 충돌해야 하는 세컨더리 모션

노드는 크게 두 가지 방식으로 사용할 수 있습니다.

- **애니메이션 포즈 추종 세컨더리 모션**: 애니메이션 포즈가 계속 목표로 작용합니다. 다루기 쉽고 헤어, 꼬리, 액세서리에 일반적으로 적합합니다.
- **물리 중심 모션**: 애니메이션 포즈를 무시하고, 중력과 힘에 제곱 델타 타임을 사용하며, 제약조건이 대부분의 형상을 결정합니다. 천 형태의 그리드나 물리적인 움직임에 적합합니다.

각 체인의 시뮬레이션 루트는 자동으로 고정됩니다. 루트 아래의 실제 본들은 본별 오버라이드나 팁 고정 옵션을 사용하지 않는 한 자유롭게 움직입니다.

## 2. 노드를 추가하기 전에

스켈레톤은 다음 규칙을 고려하여 준비하십시오.

1. 하나의 `VerletBones` 항목은 `RootBone`에서 시작하여 모든 하위 본을 재귀적으로 방문합니다.
2. 하나의 체인은 분기가 없는 단순한 계층으로 만드는 것이 가장 예측하기 쉽습니다.
3. 망토에는 길이가 비슷한 병렬 체인을 여러 개 사용하고 체인마다 `VerletBones` 항목을 하나씩 추가합니다.
4. 다중 체인 천에서는 루트를 실제 공간의 좌우 순서로 입력합니다. 배열에서 서로 이웃한 항목 사이에 측면 및 대각선 제약조건이 생성됩니다.
5. LOD에 따라 시뮬레이션 본이 제거된다면 `bRebuildSimulationOnLODChange` 활성화 여부를 결정해야 합니다.

단일 체인 예시:

```text
hair_root
└── hair_01
    └── hair_02
        └── hair_03
```

다중 체인 망토 예시:

```text
cape_left_root    cape_center_root    cape_right_root
└── ...           └── ...             └── ...
```

## 3. 노드 추가 및 연결

1. 캐릭터의 Animation Blueprint를 엽니다.
2. **AnimGraph**에 **AnimVerlet** 노드를 추가합니다.
3. 시뮬레이션을 구동할 포즈를 노드의 입력 포즈에 연결합니다.
4. AnimVerlet을 다음 스켈레탈 컨트롤 노드 또는 최종 애니메이션 포즈에 연결합니다.
5. **Setup > Verlet Bones**에 항목을 하나 이상 추가하고 `RootBone`을 선택합니다.
6. Animation Blueprint를 컴파일합니다.

AnimGraph에서 노드의 위치가 중요합니다. AnimVerlet의 입력 포즈에 포함되어야 하는 애니메이션 또는 IK 연산 뒤에 배치하십시오. AnimVerlet 뒤에 놓인 연산은 이미 시뮬레이션된 본을 추가로 수정합니다.

이 노드는 Unreal Engine의 스켈레탈 컨트롤 베이스 노드로부터 출력 알파와 LOD Threshold 같은 표준 프로퍼티도 상속합니다. 내부 시뮬레이션 파라미터를 변경하지 않고 전체 노드 효과를 블렌딩하려면 상속된 알파를 사용하십시오.

## 4. 빠른 시작: 애니메이션 기반 포니테일

본 계층이 `hair_root -> hair_01 -> hair_02 -> hair_03`이라고 가정합니다.

### 1단계: 체인 정의

`VerletBones`에 항목을 하나 추가합니다.

| 필드 | 값 |
|---|---|
| `RootBone` | `hair_root` |
| `ExcludeBones` | 비어 있음 |
| `Mass` | `1.0` |
| `bFakeBone` | `false` |

노드는 `hair_root`의 모든 하위 본을 자동으로 포함합니다. 각 본을 별도의 `VerletBones` 항목으로 추가하지 마십시오.

### 2단계: AnimationPose 프리셋으로 시작

**Preset > Preset Type**을 `AnimationPose`로 설정합니다. 이 프리셋은 다음 값을 적용합니다.

| 프로퍼티 | 프리셋 값 |
|---|---:|
| `bIgnoreAnimationPose` | `false` |
| `bUseXPBDSolver` | `false` |
| `bUseSquaredDeltaTime` | `false` |
| `bUseIsometricBendingConstraint` | `false` |
| `Damping` | `0.8` |
| `SolveIteration` | `2` |
| `Gravity` | `(0, 0, -9.8)` |

프리셋과 관련된 프로퍼티를 직접 변경하면 에디터의 프리셋 선택이 `Custom`으로 돌아갑니다.

### 3단계: 형상과 안정성 설정

다음 값을 시작점으로 사용할 수 있습니다.

| 프로퍼티 | 권장 시작값 | 목적 |
|---|---:|---|
| `bMakeFakeTipBone` | `true` | 마지막 실제 본에 시뮬레이션 자식을 추가하여 회전을 계산할 수 있게 합니다. |
| `FakeTipBoneLength` | 마지막 본 길이와 비슷한 값 | 팁 회전이 불완전하게 계산되는 것을 방지합니다. |
| `bPreserveLengthFromParent` | `true` | 각 세그먼트를 원래 길이에 가깝게 유지합니다. |
| `LengthFromParentMargin` | `0.1 cm` | 작은 보정 허용 오차를 둡니다. |
| `ConeAngle` | `45–70 degrees` | 체인이 루트 또는 부모 방향 안쪽으로 과도하게 접히는 것을 방지합니다. |
| `Thickness` | 보이는 헤어 볼륨에 맞춤 | 시뮬레이션 충돌 반경을 정의합니다. |
| `SolveIteration` | `2–4` | 제약조건이나 충돌이 눈에 띄게 해결되지 않을 때만 올립니다. |

### 4단계: 반응 튜닝

- `Damping`을 `1.0`에 가깝게 올리면 움직임이 오래 유지되어 스윙이 길어집니다.
- `Damping`을 낮추면 더 빨리 안정됩니다.
- `AnimationPoseInertia`를 올리면 현재 애니메이션 포즈 위치로 더 강하게 끌립니다.
- `AnimationPoseDeltaInertia`를 올리면 입력 포즈의 움직임이 시뮬레이션에 더 많이 전달됩니다.
- 애니메이션의 불연속 변화로 인한 큰 점프를 막으려면 `AnimationPoseDeltaInertiaClampMax`를 사용합니다.
- 체인이 원래 곡선으로 적극적으로 돌아와야 한다면 `ShapeMemoryForce`를 올립니다.

### 5단계: 몸체 충돌 추가

`CapsuleCollisionShapes`에 캡슐을 하나 추가합니다.

| 필드 | 예시 |
|---|---|
| `bUseAbsoluteWorldTransform` | `false` |
| `AttachedBone` | `spine_03` |
| `LocationOffset` | 상체 위치에 맞게 조정 |
| `RotationOffset` | 캡슐의 로컬 Z축을 상체 방향에 맞춤 |
| `Radius` | `12 cm` |
| `HalfHeight` | `22 cm` |

**Collision Input**에서 캡슐 표시를 활성화하고 프리뷰 뷰포트에서 콜라이더를 선택한 뒤 트랜스폼 기즈모로 맞춥니다. `bUseAbsoluteWorldTransform`이 false이면 이 값들은 부착 본 기준 오프셋입니다.

## 5. XPBD 다중 체인 망토 튜토리얼

캐릭터의 왼쪽에서 오른쪽으로 망토 폭을 따라 배치된, 본 개수가 비슷한 여섯 개의 비분기 체인이 있다고 가정합니다.

### 1단계: 정렬된 루트 입력

망토의 왼쪽에서 오른쪽 순서로 체인마다 `VerletBones` 항목을 하나씩 추가합니다.

```text
cape_l_03_root
cape_l_02_root
cape_l_01_root
cape_r_01_root
cape_r_02_root
cape_r_03_root
```

배열 순서는 중요합니다. 구현에서 이웃한 항목 사이에 측면, 대각선, 벤딩, 삼각형 관계를 생성하기 때문입니다. 망토는 보통 열린 면이므로 가장 왼쪽과 오른쪽 항목은 서로 분리된 상태가 정상입니다. 특수한 폐곡면이나 접힌 구간이 있다면 프리뷰 제약조건을 확인하고 필요한 위치에 커스텀 제약조건을 추가하십시오.

### 2단계: 물리 프리셋 선택

`Physics_XPBD`를 선택합니다. 제곱 델타 타임, XPBD, 아이소메트릭 벤딩을 활성화하고 Unreal 단위 중력 `(0, 0, -980)`을 사용합니다.

권장 시작값:

| 프로퍼티 | 권장 시작값 |
|---|---:|
| `InvCompliance` | `100000000` |
| `InvBendingCompliance` | `10000` |
| `SolveIteration` | `4` |
| `bPreserveLengthFromParent` | `true` |
| `bPreserveSideLength` | `true` |
| `bConstrainRightDiagonalDistance` | `true` |
| `bConstrainLeftDiagonalDistance` | `true` |
| `bUseCapsuleCollisionForChain` | `true` |

구성된 체인이 두 개 이상이고 `bUseCapsuleCollisionForChain`이 true이면 체인 충돌 형상은 삼각형이 됩니다. 토폴로지나 체인 길이가 깔끔한 면을 만들지 못한다면 구 충돌을 사용하거나 본 배치를 수정하십시오.

### 3단계: 늘어남과 접힘을 분리하여 튜닝

- `InvCompliance`는 거리 제약조건의 강성을 제어합니다. 역 컴플라이언스가 높을수록 덜 늘어납니다.
- 벤딩 역 컴플라이언스 값은 거리 강성을 대체하는 것이 아니라 접힘 저항을 제어합니다.
- `bUseBendingComplianceRange`를 활성화하면 원래 각도 근처에서는 부드럽고 많이 접혔을 때는 강하게 만들 수 있습니다.
- `BendingComplianceMaxAngle`은 `InvBendingComplianceMax`가 완전히 적용되는 접힘 각도입니다.

### 4단계: 충돌 품질 향상

1. `Thickness`를 원하는 파티클 또는 표면 반경으로 설정합니다.
2. 어깨, 상부 등, 몸통, 필요 시 팔 캡슐을 추가하거나 Collision Data Asset/Physics Asset을 사용합니다.
3. `bUseBroadphase`를 활성화된 상태로 유지합니다.
4. 토폴로지와 콜라이더 크기가 올바른지 확인한 다음에만 `SolveIteration`을 올립니다.
5. 마찰은 천천히 올립니다. `FrictionCoefficient`가 너무 높으면 망토가 몸통이나 팔에 달라붙을 수 있습니다.

### 5단계: 선택적 자기 충돌

일반 제약조건과 몸체 충돌이 안정된 뒤에만 `bUseSelfCollision`을 활성화하십시오.

- 다중 체인 표면에서 `bUseTriangleSelfCollision = false`는 구 대 삼각형 검사를 사용합니다.
- `bUseTriangleSelfCollision = true`는 삼각형 대 삼각형 검사를 사용하며 더 비쌉니다.
- `SelfCollisionAdditionalThickness`는 자기 접촉 거리를 확장합니다.

## 6. 본 체인 데이터 구조

### `FLKAnimVerletBoneSetting`

각 `VerletBones` 항목에는 다음 값이 있습니다.

| 프로퍼티 | 기본값 | 설명 |
|---|---:|---|
| `RootBone` | None | 재귀적으로 포함할 첫 스켈레탈 본입니다. 루트 파티클은 자동으로 고정됩니다. |
| `ExcludeBones` | 비어 있음 | 시뮬레이션 파티클에서 제외할 본입니다. 제외된 본의 하위 본도 계속 방문합니다. |
| `BoneUnitSettingOverride` | 비어 있음 | 선택적인 본별 고정, 관절, 두께, 충돌 모드, 질량 오버라이드입니다. |
| `bStraightenExcludedBonesByParent` | `true` | 제외된 본을 원래 포즈 위치에만 두지 않고 시뮬레이션된 부모/자식 방향을 따라 재배치합니다. |
| `bFakeBone` | `false` | 이 본들을 기준으로 오프셋 프록시 체인을 시뮬레이션합니다. 주로 가상 보조 토폴로지이며 표시된 파티클을 일반 실제 본 트랜스폼처럼 직접 출력하지 않습니다. |
| `FakeBoneOffsetDir` | Forward `(1,0,0)` | `bFakeBone` 사용 시의 오프셋 방향입니다. 사용 전에 정규화됩니다. |
| `FakeBoneOffsetSize` | `3 cm` | 가상 체인과 원본 본 트랜스폼 사이의 거리입니다. |
| `Mass` | `1.0` | 이 항목에 포함된 파티클의 기본 질량입니다. 힘에 대한 반응은 역질량으로 스케일됩니다. 최소 유효 질량은 `0.01`입니다. |

### 계층을 끊지 않고 본 제외

다음과 같이 헬퍼 본이 포함된 체인을 가정합니다.

```text
strap_root -> strap_01 -> strap_twist_helper -> strap_02 -> strap_03
```

`strap_twist_helper`를 `ExcludeBones`에 추가합니다. 헬퍼는 시뮬레이션 파티클이 되지 않지만 `strap_02`까지 탐색은 계속됩니다. `bStraightenExcludedBonesByParent`가 활성화되어 있으면 제외된 헬퍼가 주변 시뮬레이션 움직임을 기준으로 재구성됩니다.

체인에서의 제외는 콜라이더의 `ExcludeBones`와 다릅니다. 체인 제외는 시뮬레이션 토폴로지를 바꾸고, 콜라이더 제외는 선택한 시뮬레이션 본과 해당 콜라이더 사이의 접촉만 막습니다.

### `FLKAnimVerletBoneUnitSetting`

`BoneUnitSettingOverride`에 항목을 추가하고 `Bone`을 선택한 뒤 필요한 오버라이드만 활성화합니다.

| 프로퍼티 | 기본값 | 설명 |
|---|---:|---|
| `Bone` | None | 오버라이드할 실제 스켈레탈 본입니다. |
| `bLockBone` | `false` | 파티클을 애니메이션 포즈 위치에 고정합니다. |
| `LockMargin` | `0 cm` | 고정 목표에서 허용되는 거리입니다. 0은 정확한 위치 고정입니다. |
| `bOverrideConstrainConeAngleFromParent` | `false` | 글로벌 원뿔 기준 모드의 로컬 오버라이드를 활성화합니다. |
| `bConstrainConeAngleFromParent` | `false` | 오버라이드 시 애니메이션 포즈 방향 대신 조부모→부모 방향을 원뿔 기준으로 사용합니다. |
| `bOverrideConeAngle` | `false` | 이 본의 로컬 원뿔 제한을 활성화합니다. |
| `ConeAngle` | `0 degrees` | 로컬 Ball-Socket 원뿔 각도이며 `0–90`도로 제한됩니다. 양수인 로컬 값은 글로벌 값보다 우선합니다. |
| `bOverrideConeAngleOffset` | `false` | 이 본에서 노드의 글로벌 원뿔 중심 회전 오프셋을 대체합니다. 로컬 Cone Angle 오버라이드와 독립적으로 활성화할 수 있습니다. |
| `ConeAngleOffset` | Zero Rotator | 이 본의 원뿔 중심 방향에 적용할 로컬 회전입니다. 부모 파티클(`BoneA`)의 애니메이션 포즈 로컬 공간을 사용합니다. |
| `bOverrideThickness` | `false` | 로컬 충돌 반경 오버라이드를 활성화합니다. |
| `Thickness` | `0.3 cm` | 로컬 파티클, 캡슐 또는 표면 두께입니다. |
| `bOverrideToUseSphereCollisionForChain` | `false` | 노드가 캡슐/삼각형 체인 충돌을 사용하더라도 이 유닛은 구 접촉을 사용합니다. 글로벌 체인 캡슐 충돌이 꺼져 있으면 효과가 없습니다. |
| `bOverrideMass` | `false` | 로컬 질량을 활성화합니다. |
| `Mass` | `1.0` | 로컬 질량이며 최소 유효 값은 `0.01`입니다. |

활용 예시:

- 중간 부착점 고정
- 넓은 팁을 더 가볍거나 무겁게 설정
- 헤어 장식 주변의 충돌 두께만 확대
- 문제가 있는 캡슐/삼각형 세그먼트를 구로 교체
- 루트 근처의 원뿔 제한은 좁게, 팁은 느슨하게 설정
- 제작된 스켈레톤 포즈를 바꾸지 않고 특정 본의 원뿔을 뒤쪽이나 옆쪽으로 편향

## 7. Setup 파라미터

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `VerletBones` | 비어 있음 | 시뮬레이션할 루트 계층의 정렬된 목록입니다. 한 항목은 단일 체인, 여러 항목은 천 형태의 측면 관계를 만듭니다. |
| `bSubDivideBones` | `false` | 각 실제 부모-자식 사이에 가상 파티클을 삽입합니다. 스켈레톤을 바꾸지 않고 충돌 해상도와 부드러움을 높입니다. |
| `NumSubDividedBone` | `1` | 세그먼트마다 삽입할 파티클 수입니다. 수가 많을수록 비용과 필요한 반복 횟수가 늘 수 있습니다. |
| `bRebuildSimulationOnLODChange` | `false` | Required Bone LOD가 바뀌면 토폴로지, 제약조건, Broadphase, 충돌 상태를 재구성합니다. 가능한 경우 일치하는 파티클 상태를 보존합니다. |
| `bActivate` | `true` | false이면 노드 평가를 완전히 비활성화합니다. 기본 그래프 핀으로 노출됩니다. |
| `bSkipUpdateOnDedicatedServer` | `true` | Dedicated Server에서 평가를 건너뜁니다. 서버 측 시뮬레이션 트랜스폼이 실제로 필요할 때만 끄십시오. |
| `bPause` | `false` | 현재 시뮬레이션 상태를 유지하고 출력하면서 적분만 중지합니다. 리셋하지 않습니다. 기본 그래프 핀입니다. |
| `PlaySpeedRate` | `1.0` | 업데이트 델타 타임과 Time Dilation을 스케일합니다. 명시적인 일시정지는 `bPause`를 권장합니다. |
| `bUseWarmup` | `true` | 초기화 또는 `ResetPhysics` 뒤 결과를 공개하기 전에 고정 서브스텝을 수행합니다. |
| `WarmupStepCount` | `8` | 워밍업 서브스텝 횟수입니다. 다음 평가되는 비일시정지 프레임에서 수행됩니다. |
| `WarmupFixedDeltaTime` | `0.016666667 s` | 각 워밍업 서브스텝의 델타 타임이며 기본값은 약 60 Hz입니다. |
| `OutputBlendDuration` | `0.2 s` | 초기화/리셋 후 입력 포즈에서 시뮬레이션으로 블렌딩하는 시간입니다. 0이면 즉시 적용합니다. |
| `bMakeFakeTipBone` | `true` | 각 Leaf 뒤에 가상 자식을 추가하여 마지막 실제 본의 회전과 팁 충돌을 계산합니다. |
| `FakeTipBoneLength` | `10 cm` | 가상 팁 세그먼트 길이입니다. 마지막 실제 본의 시각적 연장 길이에 맞춥니다. |
| `bLockTipBone` | `false` | 실제 Leaf 파티클을 고정합니다. 활성화하면 가상 팁을 만들지 않습니다. |
| `TipBoneLockMargin` | `0 cm` | 고정된 팁의 허용 이동 반경입니다. |
| `StartBoneLockMargin` | `0 cm` | 자동 고정되는 각 체인 루트의 허용 이동 반경입니다. |

### Activate, Pause, Reset, Warmup의 차이

- **Deactivate**: 스켈레탈 컨트롤 자체가 평가되지 않습니다.
- **Pause**: 유지된 상태를 계속 출력하지만 적분하지 않습니다.
- 에디터의 **Reset Simulation**: 움직임을 지우고 파티클을 현재 포즈에 동기화합니다.
- Unreal 애니메이션 다이내믹스 흐름의 **Reset Physics**: 현재 포즈/컴포넌트 트랜스폼 기준 리셋을 예약합니다. 워밍업이 활성화되어 있으면 다음 평가 프레임에 워밍업합니다.

워밍업은 모든 서브스텝에 같은 컴포넌트 트랜스폼을 사용하므로 컴포넌트 이동 및 회전 관성을 의도적으로 무시합니다.

## 8. 애니메이션 포즈 설정

별도 언급이 없는 한 이 파라미터들은 `bIgnoreAnimationPose`가 false일 때만 동작합니다.

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `AnimationPoseDeltaInertia` | `0.03` | 프레임별 입력 포즈 이동 중 파티클에 전달되는 비율입니다. |
| `AnimationPoseDeltaInertiaScale` | `1.0` | 포즈 델타 전달의 추가 배율입니다. |
| `bClampAnimationPoseDeltaInertia` | `true` | 전달되는 포즈 델타를 제한합니다. 텔레포트, 애니메이션 점프, 몽타주 전환에 권장합니다. |
| `AnimationPoseDeltaInertiaClampMax` | `0.1` | Clamp 사용 시 업데이트마다 허용되는 최대 포즈 델타 변위입니다. |
| `bIgnoreAnimationPose` | `false` | 포즈 추종 보정을 비활성화하고 제약조건과 힘이 주로 형상을 결정하게 합니다. |
| `bAlignAnimationPoseToGravity` | `false` | 애니메이션 포즈를 `AnimationPoseReferenceDirection`에서 현재 중력 방향으로 회전시킨 뒤 목표로 사용합니다. |
| `AnimationPoseInertia` | `0.03` | 현재 부모 상대 애니메이션 포즈 위치로 직접 당기는 정도입니다. 높을수록 애니메이션을 더 밀착 추종합니다. |
| `bApplyAnimationPoseInertiaCorrection` | `true` | 목표 프레임레이트와 제한된 평균 FPS로 `AnimationPoseInertia`를 보정합니다. |
| `AnimationPoseInertiaTargetFrameRate` | `60` | 포즈 관성 보정의 기준 프레임레이트입니다. |
| `AnimationPoseReferenceDirection` | Down `(0,0,-1)` | 제작된 애니메이션 포즈에서 중력을 나타내는 컴포넌트 공간 방향입니다. 중력 정렬 옵션에 사용됩니다. |

중력 정렬은 한 컴포넌트 방향으로 늘어진 상태에서 제작한 에셋이 캐릭터 회전에 따라 월드 중력 방향과 달라질 때 유용합니다. 기준 벡터 또는 중력이 0이면 정렬 회전은 비활성화됩니다.

## 9. Solver 및 Constraint 파라미터

### 핵심 Solver

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `Damping` | `0.9` | 이전 Verlet 변위 중 유지할 비율입니다. `1`은 모두 유지하고 낮을수록 움직임을 빨리 제거합니다. |
| `bApplyDampingCorrection` | `false` | 평균 FPS를 기준으로 Damping을 보정하여 감쇠 속도를 더 일정하게 만듭니다. |
| `DampingCorrectionTargetFrameRate` | `60` | Damping 보정 기준 FPS입니다. |
| `bUseXPBDSolver` | `false` | XPBD 제약조건 Solver를 선택합니다. false이면 PBD입니다. |
| `InvCompliance` | `100000000` | XPBD 역 컴플라이언스입니다. 높을수록 거리 제약조건이 단단합니다. 실제 컴플라이언스는 `1 / InvCompliance`이므로 0으로 두지 마십시오. |
| `Stiffness` | `0.8` | XPBD가 꺼졌을 때 사용하는 PBD 거리 강성입니다. |
| `SolveIteration` | `4` | 시뮬레이션 업데이트당 제약조건 반복 횟수입니다. 높을수록 수렴도가 좋아지고 비용도 늘어납니다. 최소값은 `1`입니다. |
| `bUseSquaredDeltaTime` | `false` | 중력과 힘 항에 `dt` 대신 `dt²`을 적용합니다. 물리 프리셋이 사용하는 적분 모드입니다. |

`InvCompliance`, `Stiffness`, Solver 종류, 토폴로지는 시뮬레이션 제약조건이 구성될 때 캡처됩니다. 에디터에서는 프로퍼티를 바꾸면 프리뷰 시뮬레이션이 지워지고 다시 구성됩니다. 런타임에서 토폴로지나 Solver를 변경할 때는 Dynamics Reset 또는 노드 재초기화가 필요하다고 간주하십시오.

### Sleep

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `bUseSleep` | `true` | 거의 정지한 파티클의 모션 상태 업데이트를 중지할 수 있게 합니다. |
| `bIgnoreSleepWhenParentWakedUp` | `true` | 부모가 깨어 있는 동안 자식도 강제로 깨웁니다. 하위 구간이 얼어붙는 현상을 방지합니다. |
| `SleepDeltaThreshold` | `0.05 cm` | 정지 상태로 간주할 업데이트당 최대 변위입니다. |
| `SleepTriggerDuration` | `5 s` | Sleep에 들어가기 전에 계속 정지 상태여야 하는 시간입니다. |
| `WakeUpDeltaThreshold` | `0.1 cm` | 잠든 파티클을 깨우는 변위입니다. |

컴포넌트 프레임이 움직이면 파티클이 깨어나므로 느린 액터 이동이 Sleep에 의해 사라지지 않습니다.

### 길이, 측면, 대각선, 펴기 제약조건

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `bConstrainRightDiagonalDistance` | `false` | 이웃 체인 사이에 한 방향 대각선 제약조건을 추가합니다. 애니메이션 포즈를 무시할 때 유용합니다. |
| `bConstrainLeftDiagonalDistance` | `false` | 반대 방향 대각선을 추가합니다. 둘 다 켜면 그리드의 전단 저항이 강해집니다. |
| `bPreserveLengthFromParent` | `true` | 모든 부모-자식 세그먼트에 최종 고정 거리 보정을 적용합니다. 적은 반복 횟수로도 길이를 유지하는 데 도움이 됩니다. |
| `bPreserveLengthFromParentBetweenRealBones` | `false` | Subdivision 사용 시 삽입된 가상 파티클을 가로질러 원래 실제 부모-자식 길이도 유지합니다. |
| `LengthFromParentMargin` | `0.1 cm` | 부모-자식 고정 길이 허용 오차입니다. |
| `bPreserveSideLength` | `true` | 이웃 체인 사이에 고정 거리 보정을 적용합니다. 다중 체인 천에 사용됩니다. |
| `bPreserveSideLengthBetweenRealBones` | `false` | Subdivision 사용 시 체인 사이의 대응되는 실제 본도 제약합니다. |
| `SideLengthMargin` | `0.1 cm` | 측면 고정 길이 허용 오차입니다. |
| `bStretchEachBone` | `false` | 거리 제약조건이 각 세그먼트의 제작된 길이를 명시적으로 참조하게 합니다. |
| `StretchStrength` | `1.0` | 세그먼트별 Stretch 동작에 사용되는 강도입니다. |
| `bStraightenBendedBone` | `false` | 애니메이션 포즈 방향을 목표로 사용하지 않는 3-파티클 펴기 제약조건을 추가합니다. |
| `StraightenBendedBoneStrength` | `0.0003` | 구부러진 본을 펴는 강도입니다. 작은 값 변화도 큰 차이를 만들 수 있습니다. |

### Isometric Bending

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `bUseIsometricBendingConstraint` | `false` | 벤딩 제약조건을 추가합니다. 단일 체인에는 1D 3-파티클 벤딩을, 다중 체인에는 표면 벤딩 관계를 만듭니다. |
| `InvBendingCompliance` | `10000` | 고정 XPBD 벤딩 역 컴플라이언스입니다. |
| `bUseBendingComplianceRange` | `false` | 접힘 각도에 따라 XPBD 벤딩 강도를 변화시킵니다. |
| `InvBendingComplianceMin` | `10000` | 초기 각도 근처의 역 컴플라이언스입니다. 구현은 Min/Max 중 낮은 값을 Rest 값으로 사용합니다. |
| `InvBendingComplianceMax` | `80000` | 설정된 최대 접힘 각도 이상에서의 역 컴플라이언스입니다. |
| `BendingComplianceMaxAngle` | `120 degrees` | 최대 XPBD 벤딩 강도에 도달하는 접힘 각도입니다. |
| `BendingStiffness` | `0.01` | 고정 PBD 벤딩 강성입니다. |
| `bUseBendingStiffnessRange` | `false` | 접힘 각도에 따라 PBD 벤딩 강성을 변화시킵니다. |
| `BendingStiffnessMin` | `0.01` | 초기 각도 근처의 PBD 강성입니다. |
| `BendingStiffnessMax` | `0.1` | 설정된 최대 접힘 각도에서의 PBD 강성입니다. |
| `BendingStiffnessMaxAngle` | `120 degrees` | 최대 PBD 벤딩 강성에 도달하는 접힘 각도입니다. |

내부 Flat Bending 필드는 현재 편집 가능한 `UPROPERTY`로 노출되지 않으므로 일반적인 에디터 워크플로의 일부로 간주하지 마십시오.

### Delta Time 제어

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `FixedDeltaTime` | `0 s` | 양수이면 실제 델타 타임을 대체합니다. 0이면 업데이트 델타를 사용합니다. |
| `bApplyDeltaTimeCorrection` | `true` | Fixed Delta 사용 시 프레임레이트, Time Dilation, 목표 프레임레이트로 값을 보정합니다. |
| `DeltaTimeCorrectionTargetFrameRate` | `60` | Fixed Delta 보정 기준 FPS입니다. |
| `MinDeltaTime` | `KINDA_SMALL_NUMBER` | 최종 시뮬레이션 델타에 적용되는 하한입니다. |
| `MaxDeltaTime` | `0.05 s` | 프레임 Hitch 뒤 불안정성을 제한하는 상한입니다. |

`PlaySpeedRate`는 입력 델타 타임과 Time Dilation 양쪽에 관여합니다. 게임플레이 시스템이 Global 또는 Custom Time Dilation을 변경한다면 Slow Motion과 Fast Forward를 직접 테스트하십시오.

### Cone 및 Custom Distance 제약조건

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `bConstrainConeAngleFromParent` | `false` | 움직이는 조부모→부모 방향을 글로벌 원뿔 축으로 사용합니다. false이면 제작된 애니메이션 포즈 방향을 사용합니다. |
| `ConeAngle` | `0 degrees` | 글로벌 Ball-Socket 원뿔 제한입니다. 0은 비활성화이며 에디터 범위는 `0–90`도입니다. |
| `ConeAngleOffset` | Zero Rotator | 원뿔 중심 방향에 적용하는 글로벌 회전 오프셋입니다. `bOverrideConeAngleOffset`이 활성화되지 않은 본은 이 값을 상속합니다. |
| `CustomDistanceConstraints` | 비어 있음 | 수동 본 쌍 최소/최대 거리 제약조건입니다. |

`ConeAngleOffset`은 허용 원뿔의 중심축을 바꾸며 시뮬레이션 본을 직접 회전시키거나 `ConeAngle` 값을 변경하지 않습니다. 런타임은 먼저 애니메이션 포즈 방향 또는 조부모→부모 방향에서 기본 원뿔 방향을 구합니다. 그 방향을 부모 파티클(`BoneA`)의 애니메이션 포즈 로컬 공간으로 변환하고 Offset Rotator를 적용한 뒤 다시 원래 공간으로 변환합니다. 따라서 오프셋은 컴포넌트 또는 월드 축에 고정되지 않고 스켈레탈 방향을 따라갑니다.

글로벌 오프셋은 시뮬레이션 구성 시 실제 파티클과 생성된 Fake Tip 파티클에 복사됩니다. Bone Unit Offset Override는 선택된 실제 본과 해당 Leaf에서 생성된 Fake Tip의 글로벌 값을 대체합니다. 유효한 글로벌 또는 본별 Cone Angle이 0보다 클 때만 오프셋이 제약조건에 시각적인 영향을 줍니다.

### 튜토리얼: 망토 원뿔을 등 바깥쪽으로 편향

1. `ConeAngle`을 `45`도와 같은 양수로 설정합니다.
2. 프리뷰에서 `bShowConstraints`와 `bShowSimulatingBallSocketConstraints`를 활성화합니다.
3. 자홍색 Wire Cone이 원하는 방향으로 열릴 때까지 글로벌 `ConeAngleOffset`을 작은 단위로 조정합니다.
4. 바깥쪽 또는 어깨 체인만 다른 방향이 필요하면 해당 Root나 상단 본을 `BoneUnitSettingOverride`에 추가하고 `bOverrideConeAngleOffset`을 활성화한 뒤 로컬 `ConeAngleOffset`을 설정합니다.
5. 프리뷰 시뮬레이션을 리셋하고 여러 애니메이션 포즈에서 테스트합니다.

Pitch, Yaw, Roll은 각 부모 본의 애니메이션 포즈 로컬 축을 기준으로 해석되므로 유용한 값은 스켈레톤 제작 방향에 따라 달라집니다. 특정 Euler 성분이 항상 캐릭터의 앞이나 뒤를 의미한다고 가정하지 말고 Wire Cone 프리뷰를 사용하십시오.

각 `FLKAnimVerletCustomDistanceConstraintSetting`은 다음 값을 가집니다.

| 프로퍼티 | 의미 |
|---|---|
| `BoneA`, `BoneB` | 제약조건 양 끝점입니다. 시뮬레이션 내부 본은 시뮬레이션 파티클을 사용합니다. 설정 체인 밖의 유효한 본은 현재 애니메이션 포즈를 따르는 고정 Anchor가 됩니다. |
| `MinDistance` | 허용되는 최소 거리(cm)입니다. |
| `MaxDistance` | 허용되는 최대 거리(cm)입니다. |

두 거리가 모두 0이면 초기 애니메이션 포즈 거리가 최소와 최대가 되어 정확한 Rest Distance를 유지합니다. 같은 본을 두 끝점에 사용하지 마십시오. 유효하지 않거나 현재 LOD에 없는 외부 Anchor 본은 건너뜁니다.

시뮬레이션된 스트랩을 가슴 부착점 근처에 유지하는 예:

```text
BoneA       = strap_03
BoneB       = spine_03
MinDistance = 8 cm
MaxDistance = 15 cm
```

`spine_03`은 `VerletBones` 안에 있을 필요가 없으며, 고정 Anchor로서 애니메이션 포즈를 따릅니다.

## 10. 충돌 설정 및 데이터 소스

AnimVerlet은 다음 소스를 모두 함께 사용할 수 있습니다.

1. 노드의 직접 배열: Sphere, Capsule, Box, Plane
2. `CollisionDataAsset`
3. `CollisionPhysicsAsset`
4. 매 프레임 전달하는 `DynamicCollisionShapes`
5. `WorldCollisionProfile`을 통한 월드 충돌
6. 자기 충돌

처음 세 정적 소스는 초기화 시 합산됩니다. 직접 Shape가 먼저 복사되고, Data Asset Shape와 Physics Asset Shape가 뒤에 추가됩니다. `DynamicCollisionShapes`는 매 프레임 별도로 평가되며 이것도 기존 Shape에 추가됩니다. 중복 접촉이 의도된 것이 아니라면 같은 콜라이더를 여러 소스에서 전달하지 마십시오.

### 글로벌 충돌 파라미터

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `bUseBroadphase` | `true` | 가속 구조를 사용하여 충돌 후보 검사를 줄입니다. 단순하지 않은 구성에서는 켜 두십시오. |
| `Thickness` | `0.3 cm` | 글로벌 시뮬레이션 파티클 반경 및 체인/표면 두께입니다. |
| `FrictionCoefficient` | `0` | PBD와 XPBD 모두에서 PBD 방식 충돌 Projection에 사용하는 Coulomb 마찰 계수입니다. |
| `bUseCapsuleCollisionForChain` | `true` | 단일 체인은 부모-자식 캡슐을, 다중 체인은 삼각형을 사용합니다. false이면 구를 사용합니다. |
| `bUseSelfCollision` | `false` | 시뮬레이션 본/표면 사이의 충돌을 활성화합니다. 비용이 클 수 있습니다. |
| `bUseTriangleSelfCollision` | `false` | 다중 체인에서 구-삼각형 대신 삼각형-삼각형 자기 충돌을 선택합니다. 단일 체인은 이 값과 관계없이 캡슐을 사용합니다. |
| `SelfCollisionAdditionalThickness` | `0.1 cm` | 자기 접촉에 추가할 분리 반경입니다. |
| `WorldCollisionProfile` | None | None이 아닌 프로파일을 설정하면 해당 Unreal Collision Profile에 대한 월드 Sweep 충돌이 활성화됩니다. |
| `WorldCollisionExcludeBones` | 비어 있음 | 월드 충돌에서 제외할 시뮬레이션 본입니다. |
| `SphereCollisionShapes` | 비어 있음 | 직접 입력하는 로컬/월드 구 콜라이더입니다. |
| `CapsuleCollisionShapes` | 비어 있음 | 직접 입력하는 로컬/월드 캡슐 콜라이더입니다. |
| `BoxCollisionShapes` | 비어 있음 | 직접 입력하는 로컬/월드 박스 콜라이더입니다. |
| `PlaneCollisionShapes` | 비어 있음 | 직접 입력하는 유한/무한 평면 콜라이더입니다. |
| `CollisionDataAsset` | None | 공유 가능한 AnimVerlet 콜라이더 데이터입니다. 노드 도구를 통해 편집하고 프리뷰할 수 있습니다. |
| `CollisionPhysicsAsset` | None | 런타임에 지원되는 Physics Asset Primitive를 가져옵니다. Collision Data Asset처럼 AnimVerlet 프리뷰에서 수정할 수는 없습니다. |
| `DynamicCollisionShapes` | 비어 있음 | Animation Blueprint나 외부 코드에서 매 프레임 충돌을 전달할 때 사용하는 그래프 핀입니다. |

월드 충돌은 Physics Sweep을 수행하므로 소수의 로컬 Shape를 쓰는 것보다 일반적으로 비쌉니다. 월드 충돌에서는 다중 체인 삼각형 형상 대신 구/캡슐 방식의 체인 검사를 사용합니다.

### 공통 콜라이더 필드

모든 직접 또는 동적 콜라이더에는 다음 값이 있습니다.

| 프로퍼티 | 설명 |
|---|---|
| `bUseAbsoluteWorldTransform` | false이면 본 부착 오프셋입니다. true이면 `LocationOffset`과 해당 Shape의 `RotationOffset`을 절대 월드 트랜스폼으로 사용합니다. |
| `AttachedBone` | 절대 월드 모드가 false일 때 기준이 되는 본입니다. |
| `ExcludeBones` | 이 Shape와 충돌하지 않을 시뮬레이션 본입니다. |
| `LocationOffset` | 본 상대 위치 오프셋 또는 월드 모드의 절대 월드 위치입니다. |

Shape별 필드:

| Shape | 필드 |
|---|---|
| Sphere | `Radius` |
| Capsule | `RotationOffset`, `Radius`, `HalfHeight` |
| Box | `RotationOffset`, `HalfExtents` |
| Plane | `RotationOffset`, `bFinitePlane`, `FinitePlaneHalfExtents` |

Plane은 회전된 로컬 Up/Z 방향을 Normal로 사용합니다. `bFinitePlane = false`이면 무한 평면이고, 유한 평면은 XY Half Extent를 사용합니다.

### 본 부착 콜라이더와 절대 월드 콜라이더

캐릭터 신체에는 본 부착 콜라이더를 사용합니다.

```text
bUseAbsoluteWorldTransform = false
AttachedBone               = pelvis
LocationOffset             = pelvis 기준 로컬 오프셋
```

이미 월드 공간으로 표현된 외부 오브젝트에는 절대 월드 콜라이더를 사용합니다.

```text
bUseAbsoluteWorldTransform = true
LocationOffset             = 오브젝트 월드 위치
RotationOffset             = 오브젝트 월드 회전
```

절대 월드 콜라이더는 매 평가 시 시뮬레이션 컴포넌트 공간으로 변환됩니다.

### Collision Data Asset 워크플로

1. 클래스가 `LKAnimVerletCollisionDataAsset`인 Data Asset을 생성합니다.
2. 노드의 `CollisionDataAsset`에 할당합니다.
3. Asset의 Sphere/Capsule/Box/Plane 데이터를 직접 편집하거나 노드에 직접 Shape를 먼저 만듭니다.
4. **AnimVerlet Tool > Convert Collision To DataAsset**으로 노드의 직접 Shape를 Asset에 복사합니다.
5. 변환 뒤 Data Asset을 직접 저장합니다.

그 밖의 에디터 변환 버튼:

| 버튼 | 결과 |
|---|---|
| `Convert Collision From DataAsset` | 노드의 직접 Shape 배열을 지운 뒤 Data Asset 내용을 복사합니다. Animation Blueprint를 직접 저장해야 합니다. |
| `Convert Collision From PhysicsAsset to DataAsset` | 대상 Data Asset을 지우고 Physics Asset Primitive 변환 결과로 교체합니다. Data Asset을 직접 저장해야 합니다. |
| `Convert Collision From PhysicsAsset` | 노드의 직접 Shape를 지운 뒤 지원되는 Physics Asset Primitive를 복사합니다. Animation Blueprint를 직접 저장해야 합니다. |

Physics Asset 변환은 Sphere, Capsule/Sphyl, Box를 지원합니다. Plane은 AnimVerlet 전용이므로 별도로 작성해야 합니다.

### 런타임 동적 충돌 튜토리얼

AnimVerlet이 무기, 방패, 차량 부품 또는 다른 이동 컴포넌트와 충돌해야 한다면 `DynamicCollisionShapes`를 사용합니다.

Blueprint 흐름:

```text
Moving Primitive Component
        │
        ▼
Make Collision Shape List From Primitive Component
        │
        ▼
AnimVerlet.DynamicCollisionShapes
```

`ULKAnimVerletBlueprintFunctionLibrary`에서 제공하는 Blueprint 헬퍼:

| 함수 | 용도 |
|---|---|
| `MakeCollisionShapeListFromPrimitiveComponent` | 하나의 Primitive Component Body Instance로부터 절대 월드 Shape를 만듭니다. |
| `MakeCollisionShapeListFromSkeletalMeshComponent` | Skeletal Mesh Component가 소유한 모든 Body Instance에서 Shape를 만듭니다. |
| `MakeCollisionShapeListFromCollisionDataAsset` | Skeletal Mesh Component의 현재 본 트랜스폼으로 Data Asset을 평가하여 절대 월드 Shape를 출력합니다. |
| `MakeCollisionShapeListFromPhysicsAsset` | Skeletal Mesh Component를 기준으로 Physics Asset을 평가하여 절대 월드 Shape를 출력합니다. |
| `MakeCollisionShapeListFromCollisionShapeList` | 현재 스켈레탈 트랜스폼을 사용하여 본 부착 Shape를 절대 월드 Shape로 변환합니다. |

AnimVerlet이 데이터를 소비하는 Animation Blueprint 평가 전에 목록을 갱신해야 합니다. 데이터가 캐릭터나 다른 게임플레이 오브젝트에서 온다면 프로젝트의 애니메이션 업데이트 구조에 맞추어 생성된 `FLKAnimVerletCollisionShapeList`를 Thread-Safe Animation Blueprint 입력 변수에 저장하고 노드 핀에 연결하십시오.

## 11. Gravity, Force, Wind

### Gravity

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `Gravity` | `(0,0,0)` | 중력 형태의 변위 항입니다. 프리셋은 선형 델타 애니메이션 모드에 `-9.8`, 제곱 델타 물리 모드에 `-980`을 사용합니다. |
| `bGravityInWorldSpace` | `true` | 매 업데이트에 월드 중력을 컴포넌트 공간으로 변환합니다. false이면 컴포넌트 공간 값으로 직접 해석합니다. |

### 기타 Force

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `StretchForce` | `0` | 각 파티클을 제작된 부모→자식 방향으로 밉니다. |
| `bAlignStretchForceToGravity` | `false` | Stretch에 중력 정렬된 제작 방향을 사용합니다. |
| `SideStraightenForce` | `0` | 체인 루트의 위치 관계에서 계산한 측면 힘을 적용합니다. 다중 체인 배치에 사용됩니다. |
| `ShapeMemoryForce` | `0` | 파티클을 애니메이션 포즈 목표 방향으로 밉니다. `AnimationPoseInertia`와 달리 힘 형태의 항으로 적용됩니다. |
| `bAlignShapeMemoryForceToGravity` | `false` | 중력 정렬된 포즈를 Shape Memory 목표로 사용합니다. |
| `ExternalForce` | `(0,0,0)` | 사용자가 전달하는 상수 힘 벡터입니다. |
| `bExternalForceInWorldSpace` | `true` | External Force를 월드에서 컴포넌트 공간으로 변환합니다. |

현재 구현에서 중력을 제외한 Force 반응은 역질량으로 스케일됩니다. `bUseSquaredDeltaTime = false`이면 힘 항에 `dt`가, 활성화되어 있으면 `dt²`이 곱해집니다. 서로 다른 프리셋 사이에서 값을 그대로 복사하기보다 선택한 적분 모드 안에서 튜닝하십시오.

### Random Wind

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `RandomWindDirection` | `(0,0,0)` | 기본 랜덤 힘 방향입니다. 0 벡터이면 비활성화됩니다. |
| `RandomWindSizeMin` | `0` | 각 파티클/업데이트마다 독립적으로 샘플링하는 최소 크기입니다. |
| `RandomWindSizeMax` | `0` | 최대 크기입니다. 직관적인 결과를 위해 Min이 Max보다 작거나 같게 설정합니다. |
| `bRandomWindDirectionInWorldSpace` | `true` | 방향을 월드 공간으로 해석합니다. false이면 컴포넌트 공간입니다. |
| `AdditionalRandomWinds` | 비어 있음 | 독립적으로 샘플링되는 추가 방향성 랜덤 힘 레코드입니다. |
| `bAdjustWindComponent` | `false` | 월드 Scene의 Unreal `UWindDirectionalSourceComponent` 바람을 샘플링합니다. |
| `WindComponentScale` | `1.0` | 노출된 Wind Component 스케일입니다. 현재 런타임 업데이트 경로에서는 저장 및 동기화되지만 샘플링된 Scene Wind에 곱해지지 않으므로 이 값만 바꾸어도 결과는 변하지 않습니다. |

각 `AdditionalRandomWinds` 항목에는 `RandomForceDirection`, `RandomForceSizeMin`, `RandomForceSizeMax`, `bRandomForceDirectionInWorldSpace`가 있습니다.

랜덤 힘 방향 벡터는 시뮬레이션 업데이트에서 정규화되지 않으므로 벡터 길이도 최종 효과를 스케일합니다. Min/Max 필드만 크기를 제어하게 하려면 단위 방향 벡터를 사용하십시오.

## 12. 컴포넌트 관성

컴포넌트 관성은 소유 Skeletal Mesh Component의 월드 이동 또는 회전에 의해 발생하는 겉보기 세컨더리 모션입니다.

| 프로퍼티 | 기본값 | 사용 방법 |
|---|---:|---|
| `MoveInertiaScale` | `1.0` | 컴포넌트 이동 관성 배율입니다. `0`이면 이동으로 인한 지연을 제거합니다. |
| `bIgnoreSuddenMoveInertia` | `false` | 이동이 임계값을 넘으면 해당 프레임의 이동 관성을 버립니다. 텔레포트에 유용합니다. |
| `MoveInertiaIgnoreThreshold` | `800 cm` | 급격한 이동 거리 임계값입니다. |
| `bClampMoveInertia` | `true` | 컴포넌트 이동을 무한히 허용하지 않고 상한으로 제한합니다. |
| `MoveInertiaClampMaxDistance` | `300 cm` | 허용할 최대 이동 변위입니다. |
| `RotationInertiaScale` | `1.0` | 컴포넌트 회전 관성 배율입니다. |
| `bIgnoreSuddenRotationInertia` | `false` | 임계값을 넘는 프레임의 회전 관성을 버립니다. |
| `RotationInertiaIgnoreDegrees` | `90 degrees` | 급격한 회전 임계값입니다. |
| `bClampRotationInertia` | `true` | 허용할 컴포넌트 회전을 제한합니다. |
| `RotationInertiaClampDegrees` | `30 degrees` | 업데이트당 최대 허용 회전입니다. |
| `ComponentInertiaTangentialDamping` | `1.0` | 컴포넌트 관성 적용 뒤 유지할 부모 상대 접선 속도 비율입니다. `1`은 모두 유지하고 `0`은 제거합니다. 값은 60 Hz 스텝당 유지율로 취급되어 현재 델타 타임에 맞게 보정됩니다. |

게임플레이 텔레포트에는 Unreal의 `ResetPhysics` 흐름으로 새 포즈에 동기화하는 것이 가장 깔끔합니다. Ignore/Clamp 설정은 별도 통지 없이 발생한 큰 컴포넌트 델타에 대한 추가 보호 장치입니다.

## 13. Preview 및 에디터 컨트롤

다음 프로퍼티들은 에디터 그래프 노드에 속하며 런타임 시뮬레이션이 아닌 시각화에만 영향을 줍니다.

### Collision Input 표시

| 프로퍼티 | 기본값 | 목적 |
|---|---:|---|
| `bShowAndModifySphereCollision` | `true` | 직접/Data Asset/Physics Asset Sphere 입력을 표시하고 지원되는 선택 및 편집을 허용합니다. |
| `bShowAndModifyCapsuleCollision` | `true` | Capsule 입력을 표시합니다. |
| `bShowAndModifyBoxCollision` | `true` | Box 입력을 표시합니다. |
| `bShowAndModifyPlaneCollision` | `true` | Plane 입력을 표시합니다. |
| `bShowCollisionAssetSource` | `true` | 할당된 Asset에서 온 충돌 소스를 표시합니다. |

### 시뮬레이션 디버그 표시

| 프로퍼티 | 기본값 | 목적 |
|---|---:|---|
| `bShowBones` | `true` | 시뮬레이션 파티클/본을 그립니다. |
| `BoneThicknessRenderScale` | `1.0` | Thickness의 디버그 렌더링만 스케일합니다. |
| `bShowCapsuleBoneChainConstraints` | `true` | 선택한 모드에 따라 Capsule/Triangle/Sphere 체인 형상을 그립니다. |
| `bShowSleep` | `true` | Sleep 상태 시각화를 사용합니다. |
| `bShowBoneBounds` | `false` | Broadphase Bound를 그립니다. |
| `bShowConstraints` | `true` | 활성 제약조건 시각화의 마스터 옵션입니다. |
| `bShowDistanceConstraintLengths` | `true` | 모든 활성 `FLKAnimVerletConstraint_Distance`의 현재 끝점 거리를 중간 지점에 cm 단위 소수점 둘째 자리로 표시합니다. `bShowConstraints`가 false이면 숨겨집니다. |
| `bShowFixedPoints` | `true` | 고정점과 루트를 그립니다. |
| `bShowSimulatingBallSocketConstraints` | `true` | Cone/Ball-Socket 제약조건을 그립니다. |
| `bShowSimulatingSphereCollisionConstraints` | `false` | 활성 Sphere 충돌 제약조건을 그립니다. |
| `bShowSimulatingSphereCollisionConstraintsBounds` | `false` | 해당 Bound를 그립니다. |
| `bShowSimulatingCapsuleCollisionConstraints` | `false` | 활성 Capsule 충돌 제약조건을 그립니다. |
| `bShowSimulatingCapsuleCollisionConstraintsBounds` | `false` | 해당 Bound를 그립니다. |
| `bShowSimulatingBoxCollisionConstraints` | `false` | 활성 Box 충돌 제약조건을 그립니다. |
| `bShowSimulatingBoxCollisionConstraintsBounds` | `false` | 해당 Bound를 그립니다. |
| `bShowSimulatingPlaneCollisionConstraints` | `false` | 활성 Plane 충돌 제약조건을 그립니다. |
| `bShowIsometricBendingConstraints` | `false` | Bending 관계를 그립니다. |

파라미터 변경 뒤 이전 Momentum이 남았거나 같은 초기 포즈에서 두 설정을 비교하려면 **Preview > Reset Simulation**을 사용하십시오.

모든 구조적 부모-자식 제약조건, 다중 체인의 측면/대각선 제약조건, 유효한 Custom Distance Constraint가 런타임 Distance Constraint 목록에 저장되므로 거리 오버레이에는 이들이 모두 포함됩니다. 현재 간격을 제작된 거리 또는 Custom Min/Max 목표와 비교할 때 사용하십시오. 조밀한 천 그리드에서 라벨이 다른 디버그 표시를 가린다면 끌 수 있습니다. 에디터 전용 표시이므로 런타임 시뮬레이션 결과에는 영향을 주지 않습니다.

## 14. Preset 레퍼런스

| 프리셋 | 포즈 추종 | Solver | Delta 모드 | Bending | Damping | 반복 | Gravity |
|---|---|---|---|---|---:|---:|---|
| `Custom` | 변경하지 않음 | 변경하지 않음 | 변경하지 않음 | 변경하지 않음 | 변경하지 않음 | 변경하지 않음 | 변경하지 않음 |
| `AnimationPose` | 활성 | PBD | 선형 `dt` | 꺼짐 | `0.8` | `2` | `(0,0,-9.8)` |
| `Physics_XPBD` | 무시 | XPBD | 제곱 `dt` | 켜짐 | `0.99` | `4` | `(0,0,-980)` |
| `Physics_PBD` | 무시 | PBD | 제곱 `dt` | 꺼짐 | `0.9` | `4` | `(0,0,-980)` |

프리셋은 표에 나열된 값만 변경합니다. 본, 충돌 Shape, 질량, 컴플라이언스, 원뿔 제한, Subdivision, 상속된 Skeletal Control Alpha는 설정하지 않습니다.

## 15. 튜닝 순서

과도한 Solver 비용으로 토폴로지 문제를 가리지 않도록 다음 순서로 튜닝하십시오.

1. **Topology**: 올바른 Root, 배열 순서, Exclusion, 체인 길이 대응을 확인합니다.
2. **Integration Style**: Animation Pose, XPBD Physics, PBD Physics 중 하나를 선택합니다.
3. **Rest Shape**: Fake Tip, Pose Following, Cone Limit, Length/Side Constraint를 조정합니다.
4. **Collision Geometry**: Thickness, Collider Transform, Collider별 Exclusion을 조정합니다.
5. **Material Response**: Stiffness/Compliance, Bending, Damping, Mass를 조정합니다.
6. **External Motion**: Gravity, Force, Wind, Component Inertia를 조정합니다.
7. **Robustness**: Delta Clamp, Warmup, Reset, Sleep, LOD Rebuild를 설정합니다.
8. **Performance**: Broadphase, Solver Iteration, Subdivision, World Collision, Self Collision을 조정합니다.

## 16. 문제 해결

### 아무것도 움직이지 않음

- `bActivate`가 true이고 상속된 노드 Alpha가 0보다 큰지 확인합니다.
- `bPause`가 false인지 확인합니다.
- 현재 LOD에서 모든 `RootBone`이 유효한지 확인합니다.
- 루트는 고정됩니다. 하위 본이 있는 체인을 사용하십시오.
- 모든 파티클을 Override 또는 Tip/Root Lock으로 고정하지 않았는지 확인합니다.
- 시스템을 움직일 Gravity, Force, Animation Motion 또는 Component Movement를 추가합니다.

### 노드 출력이 보이지 않음

- 본 토폴로지 변경 후 Animation Blueprint를 컴파일합니다.
- 노드가 최종 포즈 경로에 연결되어 있는지 확인합니다.
- 상속된 Skeletal Control LOD Threshold와 Alpha를 확인합니다.
- 오프셋 프록시 토폴로지가 특별히 필요한 것이 아니라면 유일한 체인에 `bFakeBone`을 사용하지 마십시오.
- 현재 LOD의 Required Bone Container에 시뮬레이션 본이 있는지 확인합니다.

### Hitch 또는 Teleport 뒤 체인이 폭발함

- `MaxDeltaTime`을 낮춥니다.
- 텔레포트 후 Unreal의 `ResetPhysics` 동작을 호출합니다.
- Sudden Move/Rotation Ignore 옵션을 활성화합니다.
- 이동 및 회전 Clamp를 켜 둡니다.
- Warmup과 0보다 큰 `OutputBlendDuration`을 사용합니다.
- XPBD 역 컴플라이언스가 0 또는 지나치게 작지 않은지 확인합니다.

### 체인이 너무 늘어남

- `bPreserveLengthFromParent`를 활성화합니다.
- PBD `Stiffness` 또는 XPBD `InvCompliance`를 올립니다.
- `SolveIteration`을 올립니다.
- Force 크기를 줄입니다.
- Subdivision 사용 시 `bPreserveLengthFromParentBetweenRealBones`를 시험합니다.

### 다중 체인 천이 전단되거나 분리됨

- `VerletBones`가 공간 순서대로 정렬되고 깊이가 서로 대응하는지 확인합니다.
- 대각선 제약조건 두 방향을 모두 활성화합니다.
- `bPreserveSideLength`를 활성화합니다.
- 접힘 동작에는 Isometric Bending을 사용합니다.
- 프리뷰에서 Side 및 Bending Constraint를 확인합니다.
- `bShowDistanceConstraintLengths`를 활성화하여 현재 간격이 비정상적인 행이나 대각선을 찾습니다.

### 원뿔 제한이 잘못된 방향을 가리킴

- `bShowSimulatingBallSocketConstraints`를 활성화하고 자홍색 원뿔을 확인합니다.
- `bConstrainConeAngleFromParent`가 움직이는 부모 방향을 사용해야 하는지, 제작된 포즈 방향을 사용해야 하는지 확인합니다.
- `ConeAngleOffset`을 조정합니다. 부모 본의 애니메이션 포즈 로컬 축을 사용한다는 점에 주의하십시오.
- `BoneUnitSettingOverride`에 의도하지 않은 `bOverrideConeAngleOffset`이 있는지 확인합니다.
- 제약조건 설정을 변경한 뒤 프리뷰를 리셋합니다.

### 충돌을 놓침

- `Thickness` 또는 본별 Thickness Override를 올립니다.
- 긴 세그먼트에 Subdivision을 추가합니다.
- 현재 LOD에서 Collider가 유효한 본에 부착되었는지 확인합니다.
- Absolute World Transform이 실제 월드 트랜스폼인지 확인합니다.
- 해당 파티클이 Collider의 `ExcludeBones`에 포함되지 않았는지 확인합니다.
- Geometry를 확인한 뒤에만 `SolveIteration`을 올립니다.
- 형태가 까다로운 세그먼트에는 Sphere Collision Override를 활성화합니다.

### 충돌이 달라붙거나 떨림

- `FrictionCoefficient`를 낮춥니다.
- 여러 소스에서 중복 전달되는 Collider를 제거합니다.
- 서로 모순된 보정을 요구하는 겹친 Collider를 피합니다.
- Self Collision Thickness를 낮춥니다.
- Iteration을 점진적으로 올립니다.
- 적절한 Mass 분포를 사용합니다. 이웃 파티클의 질량 차이가 매우 크면 불안정하게 보일 수 있습니다.

### 프레임레이트에 따라 물리가 달라짐

- 가속도 형태의 동작에는 제곱 델타 타임을 사용하는 Physics Preset을 사용합니다.
- Fixed Delta Time과 Correction을 검토합니다.
- 적절한 경우 Damping 및 Animation Pose Inertia Correction을 활성화합니다.
- `MaxDeltaTime` 상한을 유지합니다.
- 실제 프로젝트 프레임레이트 범위를 테스트합니다. 이 노드는 반복 제약조건, 제한된 평균 FPS 보정, 선택적 Fixed Delta를 함께 사용합니다.

### LOD 전환 시 체인이 깨짐

- `bRebuildSimulationOnLODChange`를 활성화합니다.
- 노드가 평가되는 모든 LOD에 각 Root가 존재하는지 확인합니다.
- Required Bone Set에서 사라지는 외부 Custom Distance Anchor를 피합니다.
- 본이 크게 줄어드는 LOD 전에 상속된 LOD Threshold로 노드를 비활성화하는 방법을 고려합니다.

## 17. 성능 체크리스트

- `bUseBroadphase`를 활성화합니다.
- 소수의 Primitive로 충분하다면 World Sweep보다 로컬 캐릭터 Collider를 우선합니다.
- `SolveIteration = 2–4`에서 시작합니다.
- 충돌 해상도 또는 부드러운 곡률이 필요한 세그먼트에만 Subdivision을 추가합니다.
- 시각적으로 필요할 때만 Self Collision을 활성화합니다.
- 차이가 비용을 정당화하지 않는다면 다중 체인에서 Triangle-Triangle보다 Sphere-Triangle Self Collision을 우선합니다.
- 오랫동안 정지하는 액세서리에는 Sleep을 사용합니다.
- 서버 권한 시뮬레이션 트랜스폼이 필요하지 않다면 Dedicated Server 업데이트를 건너뜁니다.
- 에디터 시각화로 불필요한 Collider, 과도한 Thickness, 중복 Constraint를 찾습니다.

## 18. 권장 시작 레시피

### 부드러운 포니테일

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

### 무거운 꼬리

```text
Preset                         = AnimationPose
Damping                        = 0.9
AnimationPoseInertia           = 0.015
ConeAngle                      = 40 deg
Mass                           = 2.0
Gravity                        = 선택한 Delta 모드에 맞게 조정
SolveIteration                 = 3
```

### 물리 중심 망토

```text
Preset                          = Physics_XPBD
InvCompliance                   = 100000000
bUseIsometricBendingConstraint  = true
InvBendingCompliance            = 10000
bPreserveLengthFromParent       = true
bPreserveSideLength             = true
bConstrainRightDiagonalDistance = true
bConstrainLeftDiagonalDistance  = true
bUseBroadphase                  = true
SolveIteration                  = 4
```

이 값들은 보편적인 Material 정의가 아니라 시작점입니다. 본 간격, 캐릭터 스케일, 애니메이션 속도, Collider 배치, 목표 프레임레이트에 따라 최종 결과가 달라집니다.
