# 导航路网自定义指南

本指南面向需要修复或增强特定区域导航表现的开发者。所有内容按实际场景组织，看完就能动手写。

---

## 快速上手

创建一个 `.cs` 文件，放入 `vnavmesh/Navigation/Customizations/Territories/`（或你的子文件夹），写入：

```csharp
using vnavmesh.Navigation.Customizations.Attributes;

namespace vnavmesh.Navigation.Customizations.Territories;

[CustomizationTerritory(0)]  // 替换为实际地图 ID
public class MyFirstCustomization : NavmeshCustomization
{
    public override int Version => 1;
}
```

`Version` 每次修改自定义后递增即可，系统发现版本号变化会自动触发重建。

**地图 ID 怎么找**：打开 vnavmesh 主界面（通常是 `/vnav`），当前地图 ID 会显示在界面上。也可以用 Dalamud 的数据窗口查 `TerritoryType`。

---

## 场景一：台阶 / 坡道 / 斜面走不上去或卡住

这类问题的根源是导航网格的**台阶高度阈值**或**坡度限制**。

### 1. 调大最大攀爬高度（针对单个台阶）

```csharp
public Z0155CoerthasCentralHighlands()
{
    Settings.AgentMaxClimb = 1.2f; // 默认 0.7f，单位是世界坐标
}
```

这个参数控制"多高的台子还被认为是连贯的路径"。值太小会导致台阶被视作墙壁；值太大可能让角色尝试爬无法爬上的地方。

### 2. 调大最大坡度（针对整个斜坡 / 陡峭地形）

```csharp
public override int Version => 1;

public MyCustomization()
{
    Settings.AgentMaxSlopeDeg = 90f; // 默认 55 度
}
```

适用场景：地形本身很陡但确实可走，比如 Z0815 安穆艾兰新宫附近的悬崖地形。

### 3. 关掉 LedgeSpans 过滤（针对断裂的斜面 / 破损坡道）

```csharp
using vnavmesh.Common.Navigation.Mesh.Runtime;

public MyCustomization()
{
    Settings.Filtering -= NavmeshFilter.LedgeSpans;
}
```

`LedgeSpans` 过滤器会把边缘跨度过大的网格标记为不可走。关掉它可以允许路径从高处走向低处，穿过那些"本该是坡但被过滤掉了"的区域。适用于：
- 破损的斜坡（如 Z1044 天幕魔导城破碎坡道）
- 从船头走到陆地（Z1142 船头到岸）
- 地形过于抽象导致边缘被误杀（Z0815 安穆艾兰）

---

## 场景二：某个物体 / 地板 / 箱子阻挡了路径

你需要把这个物体的碰撞体从场景中移除或标记为不可阻挡。

### 1. 整块碰撞 mesh 移除

```csharp
using vnavmesh.Navigation.Scene;

public override void CustomizeScene(SceneExtractor scene)
{
    scene.Meshes.Remove("bg/ffxiv/fst_f1/dun/f1d5/collision/f1d5_a2_door2.pcb");
}
```

适用于：门、碎石、泥浆等不想被导航网格纳入计算的几何体。

### 2. 清除 mesh 的所有实例（保留 mesh 定义，清除放置实例）

```csharp
public override void CustomizeScene(SceneExtractor scene)
{
    foreach (var (key, mesh) in scene.Meshes)
    {
        if (key.StartsWith("bg/ffxiv/roc_r1/rad/r1r1/collision/r1r1_a1_dor"))
            mesh.Instances.Clear();
    }
}
```

适用场景：类似的门碰撞从同一个 mesh 实例化到多处。

### 3. 按材质 / 高度 / 坐标标记部分三角形为不可走

遍历 mesh 的三角形，按条件给每个原始三角形打 `ForceUnwalkable` 标记：

```csharp
public override void CustomizeScene(SceneExtractor scene)
{
    foreach (var (key, mesh) in scene.Meshes)
    {
        if (key.StartsWith("bg/ffxiv/sea_s1/fld/s1f1/collision/tr"))
        {
            foreach (var part in mesh.Parts)
            {
                for (var i = 0; i < part.Primitives.Count; i++)
                {
                    var prim = part.Primitives[i];
                    var v1   = part.Vertices[prim.V1];
                    var v2   = part.Vertices[prim.V2];
                    var v3   = part.Vertices[prim.V3];
                    if (v1.Y < 2 && v2.Y < 2 && v3.Y < 2)
                        part.Primitives[i] = prim with { Flags = prim.Flags | SceneExtractor.PrimitiveFlags.ForceUnwalkable };
                }
            }
        }
    }
}
```

这个模式非常常用，可以按：
- **高度**：`v1.Y < 某个值`，比如 Z0134 中拉诺西亚标记 Y<2 的海底三角
- **材质 ID**：`prim.Material == 0x8000`，比如 Z1190 Shaaloani 按材质标记天空捕鱼面
- **坐标范围**：同时判断 X/Y/Z

---

## 场景三：需要添加假碰撞体（填坑、做台阶、堵漏洞）

使用 `SceneExtractor` 的扩展方法直接往场景里塞 box 或 cylinder 碰撞体：

```csharp
using System.Numerics;
using vnavmesh.Navigation.Customizations.Extensions;

public override void CustomizeScene(SceneExtractor scene)
{
    // 插入轴对齐方盒
    scene.InsertAABoxCollider(new Vector3(长半轴, 高半轴, 宽半轴), new(x, y, z));

    // 插入圆柱体
    scene.InsertCylinderCollider(new Vector3(半径, 高半轴, 半径), new(x, y, z));

    // 插入方盒并强制标记为不可走
    scene.InsertAABoxCollider(
        new Vector3(10, 1, 10),
        new Vector3(195, -29, 196),
        SceneExtractor.PrimitiveFlags.ForceUnwalkable
    );

    // 插入圆柱体并标记为不可降落
    scene.InsertCylinderCollider(
        new Vector3(2, 2, 2),
        new Vector3(-40, -8, 225),
        SceneExtractor.PrimitiveFlags.Unlandable
    );
}
```

实用场景：
- 台阶太高，游戏本身没有碰撞体 → 插入一个薄的方盒作为假台阶（Z0155 皇都）
- Boss 房间地面有个洞会掉下去 → 插入 `ForceUnwalkable` 圆柱堵住（Z1242 Yuweyawata）
- 电梯移动后可能导致导航错误 → 插入固定碰撞体替代电梯位置（Z1110 魔科学研究所）

---

## 场景四：某个表面本来能走，但导航网格认为不能走

### 1. 强制标记整个 mesh 为可走

```csharp
public override void CustomizeScene(SceneExtractor scene)
{
    if (scene.Meshes.TryGetValue("bg/ffxiv/roc_r1/fld/r1f1/collision/r1f1_b7_astr1.pcb", out var tower))
    {
        foreach (var inst in tower.Instances)
            inst.ForceSetPrimFlags |= SceneExtractor.PrimitiveFlags.ForceWalkable;
    }
}
```

适用：塔楼门槛可站人但不能降落（Z0155）、Alexandar 启动之章 3 的平板。

### 2. 清除 Unlandable 标记

```csharp
if (scene.Meshes.TryGetValue("bg/ffxiv/wil_w1/fld/w1f4/collision/tr1610.pcb", out var mesh))
{
    foreach (var inst in mesh.Instances)
        inst.ForceClearPrimFlags |= SceneExtractor.PrimitiveFlags.Unlandable;
}
```

适用：某个桥面前的两块三角被标记为不可降落（Z0146 南萨纳兰）。

---

## 场景五：两条路之间有空气墙，或者两个不连通的区域需要传

### 传送连接（Teleport Link）

在 `CustomizeMesh` 中用 `LinkPoints`，适用两个区域完全不同但逻辑上角色能从一端到另一端（比如进门加载、空气墙隔断）：

```csharp
using vnavmesh.Common.Navigation.Mesh.Runtime;

public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
{
    LinkPoints(mesh, new(起点X, 起点Y, 起点Z), new(终点X, 终点Y, 终点Z));
}
```

输入的坐标会投影到最近的导航多边形上，不需要精确悬浮在某个多边形正上方。

### 下落连接（Drop Link）

`LinkDrop` 用于角色从高处跳下 / 掉落的场景，系统自动根据 `landingHint` 向下搜寻落点：

```csharp
public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
{
    LinkDrop(mesh, new(边缘X, 边缘Y, 边缘Z), new(参考落点X, 参考落点Y, 参考落点Z));
}
```

参数说明：`edgePos` 是下落出发点，`landingHint` 是落点的粗略参考坐标——系统会从此坐标开始向下逐级搜索合适的落点多边形。

### 客户端路径连接（ClientPath Link）

`LinkClientPath` 是特殊的传送连接，告诉寻路系统这里需要走游戏内置的客户端路径（比如坐骑、特殊移动）。行为和 `LinkPoints` 类似但会额外引发客户端路径跟随行为。

```csharp
LinkClientPath(mesh, new(x1, y1, z1), new(x2, y2, z2));
```

### 构建时 off-mesh 连接

在 `CustomizeSettings` 中添加 off-mesh 连接，这些连接在 Recast 构建导航网格时嵌入：

```csharp
using DotRecast.Detour;
using vnavmesh.Navigation.Customizations.Extensions;

public override void CustomizeSettings(DtNavMeshCreateParams config)
{
    // 简单的单向跨地形连接
    config.AddOffMeshConnection(new(x1, y1, z1), new(x2, y2, z2));

    // 双向连接，用 Ground 类型
    config.AddOffMeshConnection(
        new(x1, y1, z1),
        new(x2, y2, z2),
        0.5f,           // radius
        true,            // bidirectional
        0,
        NavmeshArea.ManualOffMesh,
        NavmeshPolyFlags.AllTraversable,
        NavmeshOffMeshKind.ManualOffMesh
    );
}
```

适用场景：同一片区域的上下高低差需要局部连接，且坐标已知。

### LinkPoints vs AddOffMeshConnection 何时用哪个？

- `CustomizeMesh` 中的 `LinkPoints` / `LinkDrop`：在导航网格已组装完成后操作，适合"逻辑上关联但在物理空间不连通"的连接（穿门、传送、跨空气墙）。
- `CustomizeSettings` 中的 `AddOffMeshConnection`：在 Recast 构建过程中操作，适合"物理上连通但构建时可能被隔断"的连接（同一场景内的上下坡、平台跳跃）。

两者可以同时存在。对于同一个现实场景的连接（比如走一段坡道同时需要穿一个空气墙门），两种都用。

---

## 场景六：飞行支持控制

```csharp
public override bool IsFlyingSupported(SceneDefinition definition) => false;
```

适用场景：狼狱停泊处（ID 250）虽然 TerritoryIntendedUse 是 1（野外），但其实不能飞。

```csharp
// 禁用飞行（传入 false）
```

---

## 场景七：需要精确控制构建参数（CellSize、AgentRadius 等）

### 快速方式：构造函数中直接赋值

```csharp
public MyCustomization()
{
    Settings.CellSize         = 0.25f;
    Settings.CellHeight       = 0.125f;
    Settings.AgentRadius      = 0.3f;
    Settings.AgentHeight      = 1f;
    Settings.FastBuild        = false;  // 生成细节网格
    Settings.Partitioning     = RcPartition.MONOTONE;
}
```

### 精细方式：重写 CustomizeBuildProfile

```csharp
using vnavmesh.Navigation.Customizations;

public override void CustomizeBuildProfile(SceneDefinition definition, NavmeshBuildProfile profile)
{
    profile.CellSizeOverride       = 0.125f;
    profile.AgentRadiusOverride    = 0.3f;
    profile.PartitioningOverride   = RcPartition.MONOTONE;
}
```

`NavmeshBuildProfile` 中所有可覆盖的字段见：[构建参数速查表](#构建参数速查表)。

---

## 场景八：水体 / 水下不能飞行通过

标记水面三角形为 `FlyThrough`，飞行寻路时忽略这些三角：

```csharp
foreach (var (_, m) in scene.Meshes)
{
    foreach (var p in m.Parts)
    {
        foreach (ref var prim in CollectionsMarshal.AsSpan(p.Primitives))
        {
            var v1 = p.Vertices[prim.V1];
            var v2 = p.Vertices[prim.V2];
            var v3 = p.Vertices[prim.V3];
            if (AlmostEqual(v1.Y, 0) && AlmostEqual(v2.Y, 0) && AlmostEqual(v3.Y, 0))
                prim.Flags |= SceneExtractor.PrimitiveFlags.FlyThrough;
        }
    }
}
```

参考 Z1055 无人岛：检查水面三角的 Y 值来判断是否为水体表面。

---

## 场景九：修改现有碰撞 mesh 的顶点（微调几何体）

对于游戏自带 mesh，可以直接改顶点坐标。比如延长台阶底部、抬高扶手、修改太陡的楼梯：

```csharp
if (scene.Meshes.TryGetValue("bg/ex5/03_ocn_o6/btl/o6b1/collision/o6b1_a5_stc02.pcb", out var mesh))
{
    var verts = CollectionsMarshal.AsSpan(mesh.Parts[221].Vertices);
    verts[8].X  += 1;   // 把顶点向外推
    verts[16].X += 1;
}
```

也可以微调 transform：

```csharp
if (scene.Meshes.TryGetValue("bg/ex5/02_ykt_y6/fld/y6f2/collision/y6f2_x0_tst00.pcb", out var mesh))
    mesh.Instances[0].WorldTransform.M42 += 0.05f; // 整个实例向上平移
```

或者拉高碰撞体高度（把低矮花坛变高让它真的能阻挡）：

```csharp
if (scene.Meshes.TryGetValue("bg/ffxiv/fst_f1/twn/common/collision/f1t0_a0_plnt1.pcb", out var mesh))
{
    foreach (var inst in mesh.Instances)
        inst.WorldTransform.M22 *= 2; // Y 方向拉长一倍
}
```

---

## 场景十：多版本地图（节日活动等不同时期布局）

一些地图的布局在不同游戏版本/节日间会变化。CustomizeMesh 的 `festivalLayers` 参数携带了当前活跃的"节日层"ID：

```csharp
public override void CustomizeMesh(Navmesh mesh, List<uint> festivalLayers)
{
    var festivalVersion = festivalLayers.FirstOrDefault() >> 16;

    if (festivalVersion < 0x06)
        return;  // 这个版本不需要后面的连接

    // 添加版本 0x06 以上的连接
    LinkPoints(mesh, ..., ...);

    if (festivalVersion < 0x0F)
        return;  // 版本不够高，后面不加了

    // 添加版本 0x0F 以上的连接
    // ...
}
```

参考 Z1291 Phaenna、Z1310 Oizys。

---

## 场景十一：对整个场景类型生效（跨地图自定义）

如果需要一套自定义对某种类型的全部地图生效（比如所有副本都用更精细的参数），继承 `SceneNavmeshCustomization` 而不是 `NavmeshCustomization`：

```csharp
using Lumina.Excel.Sheets;
using vnavmesh.Bootstrap;
using vnavmesh.Navigation.Scene;

namespace vnavmesh.Navigation.Customizations.SceneDefinitions;

public class Duty : SceneNavmeshCustomization
{
    public override bool Matches(SceneDefinition definition)
    {
        if (Service.LuminaRow<TerritoryType>(definition.TerritoryID) is not { ContentFinderCondition.RowId: > 0 })
            return false;

        Settings.CellSize    = 0.25f;
        Settings.CellHeight  = 0.125f;
        Settings.AgentRadius = 0.5f;
        return true;
    }
}
```

如果多个 SceneCustomization 之间有先后覆盖关系，用 `[CustomizationScenePriorityAbove(typeof(其他类))]` 控制优先级：

```csharp
[CustomizationScenePriorityAbove(typeof(Duty))]
public class PVPDuty : SceneNavmeshCustomization
{
    public override bool Matches(SceneDefinition definition) =>
        Service.LuminaRow<TerritoryType>(definition.TerritoryID) is { IsPvpZone: true };
}
```

优先级越高越晚执行，在组合链中最外层的效果会覆盖内层。

---

## 调试

### 查看最终导航网格

打开 vnavmesh 主窗口 → 选择 **Navigation Mesh** 标签。

在当前地图位置移动角色，你会看到：
- 当前所在的多边形（高亮显示）
- 多边形连通性
- 不同颜色区分的区域（地面=蓝色、攀爬=黄色、跳跃=绿色等）

### 查看碰撞体

vnavmesh 主窗口 → **Collision** 标签。你可以：
- 浏览所有场景碰撞体（按 mesh key 分组）
- 按 layer / material mask 过滤
- 用鼠标在世界中射线检测（需要开启 raycast 功能）
- 可重点关注 Streamed / Mesh / Box / Cylinder / Sphere / Plane 类型

### 查看提取后的场景几何体

vnavmesh 主窗口 → **Extracted Collision** 标签。这个视图展示的是**已经经过 CustomizeScene 处理后的碰撞体数据**——也就是即将被送入 Recast 构建导航网格的最终数据。你可以确认你的自定义是否正确生效。

### 查看 Recast 构建中间产物

vnavmesh 主窗口 → **Build** 标签（或 **Custom** 子标签）。在这里可以：
- 调整所有 `NavmeshSettings` 参数并重新构建
- 可视化 Solid Heightfield / Compact Heightfield / Contour Set / Poly Mesh / Detail Poly Mesh
- 检查高度场的体素网格（Solid Heightfield）：蓝色是可走，灰色是不可走

这对诊断"为什么台阶/坡道没被识别为可走"特别有用——直接在高度场中看蓝色体素是否延伸到了你期望的位置。

### 导出碰撞体到 .obj 文件

vnavmesh → **Export Obj** 可以将碰撞几何体导出为 Wavefront .obj 格式，方便在 Blender / 3ds Max 等工具中查看和分析。

### 获取坐标

写自定义时需要世界坐标。两种方法：
1. 在 **Navigation Mesh** 标签中，角色当前坐标会显示在界面上。
2. 在 **Collision** 标签中开启射线检测，鼠标悬停会显示射线命中位置。

### 获取 mesh key

在 **Collision** 标签中浏览碰撞体树，mesh key 会显示为完整路径（如 `bg/ffxiv/roc_r1/fld/r1f1/collision/r1f1_b7_astr1.pcb`）。这是你在 `CustomizeScene` 中通过 `scene.Meshes.TryGetValue(key, ...)` 查找 mesh 的 key。

### 调试构建失败

如果自定义写入后导航网格没有预期的变化：
1. 检查 `Version` 是否正确递增了
2. 检查 navigation mesh 标签中是否显示了当前正在使用的自定义类名
3. 强制重建：在 Navigation Mesh 标签中找到 rebuild 按钮
4. 在日志（Dalamud 控制台 /xllog）中搜索 `NavmeshBuilder` 日志

---

## 所有可用的 PrimitiveFlags（碰撞原始标志）

当你在 `CustomizeScene` 中操作碰撞体时，以下标记可以用于控制导航网格的生成：

| 标记 | 效果 |
|------|------|
| `ForceUnwalkable` | 此三角面绝对不可行走 |
| `ForceWalkable` | 强制此三角面可行走（覆盖其他标记） |
| `Unlandable` | 飞行状态可以经过，但不能降落 |
| `FlyThrough` | 飞行时直接穿透，不参与体素化 |
| `Fishable` | 此为可钓鱼水域表面 |
| `None` | 不作强制覆盖 |

`ForceSetPrimFlags` 把标记**写入**到所有实例的原始三角面上。  
`ForceClearPrimFlags` 从所有实例的原始三角面上**移除**指定标记。

---

## 构建参数速查表

以下参数可直接在构造函数中赋值给 `Settings`，或通过 `NavmeshBuildProfile` 覆盖：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `CellSize` | 0.5 | XZ 平面栅格尺寸。值越小精度越高，但构建时间呈指数增长。室内建议 0.125~0.25 |
| `CellHeight` | 0.25 | Y 轴栅格尺寸。通常设为 CellSize 的一半 |
| `AgentHeight` | 3 | 可走区域最小净高（天花板到地面）。低于此值的空间视为不可走 |
| `AgentRadius` | 1 | 角色半径。可走区域会从障碍物内缩此距离。一般设为最大角色半径 |
| `AgentMaxClimb` | 0.7 | 可攀爬的最大台阶高度。台阶超过此值会被视作墙壁 |
| `AgentMaxSlopeDeg` | 55 | 可走的最大坡度（度）。超过此值的斜面不可走 |
| `Filtering` | `LowHangingObstacles \| LedgeSpans \| WalkableLowHeightSpans` | 跨度过滤器组合。关掉 `LedgeSpans` 可以让边界区域保持可走 |
| `RegionMinSize` | 8 | 最小孤立区域大小（体素数）。小于此值的零碎区域被丢弃 |
| `RegionMergeSize` | 20 | 区域合并阈值。小区域会被合并到相邻大区域 |
| `Partitioning` | `WATERSHED` | 分区算法。`MONOTONE` 最快但生成多边形细长；`WATERSHED` 效果最好但可能产生重叠 |
| `PolyMaxEdgeLen` | 0 | 多边形最大边长（0=不限制）。限制长边防止生成极细三角形 |
| `PolyMaxSimplificationError` | 1.1 | 边缘简化允许的最大偏差。越低越贴近原始几何 |
| `PolyMaxVerts` | 6 | 每多边形最多顶点数 |
| `DetailSampleDist` | 6 | 细节网格采样距离。`FastBuild=true` 时此项无效 |
| `FastBuild` | true | 跳过细节网格生成，大幅提升构建速度 |
| `GenerateEdgeClimbLinks` | false | 自动生成攀爬链接 |
| `GenerateEdgeJumpLinks` | false | 自动生成跳跃链接 |
| `GroundTileSize` | 64 | 地面区块目标尺寸（世界单位） |
| `GroundTileCountMax` | 32 | 地面区块每轴数量上限 |
| `VolumeTiles` | [8, 8] | 体积细分方格数（L2 瓷砖数, L3 体素数） |

---

## NavmeshFilter 标志速查

| 标志 | 作用 |
|------|------|
| `LowHangingObstacles` | 把低矮障碍上方标记为可走（让角色能跨过路缘石等小障碍） |
| `LedgeSpans` | 把边缘跨度标记为不可走（防止路径悬挂在边缘上方） |
| `WalkableLowHeightSpans` | 把低矮空间标记为不可走（头顶不够高的地方不能走） |
| `Interiors` | 移除内部/非流形几何体下方的跨度 |

**常见操作**：`Settings.Filtering -= NavmeshFilter.LedgeSpans` 关掉边缘过滤。

---

## PrimitiveFlags 速查（在 CustomizeScene 中使用）

| 标记 | 效果 |
|------|------|
| `ForceUnwalkable` | 强标记为不可走 |
| `ForceWalkable` | 强标记为可走 |
| `Unlandable` | 可飞越，不可降落 |
| `FlyThrough` | 飞行穿透，不参与导航构建 |
| `Fishable` | 可钓鱼水域 |
| `None` | 不作覆盖 |

`ForceSetPrimFlags`：写入标记（增加约束）。  
`ForceClearPrimFlags`：移除标记（放松约束）。

---

## NavmeshPolyFlags 速查（用于 AddOffMeshConnection）

| 标志 | 说明 |
|------|------|
| `Ground` | 地面多边形 |
| `ManualOffMesh` | 手动 off-mesh 连接 |
| `Teleport` | 传送连接 |
| `ClientPath` | 客户端路径连接 |
| `GeneratedClimbDown` | 自动生成的下爬连接 |
| `GeneratedEdgeJump` | 自动生成的边缘跳跃连接 |
| `AllTraversable` | 以上全部可穿越标志组合 |

---

## 自定义类的文件与命名约定

- 每个地图一个文件，放在 `Territories/` 或其子文件夹下（如 `Territories/Overworld/`、`Territories/Town/`）。
- 类名建议含地图 ID，如 `Z0155CoerthasCentralHighlands`。
- `[CustomizationTerritory(地图ID)]` 必须标记在类上。
- 类可以声明为 `internal`。
- `Version` 从 1 开始，每次修改后 +1。
