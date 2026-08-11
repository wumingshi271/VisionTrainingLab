# Task01 ArmorPoseSolver 接口设计文档

## 1. 模块目标

任务一提供一个独立的位姿解算模块，模块名称为 **ArmorPoseSolver**。

调用者提供：

- 相机内参；
- 相机畸变参数；
- 小装甲板和大装甲板的真实尺寸；
- 图像中的四个装甲板角点；
- 当前装甲板类型。

模块返回：

- 位姿是否有效；
- 旋转向量；
- 平移向量；
- 旋转矩阵；
- 装甲板到相机的距离；
- yaw、pitch、roll；
- 重投影误差；
- 失败时的错误说明。

模块只负责位姿解算，不负责装甲板检测、YAML 文件读取、命令行解析、窗口显示或日志格式化，这几个功能集成在了主函数当中

## 2. 模块边界与接口接缝

ArmorPoseSolver 的公共接口位于：

```
include/task01/armor_pose_solver.hpp
```

接口接缝的设计目标是：调用者只需要了解少量类型和方法，就能得到完整的位姿结果。

实现位于：

```
src/task01/armor_pose_solver.cpp
```

实现负责隐藏以下复杂性：

- 由装甲板类型选择物理尺寸；
- 生成三维模型角点；
- PnP 求解；
- 旋转向量和旋转矩阵转换；
- 欧拉角计算；
- 重投影误差计算；
- 输入和结果合法性检查。

运行适配器位于：

```
src/task01/task01_main.cpp
```

运行适配器负责组织程序流程，但不应重新实现位姿解算逻辑。


| 模块或文件                 | 角色     | 允许承担的职责         |
| --------------------- | ------ | --------------- |
| ArmorPoseSolver       | 深模块    | 提供小接口，隐藏位姿解算复杂性 |
| armor_pose_solver.hpp | 公共接口接缝 | 对外公布类型、方法和契约    |
| armor_pose_solver.cpp | 实现     | 完成数学计算和结果整理     |
| task01_main.cpp       | 运行适配器  | 读取配置、调用模块、输出结果  |
| configs               | 配置输入   | 保存相机参数和装甲板尺寸    |
| data                  | 样例输入输出 | 保存待解算的角点和装甲板类型  |
| docs                  | 设计文档   | 记录接口和架构，不参与编译   |


核心依赖关系如下：

```
配置适配器和角点输入
→ ArmorPoseSolver
→ OpenCV 的标定和几何计算能力
→ PoseResult
```

ArmorPoseSolver 不反向依赖 task01_main，也不关心是谁调用了它。

## 3. 公共命名空间和接口清单

公共命名空间为 **task01**。

任务一的公共接口只包含四个核心名称：


| 名称              | 类型角色 | 设计目的          |
| --------------- | ---- | ------------- |
| ArmorType       | 枚举类型 | 区分小装甲板和大装甲板   |
| ArmorSize       | 数据类型 | 表示装甲板真实尺寸     |
| PoseResult      | 结果类型 | 统一承载一次解算的全部结果 |
| ArmorPoseSolver | 模块类型 | 根据角点和标定参数解算位姿 |


ArmorPoseSolver 的公共方法为：


| 方法名称            | 输入                | 输出         | 设计说明        |
| --------------- | ----------------- | ---------- | ----------- |
| ArmorPoseSolver | 相机内参、畸变参数、两种装甲板尺寸 | 已配置的解算模块   | 构造时完成长期配置绑定 |
| solve           | 四个图像角点、装甲板类型      | PoseResult | 最主要的业务接口    |
| getObjectPoints | 装甲板类型             | 四个三维模型角点   | 为测试和模型检查提供  |


除上述内容外，不向调用者公开 PnP、欧拉角和误差计算的内部步骤。

## 4. ArmorType 接口设计

**ArmorType** 是装甲板类型枚举。

支持的枚举值：


| 枚举值   | 含义   |
| ----- | ---- |
| SMALL | 小装甲板 |
| LARGE | 大装甲板 |


使用枚举而不是字符串，能够避免大小写不一致和拼写错误。

装甲板类型只表示尺寸类别，不包含检测置信度、目标编号或跟踪状态。

## 5. ArmorSize 接口设计

**ArmorSize** 表示装甲板的真实物理尺寸。


| 字段名称   | 类型     | 单位    | 约束    |
| ------ | ------ | ----- | ----- |
| width  | double | meter | 必须大于零 |
| height | double | meter | 必须大于零 |


所有三维模型点和位姿平移量统一使用 meter。

尺寸单位必须在配置适配器中统一转换。ArmorPoseSolver 不负责猜测输入是毫米还是米。

## 6. PoseResult 接口设计

**PoseResult** 把一次位姿解算的输出集中返回。


| 字段名称                  | 类型                | 单位     | 设计含义                  |
| --------------------- | ----------------- | ------ | --------------------- |
| success               | bool              | 无      | 表示结果是否通过合法性检查         |
| rvec                  | cv::Vec3d         | rad    | OpenCV Rodrigues 旋转向量 |
| tvec                  | cv::Vec3d         | meter  | 装甲板原点在相机坐标系中的位置       |
| rotation_matrix       | cv::Matx33d       | 无      | 由旋转向量得到的旋转矩阵          |
| distance              | double            | meter  | 相机原点到装甲板原点的欧氏距离       |
| yaw_deg               | double            | degree | 偏航角                   |
| pitch_deg             | double            | degree | 俯仰角                   |
| roll_deg              | double            | degree | 横滚角                   |
| reprojection_error_px | double            | pixel  | 四个角点的重投影 RMSE         |
| reprojected_points    | vector of Point2f | pixel  | 根据解算结果重新投影的四个点        |
| error_message         | string            | 无      | 失败时的可读原因              |




### 6.1 成功结果约束

当 success 为 true 时：

- rvec、tvec 和 rotation_matrix 必须是有限数；
- tvec 的 z 分量必须表示目标位于相机前方；
- distance 必须为有限的非负数；
- reprojected_points 必须包含四个点；
- reprojection_error_px 必须是有限的非负数；
- error_message 为空或仅包含非错误提示。



### 6.2 失败结果约束

当 success 为 false 时：

- error_message 必须说明失败阶段或失败原因；
- 调用者不得使用其他位姿字段作为有效测量；
- 失败结果仍应保持结构完整，避免调用者处理未初始化数据；
- 单帧角点错误不应破坏已经创建的解算模块。

PoseResult 是值语义结果，不通过输出参数修改调用者的数据。

## 7. ArmorPoseSolver 构造接口

构造接口名称为 **ArmorPoseSolver**。

构造输入：


| 输入名称              | 类型        | 设计约束           |
| ----------------- | --------- | -------------- |
| camera_matrix     | cv::Mat   | 3×3 相机内参矩阵     |
| distortion_coeffs | cv::Mat   | 畸变参数；允许空值表示无畸变 |
| small_armor       | ArmorSize | 小装甲板尺寸         |
| large_armor       | ArmorSize | 大装甲板尺寸         |


构造接口负责绑定模块运行期间不会频繁变化的配置。

构造时应检查：

- 相机内参维度是否为 3×3；
- 内参和畸变参数是否为有限数；
- 相机焦距是否为有效正值；
- 畸变参数的形状是否能被 OpenCV 接受；
- 两种装甲板的宽度和高度是否均大于零；
- 所有尺寸是否使用 meter。

模块应保存配置的内部副本，不依赖调用者后续修改的 Mat 对象。

配置错误属于构造阶段错误。建议通过标准参数异常报告，避免创建一个无法工作的解算模块。

ArmorPoseSolver 不提供运行中的配置修改方法。这样可以保证同一个模块实例的计算规则保持一致，也避免一半调用使用旧配置、一半调用使用新配置。

## 8. solve 接口设计

核心方法名称为 **solve**。

接口关系为：

```
图像角点 + ArmorType
→ solve
→ PoseResult
```



### 8.1 输入 image_points

输入类型为四个二维图像点。

必须满足：

- 点数量严格为四个；
- 坐标均为有限数；
- 坐标单位为 pixel；
- 顺序固定为 LT、RT、RB、LB；
- 点不能组成面积为零或接近零的四边形。

点顺序定义如下：


| 索引  | 名称  | 含义               |
| --- | --- | ---------------- |
| 0   | LT  | left-top，左上角     |
| 1   | RT  | right-top，右上角    |
| 2   | RB  | right-bottom，右下角 |
| 3   | LB  | left-bottom，左下角  |


同一个顺序必须同时用于图像点和三维模型点。

### 8.2 输入 armor_type

输入类型为 ArmorType。

- SMALL 使用构造时提供的小装甲板尺寸；
- LARGE 使用构造时提供的大装甲板尺寸。

调用者不直接传入宽度和高度，避免一次调用中出现类型与尺寸不匹配。

### 8.3 solve 的输出

solve 始终返回 PoseResult。

调用者首先检查 success，再读取位姿和误差字段。

solve 不负责：

- 写文件；
- 打开或关闭窗口；
- 打印日志；
- 修改输入角点；
- 维护跨帧状态；
- 进行目标跟踪。

因此它是无外部副作用的同步计算接口，适合被主程序、测试程序或后续任务复用。

## 9. getObjectPoints 接口设计

辅助方法名称为 **getObjectPoints**。

接口关系为：

```
ArmorType
→ getObjectPoints
→ 四个三维模型角点
```

返回点的顺序与 solve 要求的图像点顺序完全一致：

```
LT → RT → RB → LB。
```

三维模型坐标约定如下：

- 原点位于装甲板中心；
- X 正方向指向装甲板右侧；
- Y 正方向指向装甲板下方；
- Z 为零，四个点位于装甲板平面；
- 坐标单位为 meter。

该方法主要服务于：

- 验证小、大装甲板的模型尺寸；
- 构造合成测试数据；
- 检查图像点与模型点的对应关系；
- 为外部可视化适配器提供模型点。

普通业务调用只需要 solve。getObjectPoints 是用于测试和模型检查的扩展接口。

## 10. 坐标系和单位契约



### 10.1 图像坐标系

- 原点位于图像左上角；
- X 正方向向右；
- Y 正方向向下；
- 坐标单位为 pixel。



### 10.2 装甲板坐标系

- 原点位于装甲板中心；
- X 正方向向右；
- Y 正方向向下；
- Z 轴垂直于装甲板平面；
- 模型坐标单位为 meter。



### 10.3 相机坐标系

采用 OpenCV 常用约定：

- X 正方向向右；
- Y 正方向向下；
- Z 正方向指向相机前方；
- tvec 表示装甲板原点在相机坐标系中的位置。

位姿变换的语义为：装甲板坐标系中的点经过旋转和平移后，得到相机坐标系中的点。

### 10.4 角度和误差

- rvec 使用弧度制旋转向量；
- yaw_deg、pitch_deg、roll_deg 使用角度制；
- reprojection_error_px 使用像素；
- distance 和 tvec 使用 meter。

欧拉角采用 yaw、pitch、roll 的固定旋转顺序约定。旋转矩阵是更基础、更稳定的姿态表达；接近欧拉角奇异位置时，调用者应优先使用 rotation_matrix。

## 11. 错误处理契约

错误分为配置错误和单次解算错误。

### 11.1 配置错误

以下错误在构造阶段发现：

- 相机内参维度错误；
- 相机参数包含 NaN 或 Inf；
- 畸变参数格式不可用；
- 装甲板尺寸非正数；
- 配置单位无法转换为 meter。

构造失败后不应产生可用的 ArmorPoseSolver 实例。

### 11.2 单次解算错误

以下错误通过 success 为 false 的 PoseResult 返回：

- 角点数量不是四个；
- 角点包含 NaN 或 Inf；
- 四边形退化；
- PnP 没有得到有效解；
- 目标位于相机后方；
- 平移量、旋转量或重投影结果不是有限数；
- 重投影误差无法计算。

error_message 应让运行适配器能够直接记录或展示，不要求调用者理解 OpenCV 的内部异常文本。

## 12. 实现内部职责

以下名称只属于实现内部，不是公共接口承诺：


| 内部职责名称                       | 责任                |
| ---------------------------- | ----------------- |
| getArmorSize                 | 根据 ArmorType 选择尺寸 |
| validateImagePoints          | 验证点数量、有限性和几何退化    |
| rotationMatrixToEulerDegrees | 将旋转矩阵转换为展示用欧拉角    |
| calculateReprojectionError   | 生成重投影点并计算误差       |


这些内部职责可以在实现中拆分，也可以合并。

调用者不应依赖它们的名称、参数或执行顺序。这样可以保持模块的深度，让 PnP 和误差计算的变化集中在一个实现位置。

## 13. 配置文件接口

配置文件为：

**configs/task01_pose_solver.yaml**

配置适配器负责读取文件，并将文件内容转换为 ArmorPoseSolver 的构造输入。


| 配置键                | 内容       | 约束         |
| ------------------ | -------- | ---------- |
| camera_matrix      | 九个相机内参数值 | 适配为 3×3 矩阵 |
| distort_coeffs     | 畸变参数序列   | 空序列表示无畸变   |
| armor.unit         | 尺寸单位     | 设计值为 meter |
| armor.small_width  | 小装甲板宽度   | 大于零        |
| armor.small_height | 小装甲板高度   | 大于零        |
| armor.large_width  | 大装甲板宽度   | 大于零        |
| armor.large_height | 大装甲板高度   | 大于零        |


YAML 读取不属于 ArmorPoseSolver 的职责。

配置适配器应在创建 Solver 之前完成字段存在性、类型、单位和数值范围检查。文件错误应在进入位姿解算前报告。

## 14. 样例数据接口

样例数据文件为：

**data/task01_sample_points.yaml**


| 数据键               | 内容            | 约束               |
| ----------------- | ------------- | ---------------- |
| sample.armor_type | small 或 large | 必须能映射到 ArmorType |
| sample.corners    | 四个二维点         | 顺序为 LT、RT、RB、LB  |


样例数据只表达一次解算所需的输入，不重复保存相机参数和装甲板尺寸。

数据适配器负责：

- 将 small 或 large 转换为 ArmorType；
- 将四组坐标转换为图像点；
- 检查点数量和基本格式；
- 将数据交给 solve。



## 15. 运行入口的接口设计

task01_main 是运行适配器，不是位姿模块的一部分。

它的职责顺序为：

1. 接收配置文件路径和样例数据路径；
2. 读取并验证配置；
3. 读取并验证样例角点；
4. 创建 ArmorPoseSolver；
5. 调用 solve；
6. 根据 PoseResult 输出结果；
7. 在需要时执行测试或可视化。

task01_main 不应：

- 直接调用 PnP；
- 自己计算距离；
- 自己转换欧拉角；
- 自己重新计算重投影误差；
- 修改 Solver 的内部配置。

这种分工使运行适配器可以替换，而不影响位姿模块。

## 16. CMake 设计思路

本项目当前遵循以下构建约束：

- 使用仓库规定的 CMake 4.2；
- 使用 C++14；
- 顶层 CMakeLists.txt 作为构建入口；
- 目标程序名称为 VisionTrainingLab；
- build 或 cmake-build-debug 等目录只保存生成物，不属于源代码。

本节只描述构建架构，不提供 CMake 代码。

### 16.1 顶层构建职责

顶层 CMakeLists.txt 负责：

- 声明项目和 C++ 语言标准；
- 发现 OpenCV 依赖；
- 为目标提供公共头文件搜索路径；
- 收集任务一的头文件、实现文件和运行入口；
- 创建 VisionTrainingLab 构建目标；
- 将 OpenCV 依赖链接到需要它的目标。

顶层构建文件不负责：

- 读取 YAML；
- 检查角点；
- 选择装甲板尺寸；
- 执行 PnP；
- 编写运行日志。



### 16.2 目标组成

当前阶段采用一个可执行目标，组成关系如下：

```
VisionTrainingLab
├── task01_main.cpp：运行入口和适配流程
├── armor_pose_solver.cpp：位姿模块实现
└── armor_pose_solver.hpp：公共接口说明
```

头文件被纳入目标是为了方便 IDE 和构建系统展示依赖，但真正的编译实现来自 cpp 文件。

运行时使用的 configs 和 data 文件不应被当作 C++ 源文件编译。它们通过运行路径被读取，文档也不参与构建。

### 16.3 OpenCV 依赖分层

ArmorPoseSolver 的直接依赖主要是：

- OpenCV core：矩阵、向量和点类型；
- OpenCV calib3d：PnP、旋转转换和投影计算。

如果运行入口保留图像显示，还需要由运行适配器承担 highgui 依赖。

如果运行入口需要额外的绘图能力，再由运行适配器承担 imgproc 依赖。这样位姿模块不会因为显示功能而扩大自己的依赖。

### 16.4 依赖方向

依赖方向必须保持单向：

```
VisionTrainingLab
→ task01_main
→ ArmorPoseSolver
→ OpenCV
```

配置适配器和可视化适配器由 task01_main 使用。

ArmorPoseSolver 不依赖 task01_main，也不依赖具体 YAML 文件。未来更换为其他入口、测试程序或机器人节点时，仍可复用同一个模块。

### 16.5 任务扩展时的演进

当任务数量增加时，可以将 armor_pose_solver.cpp 提取为独立的任务一静态库目标。

演进后的关系为：

- task01_pose_solver：只包含位姿模块实现；
- VisionTrainingLab：链接 task01_pose_solver，负责运行；
- 后续测试目标：链接 task01_pose_solver，直接通过公共接口测试；
- 后续任务目标：按需链接 task01_pose_solver。

此时顶层 CMake 负责组织子目录和目标，任务目录的构建描述负责任务一自身的源文件与依赖。

是否拆分独立库应以实际出现第二个适配器为依据。只有一个调用者时，保持简单的单目标结构；出现测试目标或后续任务复用时，再建立独立库接缝。

### 16.6 头文件可见性

include/task01 是公共接口目录，应该提供给使用 ArmorPoseSolver 的目标。

src/task01 是实现目录，不应作为公共头文件搜索路径暴露给其他任务。

这种目录和依赖安排能把实现细节留在模块内部，减少调用者对文件布局的依赖。

## 17. 测试接口设计

当前仓库没有测试框架和 CTest 目标。

未来测试应只通过 ArmorPoseSolver 的公共接口验证行为。接口本身就是测试接缝，不需要读取 cpp 文件内部状态。

测试范围至少包括：

- 小装甲板和大装甲板的正常输入；
- 四角点顺序为 LT、RT、RB、LB；
- 角点数量错误；
- 角点包含非有限值；
- 四边形退化；
- 相机配置不完整或尺寸非法；
- PnP 失败或目标位于相机后方；
- 重投影误差为有限值；
- getObjectPoints 的点序与尺寸；
- 单位和坐标系的一致性。

测试数据应保持确定性。合成测试可以使用 getObjectPoints 生成已知模型输入，再通过公开的 solve 检查结果。

当正式增加测试目标时，测试目标只需要链接 task01_pose_solver，不应复制位姿实现。

## 18. 接口使用原则

调用者只需要记住以下关系：

ArmorType
表示解算哪一种装甲板。

ArmorSize
表示构造模块所需的真实尺寸。

ArmorPoseSolver
保存相机和尺寸配置，并提供位姿解算能力。

PoseResult
承载一次调用的完整结果。

正常业务路径只依赖：

- 创建 ArmorPoseSolver；
- 调用 solve；
- 检查 PoseResult.success；
- 读取结果字段。

getObjectPoints 只用于模型检查、合成测试和可视化适配。

## 19. 最终架构总结

任务一的公共接缝为：

```
include/task01/armor_pose_solver.hpp
```

公共模块为：

```
task01::ArmorPoseSolver
```

公共数据类型为：

```
task01::ArmorType、task01::ArmorSize、task01::PoseResult
```

公共计算入口为：

```
solve
```

公共模型点查询入口为：

```
getObjectPoints
```

最终的数据流为：

```
配置文件
→ 配置适配器
→ ArmorPoseSolver 构造接口
→ 图像角点和 ArmorType
→ solve
→ PoseResult
→ 运行入口输出或后续模块消费
```

最终的工程关系为：

```
公共接口
→ 位姿模块实现
→ 运行适配器
→ VisionTrainingLab
```

CMake 只负责建立这条编译依赖关系，不承载算法实现和运行时业务逻辑。这样可以让接口保持小而稳定，让实现集中在一个模块中，并为测试和后续任务复用留下清晰的接缝。