# 15 秒超时导致比赛重复执行的根因

## 1. 现象

节点启动后需要满足两种首次启动方式：15 秒内收到“开始执行任务”则语音启动；未收到则 15 秒自动启动。基线代码在语音成功启动后，完整比赛运行结束又被超时逻辑写入一次开始命令，导致第二遍比赛。

这不是“15 秒太短”或单个 `sleep` 的问题，而是启动消费状态和任务生命周期缺失。

## 2. 根因代码

以下行号指创建本分支前的 `src/competition/scripts/navigation_transport/voice_control_navigation.py` 基线（约第 705-810 行）：

- 705-708：`run()` 创建 `voice_command_wait_start`、15 秒 `wakeup_timeout` 和 `start_time`。
- 710：用 `while not rospy.is_shutdown() and self.running` 持续包住整个比赛逻辑。
- 711-718：唤醒超时路径向 `self.words` 注入开始命令，并设置 `auto_start_used=True`。
- 720-726：任何匹配的 `self.words` 都直接进入完整任务。
- 728：注销语音订阅，但这只阻止新语音消息，不会取消内部超时条件。
- 730-795：完整比赛在同一个外层循环中同步运行，通常远超过 15 秒。
- 799：任务返回后无条件执行 `self.words=None`。
- 801-808：若 `auto_start_used` 仍为 False 且从 706 行起已超过 15 秒，代码再次把开始命令写入 `self.words`。
- 809：`continue` 回到 710 行；下一轮 720-726 再次进入完整任务。

语音回调只设置 `self.words`/唤醒标志。语音成功路径没有在进入任务时消费 `auto_start_used`，所以重复链路是：

```text
语音开始
-> auto_start_used 仍为 False
-> 完整比赛在 while 内运行超过 15 秒
-> 任务结束，self.words 清空
-> 15 秒条件已满足
-> 超时分支再次写入开始命令
-> while 再执行一遍完整比赛
```

若第一次本来就是超时启动，超时路径会先设置 `auto_start_used=True`，因此该故障更容易表现为“语音启动后第二遍自动执行”。

## 3. 次要设计缺陷

- 没有 `WAITING_FOR_START/RUNNING/COMPLETED` 状态，任务完成后仍回到同一个等待循环。
- 没有 `mission_finished` 或终态门禁。
- 没有 `mission_execution_count`，无法检测第二次调用。
- 语音、超时和服务入口没有统一原子检查。
- 没有 `threading.Lock`，14.9 秒语音与 15 秒回调可能竞争。
- 基线超时是循环内比较时间，不是可取消的 oneshot `rospy.Timer`。
- 注销订阅不能取消已排队回调，也不能取消内部超时注入。
- 完成后没有永久保持 `COMPLETED`，也没有限定 reset 只能人工调用。

因此，仅把 15 秒改长、加普通布尔值或在任务末尾 `sleep` 都不能从结构上修复。

## 4. 本分支修复

本分支把所有启动方式统一到一次性入口 `request_mission_start(trigger_source)`，并用 `MissionLifecycle` 的内部锁完成原子状态转换：

1. 只在 `WAITING_FOR_START` 接受请求。
2. 同时检查 `mission_started=false`、`mission_finished=false`、`start_trigger_consumed=false`、无 stop 且 ROS 未关闭。
3. 接受第一个请求时立即设置 `start_trigger_consumed=true`、`mission_started=true`、`mission_execution_count += 1` 和 `STARTING`。
4. 立即取消保存的 15 秒 oneshot Timer。
5. 只创建一个任务线程，并只调用一次 `run_mission_once()`。
6. 任务进入 `RUNNING` 后不再接受语音、超时或 start service。
7. 完成后保持 `COMPLETED`、`mission_finished=true`、`mission_execution_count=1`，不返回等待态、不重建 Timer。

15 秒 Timer 只执行：

```python
self.request_mission_start('timeout')
```

即使 Timer 回调已经进入 ROS 回调队列，也必须经过同一把锁和状态检查。stop、error、completed 和 shutdown 都取消 Timer。

## 5. 与比赛流程的关系

三个场景识别会话不改变启动生命周期。第三次识别只设置 `three_scene_recognitions_finished`；原剩余任务、返航和播报全部完成后才进入 `COMPLETED`。识别失败、语音播放失败或重复的 move_base 成功回调都不能把状态改回 `WAITING_FOR_START`。

## 6. 修改前后行为

| 项目 | 修改前 | 本分支 |
| --- | --- | --- |
| 启动入口 | 语音/循环超时分别写 `self.words` | 全部调用同一原子入口 |
| 15 秒实现 | 外层 `while` 中反复比较 | 保存对象的 oneshot `rospy.Timer` |
| 语音启动消费超时 | 未消费 | 接受首个请求时立即消费并取消 Timer |
| 完整任务 | 外层循环内可再次进入 | 单任务线程、`run_mission_once()` |
| 完成后状态 | 继续循环 | 永久保持 `COMPLETED` |
| 竞争保护 | 无锁 | `threading.Lock`/生命周期锁 |
| 执行次数 | 无计数 | 节点生命周期默认保持 1 |

## 7. 对原功能影响

保留“语音成功则启动，否则 15 秒自动启动”需求；不改变导航、夹取、放置、坡面和语音识别内容。变化仅是首个有效触发获胜，其他触发被明确拒绝，任务完成后不会自动跑第二遍。

调试 reset 只能在 `COMPLETED/STOPPED/ERROR`、没有活动任务线程且底盘已停止时人工调用。正式比赛不会自动 reset。

## 8. 验证要求

自动测试必须覆盖：第 5 秒语音、纯 15 秒超时、14.9 秒竞争、多次语音、运行中再次启动、完成后语音/服务、完成后等待至少 30 秒、stop/error/shutdown 取消 Timer，并断言 `mission_execution_count == 1`。

Windows 上可验证无 ROS 依赖的生命周期单元测试；真实 `rospy.Timer`、语音话题并发和完成后 30 秒 ROS 集成测试需要小车实机确认。

## 9. 回滚

若必须回滚，可恢复本分支前的主程序和生命周期模块，但会重新暴露已确认的二次启动缺陷。不要只回滚锁或 Timer 的一部分；启动入口、终态保持和任务线程是一组不可拆分的修复。
