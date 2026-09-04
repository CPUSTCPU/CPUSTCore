## 改动说明

<!-- 说明问题、根因和采用的方案。 -->

## 影响范围

- [ ] 仅文档或工具
- [ ] 前端 / 分支预测
- [ ] Rename / Issue / Execute / ROB / Writeback
- [ ] 访存系统 / Cache / TLB
- [ ] 顶层接口 / RTL 生成
- [ ] FPGA 面积或时序

## 验证

- [ ] `sbt --no-colors test`
- [ ] `sbt "runMain CPUSTC.GenerateChipLabTop split"`（影响 RTL 时）
- [ ] `git diff --check`
- [ ] 已增加或更新覆盖本改动的测试

请粘贴关键结果，或说明未执行项目的原因：

```text

```

## 性能与时序

<!-- 若不适用请写 N/A；否则给出基线、测试口径、IPC/面积/WNS/TNS 等结果。 -->

## 已知限制

<!-- 列出未覆盖场景、后续工作或兼容性影响。 -->
