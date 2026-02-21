## **Goal**
- 直接将现有代码中的油门输入映射为0~25以内的数值
- 直接将现有代码中的pitch, roll的joystick输入映射为-5到5的target_acc_w
- 将上面映射出来的数值直接赋给input_acc，例如
```
target_acc_w(5,-5,13)
```
这里5是pitch的输入，-5是roll的输入，13是油门的输入
- target_pitch, target_roll不再直接从joytick处赋值，而是按照controlCore中原有的逻辑进行计算，由target_acc_dash得到
## 代码风格要求
- 内存安全，保持简洁清楚明白
- 
## 测试
- 直接使用四根绳子绑上之后放心去给油门即可
  - 可以悬在半空中
  - 梯子、铝架、桌子加重物！
- 水中时：
  - 其实水下运动时感觉也可以直接给target_acc_w而不是直接去给target_pitch, target_roll?!
