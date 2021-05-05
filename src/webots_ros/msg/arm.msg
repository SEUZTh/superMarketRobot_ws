string type #6种模式 normal、preGrab、grab、uplift、down2up、down2down
bool grabFlag #是否抓取 true表示抓取 false表示松开
#以下变量针对于normal模式
bool[5] operateFlag #是否操作对应机械臂
float64[5] angle #对应位置的角度


