# third_party 说明

`3DDFA_V2/` 来自官方仓库：

```text
https://github.com/cleardusk/3DDFA_V2
```

当前拉取的提交记录在：

```text
3DDFA_V2_COMMIT.txt
```

本模块对官方代码做了一个 Windows 兼容补丁：

```text
3DDFA_V2/FaceBoxes/utils/nms_wrapper.py
```

原因：官方 `FaceBoxes` 默认导入编译版 `cpu_nms`。在当前 Windows 环境没有编译该扩展时，会退回到官方仓库自带的纯 Python `py_cpu_nms.py`。
