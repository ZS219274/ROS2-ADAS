# 第三方库说明

本目录包含项目所需的第三方矩阵库，已打包在项目中，无需单独安装。

## 包含的库

- **Eigen3** (版本 3.4.0): 矩阵运算库（头文件库）
- **OsqpEigen**: OSQP 求解器的 Eigen 接口（头文件库）
- **OSQP**: 二次规划求解器（包含动态库和静态库）
- **QDLDL**: OSQP 的依赖库（头文件）

## 目录结构

```
third_party/
├── include/          # 头文件目录
│   ├── eigen-3.4.0/  # Eigen3 头文件
│   ├── OsqpEigen/    # OsqpEigen 头文件
│   ├── osqp/         # OSQP 头文件
│   └── qdldl/        # QDLDL 头文件
└── lib/              # 库文件目录
    ├── libosqp.so    # OSQP 动态库
    └── libosqpstatic.a  # OSQP 静态库
```

## 使用方法

这些库已通过 CMake 配置文件自动配置，无需手动设置。CMakeLists.txt 会自动查找并使用这些库。

## 注意事项

- 这些库是从 `/usr/local/include` 和 `/usr/local/lib` 复制而来
- 确保在编译时这些库文件可访问
- 如果遇到链接问题，请检查库文件路径是否正确

