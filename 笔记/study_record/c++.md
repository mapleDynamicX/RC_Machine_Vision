# study

### `1.` 函数用于接收命令行参数int main(int argc, char **argv)

**argc**

<div>
    类型：int
    含义：表示命令行参数的数量（包括程序名称本身）。
    值范围：至少为 1（当程序不带参数运行时，argc=1，仅包含程序名）。
    示例：
    命令行 ./myapp input.txt -v
    → argc = 3（程序名 + 2 个参数）。
</div>

**argv**

<div>
类型：char **（或等效的 char *argv[]）
含义：指向字符串数组的指针，每个字符串存储一个命令行参数。
数组结构：
    argv[0]：程序名称（路径），如 "./myapp"。
    argv[1] 到 argv[argc-1]：用户输入的参数。
    argv[argc]：固定为 NULL（结束标志）。
</div>

内存布局：

```
argv[0] → "./myapp"
argv[1] → "input.txt"
argv[2] → "-v"
argv[3] → NULL
```
