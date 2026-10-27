本目录存放自建消息定义（`.msg`）。新增文件后需在包根目录 `CMakeLists.txt` 的
`add_message_files(FILES ...)` 中列出，再重新 `catkin_make`。
