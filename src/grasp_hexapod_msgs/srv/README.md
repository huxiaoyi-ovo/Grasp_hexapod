本目录存放自建服务定义（`.srv`），`---` 上方为请求字段、下方为响应字段。
新增文件后需在包根目录 `CMakeLists.txt` 的 `add_service_files(FILES ...)` 中列出，
再重新 `catkin_make`。
