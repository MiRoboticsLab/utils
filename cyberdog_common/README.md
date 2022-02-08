# Cyberdog基础功能API封装

**具体包括：**

- json
- toml



## Json

1. 采用 [rapidjson](https://github.com/Tencent/rapidjson) , V1.1.0

2.  代码中使用json，只需引入头文件即可，封装的API位于该头文件中：

   cmakelsit.txt:

   ```cmake
   find_package(rapidjson REQUIRED)
   
   ament_target_dependencies(${your_executable} rapidjson)
   ```

   package.xml:

   ```xml
   <depend>rapidjson</depend>
   ```

   source code:

   ```c++
   #include "cyberdog_common/cyberdog_json.hpp"
   
   # 另外，全部功能位于命名空间cyberdog::common之内.
   ```

3. 该封装只做了序列化场景的封装，并非全部API，这点要注意。主要包括Add(修改值), Get(获取值), ToString（序列化）, FromString(反序列化), ToFile(存储), FromFile(读取).

4. 若要进一步或直接使用rapidjson, 其源码位于"cyberdog_ws/third_party/rapidjson"，但不推荐这么做。如果发生第三方库的更换，其回归工作量可能较大。

5. 为了做基本的隔离，请使用"json"代替"rapidjson"命名空间

   ```c++
   # cyberdog_json.hpp
   namespace cyberdog
   {
   namespace common
   {
   /* 要求使用json代替rapidjson命名空间 */
   namespace json = rapidjson;
   }  // namespace common
   }  // namespace cyberdog
   
   # 使用时
   cyberdog::common::json Document d(cyberdog::common::json::kObjectType);
   # 当然你可以使用自己的别名，例如：
   namespace json = cyberdog::common::json;
   ```

6. 基本应用请参考json_test.cpp.

7. 更多参考：

   [rapidjson教程](http://rapidjson.org/zh-cn/index.html)

   [json协议规范](https://www.rfc-editor.org/info/rfc7159)



## Toml

1. 采用 [toml11](https://github.com/ToruNiina/toml11) , V3.7.0;

2. 引入方式同json：

3.  代码中使用json，只需引入头文件即可，封装的API位于该头文件中：

   cmakelsit.txt:

   ```cmake
   find_package(toml REQUIRED)
   
   ament_target_dependencies(${your_executable} toml)
   ```

   package.xml:

   ```xml
   <depend>toml</depend>
   ```

   source code:

   ```c++
   #include "cyberdog_common/cyberdog_toml.hpp"
   
   # 另外，全部功能位于命名空间cyberdog::common之内.
   ```

4. 若要进一步或直接使用toml11, 其源码位于"cyberdog_ws/third_party/toml"，但不推荐这么做。如果发生第三方库的更换，其回归工作量可能较大。

5. 基本应用请参考json_test.cpp.

6. 更多参考：

   [toml11](https://github.com/ToruNiina/toml11)

   [toml协议](https://github.com/toml-lang/toml)

   [toml配置入门](https://mojotv.cn/2018/12/26/what-is-toml)

