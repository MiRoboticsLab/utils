对一些通用第三块的封装

具体包括：
1. CyberdogJson: json的封装库，目前采用rapidjson；
该类中API均为static类型，标注类名直接调用即可；
如源码中几处public标注所示，主要分为add、get和serialize三个类型的API；
具体使用方法可以参考test中的json_test.cpp；

