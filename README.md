# Overview  
物体認識・把持を管理するマスター  
Actionを通してobject_recognizerで物体認識、object_grasperで物体把持を行う  

![物体認識・把持](https://user-images.githubusercontent.com/33217285/76415666-ee49f280-63dc-11ea-93c2-8c845aeedfe9.png)

# Usage  
物体把持に必要なノードを立ち上げます(RealSenseやdarknetについては省きます)  

    $ roslaunch manipulation manipulation.launch  

マスターからはサービスで立ち上げます  
  |Name|/manipulation|  
  |:---:|:---:|  
  |**Type**|**manipulation/ManipulateSrv**|  
  |**Args**|**target(String)**|  

使用している型は独自srvなのでimportしてください  
MiniPC(NUC):  
    **`from mimi_common_pkg.srv import ManipulateSrv`**  
Jetson:  
    **`from manipulation import ManipulateSrv`**  

例としてこんな感じ  
    **`rospy.ServiceProxy('/manipulation', ManipulateSrv)`**  

# Input and Output  
把持したい物体の名前(darknetに表示される名称)をString型で引数として呼び出して下さい  
物体の名前を`'any'`とした場合は把持可能な物体を適当に把持します  

把持に成功で`True`, 失敗で`False`を返します  

# Other  
object_recognizerのリンクは[こちら](https://github.com/HappyTatsuhito/object_recognizer)  
object_grasperのリンクは[こちら](https://github.com/HappyTatsuhito/object_grasper)
